/*
This file is part of BCP-MAPF.

BCP-MAPF is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

BCP-MAPF is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with BCP-MAPF.  If not, see <https://www.gnu.org/licenses/>.

Author: Edward Lam <ed@ed-lam.com>
*/

//#define PRINT_DEBUG

#include "Pricer_TruffleHog.h"
#include "Includes.h"
#include "ProblemData.h"
#include "VariableData.h"
#include "scip/cons_setppc.h"
#include "ConstraintHandler_VertexConflicts.h"
#include "ConstraintHandler_EdgeConflicts.h"
#include "Constraint_VertexBranching.h"
//#include "Constraint_WaitBranching.h"
#include "Constraint_LengthBranching.h"
#include <chrono>
#include <numeric>

#include "trufflehog/Instance.h"
#include "trufflehog/AStar.h"

// Pricer properties
#define PRICER_NAME     "trufflehog"
#define PRICER_DESC     "Truffle Hog pricer"
#define PRICER_PRIORITY 0
#define PRICER_DELAY    TRUE    // Only call pricer if all problem variables have non-negative reduced costs

#define EPS (1e-6)
#define STALLED_NB_ROUNDS (4)
#define STALLED_ABSOLUTE_CHANGE (-1)

struct PricingOrder
{
    Agent a;
    bool must_price;
    SCIP_VAR* new_var;
};

// Pricer data
struct SCIP_PricerData
{
    SCIP_CONSHDLR* vertex_branching_conshdlr;           // Constraint handler for vertex branching
//    SCIP_CONSHDLR* wait_branching_conshdlr;           // Constraint handler for wait branching
    SCIP_CONSHDLR* length_branching_conshdlr;           // Constraint handler for length branching
    Agent N;                                            // Number of agents

    SCIP_Real* agent_part_dual;                         // Dual variable values of agent set partition constraints
    SCIP_Real* price_priority;                          // Pricing priority of each agent
    bool* agent_priced;                                 // Indicates if an agent is priced in the current round
    PricingOrder* order;                                // Order of agents to price

#ifdef USE_ASTAR_SOLUTION_CACHING
    Vector<AStar::Data> previous_data;                  // Inputs to the previous run for an agent
#endif

    SCIP_Longint last_solved_node;                      // Node number of the last node pricing
    SCIP_Real last_solved_lp_obj[STALLED_NB_ROUNDS];    // LP objective in the last few rounds of pricing
};

// Initialize pricer (called after the problem was transformed)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wuninitialized"
static
SCIP_DECL_PRICERINIT(pricerTruffleHogInit)
{
    // Check.
    debug_assert(scip);
    debug_assert(pricer);

    // Get problem data.
    auto probdata = SCIPgetProbData(scip);
    debug_assert(probdata);

    // Create pricer data.
    SCIP_PricerData* pricerdata;
    SCIP_CALL(SCIPallocBlockMemory(scip, &pricerdata));
    new (pricerdata) SCIP_PricerData;
    pricerdata->N = SCIPprobdataGetN(probdata);
    pricerdata->last_solved_node = -1;

    // Find constraint handler for branching decisions.
    pricerdata->vertex_branching_conshdlr = SCIPfindConshdlr(scip, "vertex_branching");
    release_assert(pricerdata->vertex_branching_conshdlr,
                   "Constraint handler for vertex branching is missing");
//    pricerdata->wait_branching_conshdlr = SCIPfindConshdlr(scip, "wait_branching");
//    release_assert(pricerdata->wait_branching_conshdlr,
//                   "Constraint handler for wait branching is missing");
    pricerdata->length_branching_conshdlr = SCIPfindConshdlr(scip, "length_branching");
    release_assert(pricerdata->length_branching_conshdlr,
                   "Constraint handler for length branching rule is missing");

    // Create array for dual variable values of agent partition constraints.
    SCIP_CALL(SCIPallocBlockMemoryArray(scip, &pricerdata->agent_part_dual, pricerdata->N));
    // Overwritten in each run. No need for initialisation.

    // Create array for agent priority in pricing.
    SCIP_CALL(SCIPallocBlockMemoryArray(scip, &pricerdata->price_priority, pricerdata->N));
    memset(pricerdata->price_priority, 0, sizeof(SCIP_Real) * pricerdata->N);

    // Create array for priced indicator.
    SCIP_CALL(SCIPallocBlockMemoryArray(scip, &pricerdata->agent_priced, pricerdata->N));
    memset(pricerdata->agent_priced, 0, sizeof(bool) * pricerdata->N);

    // Create array for order of agents to price.
    SCIP_CALL(SCIPallocBlockMemoryArray(scip, &pricerdata->order, pricerdata->N));
    // Overwritten in each run. No need for initialisation.

    // Create space to store the penalties from the previous failed iteration.
#ifdef USE_ASTAR_SOLUTION_CACHING
    pricerdata->previous_data.resize(pricerdata->N);
#endif

    // Set pointer to pricer data.
    SCIPpricerSetData(pricer, pricerdata);
    SCIPprobdataSetPricerData(probdata, pricerdata);

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Free pricer
static
SCIP_DECL_PRICERFREE(pricerTruffleHogFree)
{
    // Check.
    debug_assert(scip);
    debug_assert(pricer);

    // Get pricer data.
    auto pricerdata = SCIPpricerGetData(pricer);
    debug_assert(pricerdata);

    // Deallocate.
    SCIPfreeBlockMemoryArray(scip, &pricerdata->agent_part_dual, pricerdata->N);
    SCIPfreeBlockMemoryArray(scip, &pricerdata->price_priority, pricerdata->N);
    SCIPfreeBlockMemoryArray(scip, &pricerdata->agent_priced, pricerdata->N);
    SCIPfreeBlockMemoryArray(scip, &pricerdata->order, pricerdata->N);
    pricerdata->~SCIP_PricerData();
    SCIPfreeBlockMemory(scip, &pricerdata);

    // Done.
    return SCIP_OKAY;
}

enum class MasterProblemStatus
{
    Infeasible = 0,
    Fractional = 1,
    Integral = 2
};

// Compute ordering of agents to price
MasterProblemStatus calculate_agents_order(
    SCIP* scip,                    // SCIP
    SCIP_PROBDATA* probdata,       // Problem data
    SCIP_PricerData* pricerdata    // Pricer data
)
{
    // Get problem data.
    const auto N = SCIPprobdataGetN(probdata);

    // Get variables.
    const auto& dummy_vars = SCIPprobdataGetDummyVars(probdata);
    const auto& agent_vars = SCIPprobdataGetAgentVars(probdata);

    // Calculate the order of the agents.
    MasterProblemStatus master_lp_status = MasterProblemStatus::Integral;
    auto order = pricerdata->order;
    for (Agent a = 0; a < N; ++a)
    {
        // Must price an agent if it is using an artificial variable.
        bool must_price_agent = false;
        if (SCIPisPositive(scip, SCIPgetSolVal(scip, nullptr, dummy_vars[a])))
        {
            master_lp_status = MasterProblemStatus::Infeasible;
            must_price_agent = true;
        }

        // Must price an agent if it is fractional.
        if (!must_price_agent)
        {
            for (const auto& [var, var_val] : agent_vars[a])
            {
                debug_assert(var);
                debug_assert(var_val == SCIPgetSolVal(scip, nullptr, var));
                if (!SCIPisIntegral(scip, var_val))
                {
                    master_lp_status = std::min(master_lp_status, MasterProblemStatus::Fractional);
                    must_price_agent = true;
                    break;
                }
            }
        }

        // Store.
        order[a] = {a, must_price_agent, nullptr};
    }

    // Price all agents if the master problem solution is integral.
    if (master_lp_status == MasterProblemStatus::Integral)
    {
        for (Agent a = 0; a < N; ++a)
        {
            order[a].must_price = true;
        }
    }

    // Sort.
    {
        auto price_priority = pricerdata->price_priority;
        std::sort(order,
                  order + N,
                  [price_priority](const PricingOrder& a, const PricingOrder& b)
                  {
                      return (a.must_price >  b.must_price) ||
                             (a.must_price == b.must_price && price_priority[a.a] > price_priority[b.a]);
                  });
    }

    // Reset.
    memset(pricerdata->agent_priced, 0, sizeof(bool) * pricerdata->N);

    // Done.
    return master_lp_status;
}

static
SCIP_RETCODE run_trufflehog_pricer(
    SCIP* scip,               // SCIP
    SCIP_PRICER* pricer,      // Pricer
    SCIP_RESULT* result,      // Output result
    SCIP_Bool* stopearly,     // Output flag to indicate early branching is required
    SCIP_Real* lower_bound    // Output lower bound
)
{
    // No Farkas pricing.
    constexpr bool is_farkas = false;

    // Check.
    debug_assert(scip);
    debug_assert(pricer);

    // Get pricer data.
    auto pricerdata = SCIPpricerGetData(pricer);
    debug_assert(pricerdata);

    // Print.
    if constexpr (!is_farkas)
    {
        debugln("Starting pricer for feasible master problem at node {}, depth {}, node LB {}, LP obj {}:",
                SCIPnodeGetNumber(SCIPgetCurrentNode(scip)),
                SCIPgetDepth(scip),
                SCIPgetNodeLowerbound(scip, SCIPgetCurrentNode(scip)),
                SCIPgetLPObjval(scip));
    }
    else
    {
        debugln("Starting pricer for infeasible master problem at node {}, depth {}:",
                SCIPnodeGetNumber(SCIPgetCurrentNode(scip)),
                SCIPgetDepth(scip));
    }

    // Get problem data.
    auto probdata = SCIPgetProbData(scip);
    const auto N = SCIPprobdataGetN(probdata);
    const auto& map = SCIPprobdataGetMap(probdata);
    const auto& agents = SCIPprobdataGetAgentsData(probdata);

    // Update variable values.
    update_variable_values(scip);

    // Create order of agents to solve.
    auto order = pricerdata->order;
    const auto master_lp_status = calculate_agents_order(scip, probdata, pricerdata);
    for (Agent a = 0; a < N; ++a)
    {
        pricerdata->price_priority[a] /= PRICE_PRIORITY_DECAY_FACTOR;
    }

    // Early branching if LP is stalled.
    if constexpr (!is_farkas)
    {
        if (master_lp_status == MasterProblemStatus::Fractional)
        {
            const auto current_node = SCIPnodeGetNumber(SCIPgetCurrentNode(scip));
            if (pricerdata->last_solved_node != current_node)
            {
                constexpr auto nan = std::numeric_limits<SCIP_Real>::quiet_NaN();
                std::fill(pricerdata->last_solved_lp_obj, pricerdata->last_solved_lp_obj + STALLED_NB_ROUNDS, nan);
                pricerdata->last_solved_node = current_node;
            }
            else
            {
                std::memmove(pricerdata->last_solved_lp_obj,
                             pricerdata->last_solved_lp_obj + 1,
                             sizeof(SCIP_Real) * (STALLED_NB_ROUNDS - 1));
                pricerdata->last_solved_lp_obj[STALLED_NB_ROUNDS - 1] = SCIPgetLPObjval(scip);

                debugln("");
                debugln("   LP history: {}", fmt::join(pricerdata->last_solved_lp_obj, pricerdata->last_solved_lp_obj + STALLED_NB_ROUNDS, " "));

                bool stalled = true;
                for (Int idx = 0; idx < STALLED_NB_ROUNDS - 1; ++idx)
                {
                    const auto change = pricerdata->last_solved_lp_obj[idx + 1] - pricerdata->last_solved_lp_obj[idx];
                    debugln("   LP absolute change {}", change);
                    if (!(change <= 0 && change >= STALLED_ABSOLUTE_CHANGE)) // Stalled if change is positive or less than some amount
                    {
                        stalled = false;
                        break;
                    }
                }
                if (stalled)
                {
                    debugln("   LP stalled - skip pricing");
                    *stopearly = true;
                    return SCIP_OKAY;
                }
                else
                {
                    debugln("   LP not stalled - start pricing");
                }
            }
        }
    }

    // Get variables.
    const auto& vars = SCIPprobdataGetVars(probdata);

    // Get constraints.
    const auto& agent_part = SCIPprobdataGetAgentPartConss(probdata);
    const auto& vertex_conflicts_conss = vertex_conflicts_get_constraints(probdata);
    const auto& edge_conflicts_conss = edge_conflicts_get_constraints(probdata);
    const auto& agent_goal_vertex_conflicts = SCIPprobdataGetAgentGoalVertexConflicts(probdata);
#ifdef USE_WAITEDGE_CONFLICTS
    const auto& agent_goal_edge_conflicts = SCIPprobdataGetAgentGoalEdgeConflicts(probdata);
#endif

    // Get cuts.
    const auto& agent_robust_cuts = SCIPprobdataGetAgentRobustCuts(probdata);
#ifdef USE_RECTANGLE_CLIQUE_CONFLICTS
    const auto& rectangle_clique_conflicts_conss = rectangle_clique_conflicts_get_constraints(probdata);
#endif
#ifdef USE_GOAL_CONFLICTS
    const auto& goal_agent_goal_conflicts = SCIPprobdataGetGoalAgentGoalConflicts(probdata);
    const auto& crossing_agent_goal_conflicts = SCIPprobdataGetCrossingAgentGoalConflicts(probdata);
#endif
#ifdef USE_PATH_LENGTH_NOGOODS
    const auto& path_length_nogoods = SCIPprobdataGetPathLengthNogoods(probdata);
#endif

    // Get constraints for branching decisions.
    const auto n_vertex_branching_conss = SCIPconshdlrGetNConss(pricerdata->vertex_branching_conshdlr);
    auto vertex_branching_conss = SCIPconshdlrGetConss(pricerdata->vertex_branching_conshdlr);
    debug_assert(n_vertex_branching_conss == 0 || vertex_branching_conss);
    const auto n_length_branching_conss = SCIPconshdlrGetNConss(pricerdata->length_branching_conshdlr);
    auto length_branching_conss = SCIPconshdlrGetConss(pricerdata->length_branching_conshdlr);
    debug_assert(n_length_branching_conss == 0 || length_branching_conss);

    // Get the low-level solver.
    auto& astar = SCIPprobdataGetAStar(probdata);

    // Get data from the low-level solver.
    auto& [start,
           waypoints,
           goal,
           earliest_goal_time,
           latest_goal_time,
           cost_offset,
           latest_visit_time,
           edge_penalties,
           finish_time_penalties
#ifdef USE_GOAL_CONFLICTS
         , goal_penalties
#endif
    ] = astar.data();

    // Print used paths.
#ifdef PRINT_DEBUG
    print_used_paths(scip);
#endif

    // Use debug solution.
//     {
//         static int iter = 0;
//         ++iter;
//         if (iter == 1)
//         {
//             // Get dual variable values of agent partition constraints.
//             auto agent_part_dual = pricerdata->agent_part_dual;
//             for (Agent a = 0; a < N; ++a)
//             {
//                 // Get the constraint.
//                 auto cons = agent_part[a];
//                 debug_assert(cons);
//
//                 // Check that the constraint is not (locally) disabled/redundant.
//                 debug_assert(SCIPconsIsEnabled(cons));
//
//                 // Check that no variable is fixed to one.
//                 debug_assert(SCIPgetNFixedonesSetppc(scip, cons) == 0);
//
//                 // Store dual value.
//                 agent_part_dual[a] = is_farkas ? SCIPgetDualfarkasSetppc(scip, cons) : SCIPgetDualsolSetppc(scip, cons);
//                 debug_assert(SCIPisGE(scip, agent_part_dual[a], 0.0));
//             }
//
//             // Store input paths.
//             Vector<Vector<Pair<Position,Position>>> input_paths_xy{
//             };
//             Vector<Agent> input_paths_agent(N);
//             std::iota(input_paths_agent.begin(), input_paths_agent.end(), 0);
//
//             // Add each path.
//             for (size_t idx = 0; idx < input_paths_xy.size(); ++idx)
//             {
//                 // Used to indicate whether a border has been added.
//                 constexpr Position padding = 0;
//
//                 // Get the path and agent.
//                 const auto& input_path_xy = input_paths_xy[idx];
//                 const auto a = input_paths_agent[idx];
//                 println("Solving for agent {}", a);
//
//                 // Convert input paths to node IDs.
//                 Vector<Node> input_path(input_path_xy.size());
//                 for (Int idx = 0; idx < static_cast<Int>(input_path_xy.size()); ++idx)
//                 {
//                     const auto [x, y] = input_path_xy[idx];
//                     input_path[idx] = map.get_id(x + padding, y + padding);
//                 }
//                 release_assert(input_path.front() == agents[a].start,
//                                "Path start ({},{}) != agent start ({},{})",
//                                map.get_x(input_path.front()),
//                                map.get_y(input_path.front()),
//                                map.get_x(agents[a].start),
//                                map.get_y(agents[a].start));
//                 release_assert(input_path.back() == agents[a].goal,
//                                "Path end ({},{}) != agent goal ({},{})",
//                                map.get_x(input_path.back()),
//                                map.get_y(input_path.back()),
//                                map.get_x(agents[a].goal),
//                                map.get_y(agents[a].goal));
//
//                 // Search.
//                 astar.preprocess_input();
//                 astar.before_solve(); // TODO: Merge back in.
//                 astar.set_verbose();
//                 const auto [path_vertices, path_cost] = astar.calculate_cost(input_path);
//
//                 release_assert(path_vertices.size() == input_path.size());
//                 for (size_t idx = 0; idx < path_vertices.size(); ++idx)
//                 {
//                     release_assert(path_vertices[idx].n == input_path[idx]);
//                 }
//             }
//             astar.set_verbose(false);
//             *result = SCIP_SUCCESS;
//             return SCIP_OKAY;
//         }
//         else
//         {
//             *result = SCIP_DIDNOTRUN;
//             return SCIP_OKAY;
//         }
//     }

    // Find the makespan.
    Time makespan = 0;
    for (const auto& [var, _] : vars)
    {
        // Get the path length.
        debug_assert(var);
        auto vardata = SCIPvarGetData(var);
        const auto path_length = SCIPvardataGetPathLength(vardata);

        // Store the length of the longest path.
        if (path_length > makespan)
        {
            makespan = path_length;
        }
    }

    // Print dual values.
//#ifdef PRINT_DEBUG
//    print_agent_part_dual(scip, is_farkas);
//    print_vertex_conflicts_dual(scip, is_farkas);
//    print_edge_conflicts_dual(scip, is_farkas);
//    print_two_agent_robust_cuts_dual(scip, is_farkas);
//#ifdef USE_GOAL_CONFLICTS
//    print_goal_conflicts_dual(scip, is_farkas);
//#endif
//#ifdef USE_RECTANGLE_CLIQUE_CONFLICTS
//    print_rectangle_clique_conflicts_dual(scip, is_farkas);
//#endif
//#endif

    // Set up reservation table. Reserve vertices of paths with value 1.
#ifdef USE_RESERVATION_TABLE
    auto& restab = astar.reservation_table();
    restab.clear_reservations();
    for (const auto& [var, var_val] : vars)
    {
        debug_assert(var);
        debug_assert(var_val == SCIPgetSolVal(scip, nullptr, var));
        if (var_val >= 0.5)
        {
            // Get the path.
            auto vardata = SCIPvarGetData(var);
            const auto path_length = SCIPvardataGetPathLength(vardata);
            const auto path = SCIPvardataGetPath(vardata);

            // Update reservation table.
            Node n;
            Time t = 0;
            for (; t < path_length; ++t)
            {
                n = path[t].n;
                restab.reserve(NodeTime{n, t});
            }
            for (; t < makespan; ++t)
            {
                restab.reserve(NodeTime{n, t});
            }
        }
    }
#endif

    // Make edge penalties for all agents.
    EdgePenalties global_edge_penalties;

    // Input dual values for vertex conflicts.
    for (const auto& [nt, vertex_conflict] : vertex_conflicts_conss)
    {
        const auto& [row] = vertex_conflict;
        const auto dual = is_farkas ? SCIProwGetDualfarkas(row) : SCIProwGetDualsol(row);
        debug_assert(SCIPisFeasLE(scip, dual, 0.0));
        if (SCIPisFeasLT(scip, dual, 0.0))
        {
            // Add the dual variable value to the edges leading into the vertex.
            const auto t = nt.t - 1;
            {
                const auto n = map.get_south(nt.n);
                auto& penalties = global_edge_penalties.get_edge_penalties(n, t);
                penalties.north -= dual;
            }
            {
                const auto n = map.get_north(nt.n);
                auto& penalties = global_edge_penalties.get_edge_penalties(n, t);
                penalties.south -= dual;
            }
            {
                const auto n = map.get_west(nt.n);
                auto& penalties = global_edge_penalties.get_edge_penalties(n, t);
                penalties.east -= dual;
            }
            {
                const auto n = map.get_east(nt.n);
                auto& penalties = global_edge_penalties.get_edge_penalties(n, t);
                penalties.west -= dual;
            }
            {
                const auto n = map.get_wait(nt.n);
                auto& penalties = global_edge_penalties.get_edge_penalties(n, t);
                penalties.wait -= dual;
            }
        }
    }

    // Input dual values for edge conflicts.
    for (const auto& [et, edge_conflict] : edge_conflicts_conss)
    {
        const auto& [row, edges, t] = edge_conflict;
        const auto dual = is_farkas ? SCIProwGetDualfarkas(row) : SCIProwGetDualsol(row);
        debug_assert(SCIPisFeasLE(scip, dual, 0.0));
        if (SCIPisFeasLT(scip, dual, 0.0))
        {
            // Add the dual variable value to the edges.
            for (const auto e : edges)
            {
                auto& penalties = global_edge_penalties.get_edge_penalties(e.n, t);
                penalties.d[e.d] -= dual;
            }
        }
    }

    // Price each agent.
    Float min_reduced_cost = 0;
#ifdef PRINT_DEBUG
    Int nb_new_cols = 0;
#endif
    bool found = false;
    auto agent_priced = pricerdata->agent_priced;
    for (Int order_idx = 0;
         order_idx < N && (!found || order[order_idx].must_price) && !SCIPisStopped(scip);
         ++order_idx)
    {
        // Create output.
        Vector<Edge> path;
        Vector<NodeTime> path_vertices;
        SCIP_Real path_cost;

        // Set up start and end points.
        const auto a = order[order_idx].a;
        start = agents[a].start;
        goal = agents[a].goal;

        // Input the agent partition dual.
        {
            // Get the constraint.
            auto cons = agent_part[a];
            debug_assert(cons);

            // Check that the constraint is not (locally) disabled/redundant.
            debug_assert(SCIPconsIsEnabled(cons));

            // Check that no variable is fixed to one.
            debug_assert(SCIPgetNFixedonesSetppc(scip, cons) == 0);

            // Store dual value.
            const auto dual = is_farkas ? SCIPgetDualfarkasSetppc(scip, cons) : SCIPgetDualsolSetppc(scip, cons);
            debug_assert(SCIPisGE(scip, dual, 0.0));
            cost_offset = -dual;
        }

        // Modify edge costs for two-agent robust cuts.
        edge_penalties = global_edge_penalties;
        finish_time_penalties.clear();
#ifdef USE_GOAL_CONFLICTS
        goal_penalties.clear();
#endif
        for (const auto& [row, ets_begin, ets_end] : agent_robust_cuts[a])
        {
            const auto dual = is_farkas ? SCIProwGetDualfarkas(row) : SCIProwGetDualsol(row);
            debug_assert(SCIPisFeasLE(scip, dual, 0.0));
            if (SCIPisFeasLT(scip, dual, 0.0))
            {
                for (auto it = ets_begin; it != ets_end; ++it)
                {
                    // Incur the penalty on the edge.
                    const auto n = it->n;
                    const auto d = it->d;
                    const auto t = it->t;
                    auto& penalties = edge_penalties.get_edge_penalties(n, t);
                    penalties.d[d] -= dual;

                    // If a wait edge in a two-agent robust cut corresponds to waiting at the goal, incur the
                    // penalty for staying at the goal because the low-level solver doesn't traverse this edge.
                    if (n == goal && d == Direction::WAIT)
                    {
                        const auto conflict_time = it->t;
                        finish_time_penalties.add(conflict_time, -dual);
                    }
                }
            }
        }

        // If the node of a vertex conflict is the goal, incur a penalty for waiting (indefinitely) at the goal
        // after the agent has completed its path.
        for (const auto& [t, row] : agent_goal_vertex_conflicts[a])
        {
            const auto dual = is_farkas ? SCIProwGetDualfarkas(row) : SCIProwGetDualsol(row);
            debug_assert(SCIPisFeasLE(scip, dual, 0.0));
            if (SCIPisFeasLT(scip, dual, 0.0))
            {
                // Add the penalty if the agent finishes before time t. The penalty at time t is accounted for in the
                // global edge penalties.
                finish_time_penalties.add(t - 1, -dual);
            }
        }

        // If the wait edge in a wait edge conflict is at the goal, incur a penalty for staying at the goal.
#ifdef USE_WAITEDGE_CONFLICTS
        for (const auto& [t, row] : agent_goal_edge_conflicts[a])
        {
            const auto dual = is_farkas ? SCIProwGetDualfarkas(row) : SCIProwGetDualsol(row);
            debug_assert(SCIPisFeasLE(scip, dual, 0.0));
            if (SCIPisFeasLT(scip, dual, 0.0))
            {
                // If an agent finishes at time t, it does not traverse the edge at time t to t+1. So the agent is
                // penalised here.
                finish_time_penalties.add(t, -dual);
            }
        }
#endif

        // Add goal crossings or finish time penalties for goal conflicts. If agent a1 finishes at or before time t,
        // incur the penalty. If agent a2 crosses the goal of agent a1 at or after time t, incur the penalty.
#ifdef USE_GOAL_CONFLICTS
        for (const auto& [t, row] : goal_agent_goal_conflicts[a])
        {
            const auto dual = is_farkas ? SCIProwGetDualfarkas(row) : SCIProwGetDualsol(row);
            debug_assert(SCIPisFeasLE(scip, dual, 0.0));
            if (SCIPisFeasLT(scip, dual, 0.0))
            {
                finish_time_penalties.add(t, -dual);
            }
        }
        for (const auto& [nt, row] : crossing_agent_goal_conflicts[a])
        {
            const auto dual = is_farkas ? SCIProwGetDualfarkas(row) : SCIProwGetDualsol(row);
            debug_assert(SCIPisFeasLE(scip, dual, 0.0));
            if (SCIPisFeasLT(scip, dual, 0.0))
            {
                goal_penalties.add(nt, -dual);
            }
        }
#endif

        // Modify edge costs for path length nogoods. If agent a finishes at or before time t, incur the penalty.
#ifdef USE_PATH_LENGTH_NOGOODS
        for (const auto& [row, latest_finish_times] : path_length_nogoods)
            for (const auto& [nogood_a, t] : latest_finish_times)
                if (a == nogood_a)
                {
                    const auto dual = is_farkas ? SCIProwGetDualfarkas(row) : SCIProwGetDualsol(row);
                    debug_assert(SCIPisFeasLE(scip, dual, 0.0));
                    if (SCIPisFeasLT(scip, dual, 0.0))
                    {
                        finish_time_penalties.add(t, -dual);
                    }
                }
#endif

        // Modify edge costs for vertex branching decisions.
        waypoints.clear();
        for (Int c = 0; c < n_vertex_branching_conss; ++c)
        {
            // Get the constraint.
            auto cons = vertex_branching_conss[c];
            debug_assert(cons);

            // Ignore constraints that are not active since these are not on the current
            // active path of the search tree.
            if (!SCIPconsIsActive(cons))
                continue;

            // Enforce the decision.
            const auto branch_a = SCIPgetVertexBranchingAgent(cons);
            const auto dir = SCIPgetVertexBranchingDirection(cons);
            const auto nt = SCIPgetVertexBranchingNodeTime(cons);
            if ((a == branch_a && dir == VertexBranchDirection::Forbid) ||
                (a != branch_a && dir == VertexBranchDirection::Use))
            {
                // Don't use the vertex.
                const auto prev_time = nt.t - 1;
                {
                    const auto n = map.get_south(nt.n);
                    auto& penalties = edge_penalties.get_edge_penalties(n, prev_time);
                    penalties.north = std::numeric_limits<Cost>::infinity();
                }
                {
                    const auto n = map.get_north(nt.n);
                    auto& penalties = edge_penalties.get_edge_penalties(n, prev_time);
                    penalties.south = std::numeric_limits<Cost>::infinity();
                }
                {
                    const auto n = map.get_west(nt.n);
                    auto& penalties = edge_penalties.get_edge_penalties(n, prev_time);
                    penalties.east = std::numeric_limits<Cost>::infinity();
                }
                {
                    const auto n = map.get_east(nt.n);
                    auto& penalties = edge_penalties.get_edge_penalties(n, prev_time);
                    penalties.west = std::numeric_limits<Cost>::infinity();
                }
                {
                    const auto n = map.get_wait(nt.n);
                    auto& penalties = edge_penalties.get_edge_penalties(n, prev_time);
                    penalties.wait = std::numeric_limits<Cost>::infinity();
                }
            }
            else if (a == branch_a)
            {
                debug_assert(dir == VertexBranchDirection::Use);

                // Store a waypoint to enforce use of the vertex.
                waypoints.push_back(nt);
            }
        }

        // Sort waypoints by time.
        std::sort(waypoints.begin(), waypoints.end(), [](const auto& a, const auto& b)
        {
            return a.t < b.t;
        });
#ifdef DEBUG
        for (size_t idx = 1; idx < waypoints.size(); ++idx)
        {
            debug_assert(waypoints[idx - 1].t < waypoints[idx].t);
        }
#endif

        // Modify edge costs for length branching decisions.
        debug_assert(astar.max_path_length() >= 1);
        earliest_goal_time = 0;
        latest_goal_time = astar.max_path_length() - 1;
        latest_visit_time = map.latest_visit_time();
        for (Int c = 0; c < n_length_branching_conss; ++c)
        {
            // Get the constraint.
            auto cons = length_branching_conss[c];
            debug_assert(cons);

            // Ignore constraints that are not active since these are not on the current active path of the search tree.
            if (!SCIPconsIsActive(cons))
                continue;

            // Enforce the decision if the same agent. Disable crossing if different agent.
            const auto branch_a = SCIPgetLengthBranchingAgent(cons);
            const auto dir = SCIPgetLengthBranchingDirection(cons);
            const auto nt = SCIPgetLengthBranchingNodeTime(cons);
            if (a == branch_a)
            {
                if (dir == LengthBranchDirection::LEq)
                {
                    latest_goal_time = std::min(latest_goal_time, nt.t);
                }
                else
                {
                    earliest_goal_time = std::max(earliest_goal_time, nt.t);
                }
            }
            else if (dir == LengthBranchDirection::LEq)
            {
                // Block the vertex at time t and onwards.
                latest_visit_time[nt.n] = nt.t - 1;
            }
        }
        debug_assert(waypoints.empty() || latest_goal_time >= waypoints.back().t);

        // Preprocess input data.
        astar.preprocess_input();

        // Start timer.
#ifdef PRINT_DEBUG
        const auto start_time = std::chrono::high_resolution_clock::now();
#endif

        // Skip running A* if the penalties in the last iteration of this agent have stayed the same or worsened.
#ifdef USE_ASTAR_SOLUTION_CACHING
        if (!astar.data().can_be_better(pricerdata->previous_data[a]))
        {
            goto FINISHED_PRICING_AGENT;
        }
#endif

        // Solve.
        astar.before_solve(); // TODO: Merge back in.
#ifdef USE_SIPP
        std::tie(path_vertices, path_cost) = astar.solve_sipp<is_farkas>();
#ifdef DEBUG
        {
            const auto [time_expanded_astar_path_vertices, time_expanded_astar_path_cost] = astar.solve<is_farkas>();
            debug_assert(std::abs(time_expanded_astar_path_cost - path_cost) < 1e-8);
        }
#endif
#else
        std::tie(path_vertices, path_cost) = astar.solve<is_farkas>();
#endif
        if (!path_vertices.empty())
        {
            // Get the path.
            for (auto it = path_vertices.begin(); it != path_vertices.end(); ++it)
            {
                const auto d = it != path_vertices.end() - 1 ?
                               map.get_direction(it->n, (it + 1)->n) :
                               Direction::INVALID;
                path.push_back(Edge{it->n, d});
            }

            // Add a column only if the path has negative reduced cost.
            min_reduced_cost = std::min(min_reduced_cost, path_cost);
            if (SCIPisSumLT(scip, path_cost, 0.0))
            {
                // Print.
                debugln("    Found path with length {}, reduced cost {:.6f} ({})",
                        path.size(),
                        path_cost,
                        format_path(probdata, path.size(), path.data()));

                // Add column.
                SCIP_VAR* var = nullptr;
                SCIP_CALL(SCIPprobdataAddPricedVar(scip, probdata, a, path.size(), path.data(), &var));
                debug_assert(var);
                found = true;
                order[order_idx].new_var = var;
                pricerdata->price_priority[a]++;
#ifdef PRINT_DEBUG
                nb_new_cols++;
#endif

                // Update reservation table.
#ifdef USE_RESERVATION_TABLE
                {
                    Node n;
                    Time t = 0;
                    for (; t < static_cast<Time>(path.size()); ++t)
                    {
                        n = path[t].n;
                        restab.reserve(NodeTime{n, t});
                    }
                    for (; t < makespan; ++t)
                    {
                        restab.reserve(NodeTime{n, t});
                    }
                }
#endif

                // Advance to the next agent.
                goto FINISHED_PRICING_AGENT;
            }
        }

        // Store the penalties of the run.
#ifdef USE_ASTAR_SOLUTION_CACHING
        pricerdata->previous_data[a] = astar.data();
#endif

        // End of this agent.
        FINISHED_PRICING_AGENT:
        agent_priced[a] = true;

        // End timer.
#ifdef PRINT_DEBUG
        const auto end_time = std::chrono::high_resolution_clock::now();
        const auto duration = std::chrono::duration<double>(end_time - start_time).count();
        debugln("    Done in {:.4f} seconds", duration);
#endif
    }

    // Print.
    debugln("Added {} new columns", nb_new_cols);

    // Finish.
    if (!SCIPisStopped(scip))
    {
        // Compute lower bound.
        if constexpr (!is_farkas)
        {
            bool all_agents_priced = true;
            for (Agent a = 0; a < N; ++a)
                if (!agent_priced[a])
                {
                    all_agents_priced = false;
                    break;
                }
            if (all_agents_priced)
            {
                *lower_bound = SCIPgetLPObjval(scip) + N * min_reduced_cost;
                debugln("   Computed lower bound {}", *lower_bound);
            }
        }

        // Mark as completed.
        *result = SCIP_SUCCESS;
    }
    return SCIP_OKAY;
}

// Reduced cost pricing for feasible master problem
static
SCIP_DECL_PRICERREDCOST(pricerTruffleHogRedCost)
{
    return run_trufflehog_pricer(scip, pricer, result, stopearly, lowerbound);
}

// Farkas pricing for infeasible master problem
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_PRICERFARKAS(pricerTruffleHogFarkas)
{
    unreachable();
}
#pragma GCC diagnostic pop

// Create pricer and include it in SCIP
SCIP_RETCODE SCIPincludePricerTruffleHog(
    SCIP* scip    // SCIP
)
{
    // Include pricer.
    SCIP_PRICER* pricer;
    SCIP_CALL(SCIPincludePricerBasic(scip,
                                     &pricer,
                                     PRICER_NAME,
                                     PRICER_DESC,
                                     PRICER_PRIORITY,
                                     PRICER_DELAY,
                                     pricerTruffleHogRedCost,
                                     pricerTruffleHogFarkas,
                                     nullptr));

    // Set callbacks.
    SCIP_CALL(SCIPsetPricerInit(scip, pricer, pricerTruffleHogInit));
    SCIP_CALL(SCIPsetPricerFree(scip, pricer, pricerTruffleHogFree));

    // Done.
    return SCIP_OKAY;
}

// Add problem specific data to the pricer and activate
SCIP_RETCODE SCIPpricerTruffleHogActivate(
    SCIP* scip    // SCIP
)
{
    // Check.
    debug_assert(scip);

    // Get pricer.
    auto pricer = SCIPfindPricer(scip, PRICER_NAME);
    debug_assert(pricer);

    // Activate pricer.
    SCIP_CALL(SCIPactivatePricer(scip, pricer));

    // Done.
    return SCIP_OKAY;
}
