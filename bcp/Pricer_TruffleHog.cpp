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
#define PRICER_NAME               "trufflehog"
#define PRICER_DESC       "Truffle Hog pricer"
#define PRICER_PRIORITY                      0
#define PRICER_DELAY                      TRUE  // Only call pricer if all problem variables have non-negative reduced costs

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
    SCIP_CONSHDLR* vertex_branching_conshdlr;                                      // Constraint handler for vertex branching
//    SCIP_CONSHDLR* wait_branching_conshdlr;                                      // Constraint handler for wait branching
    SCIP_CONSHDLR* length_branching_conshdlr;                                      // Constraint handler for length branching
    Agent N;                                                                       // Number of agents

    SCIP_Real* agent_part_dual;                                                    // Dual variable values of agent set partition constraints
    SCIP_Real* price_priority;                                                     // Pricing priority of each agent
    PricingOrder* order;                                                           // Order of agents to price

#ifdef USE_ASTAR_SOLUTION_CACHING
    Vector<Vector<NodeTime>> previous_segments;                                    // End-points of segments in previous failed iteration
    Vector<Time> previous_earliest_finish;                                         // Earliest time to reach the goal in the previous failed iteration
    Vector<Time> previous_latest_finish;                                           // Latest time to reach the goal in the previous failed iteration
    Vector<Float> previous_agent_part_dual;                                        // Dual variable values of agent set partition constraints in previous failed iteration
    Vector<HashTable<NodeTime, TruffleHog::EdgeCosts>> previous_edge_penalties;    // Edge penalties used in previous failed iteration
    Vector<Vector<Cost>> previous_time_finish_penalties;                           // Time to finish penalties in previous failed iteration
#ifdef USE_GOAL_CONFLICTS
    Vector<Vector<GoalCrossing>> previous_goal_crossings;                          // Goal crossing penalties in previous failed iteration
#endif
#endif

    SCIP_Longint last_solved_node;                                                 // Node number of the last node pricing
    SCIP_Real last_solved_lp_obj[STALLED_NB_ROUNDS];                               // LP objective in the last few rounds of pricing
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

    // Create array for order of agents to price.
    SCIP_CALL(SCIPallocBlockMemoryArray(scip, &pricerdata->order, pricerdata->N));
    // Overwritten in each run. No need for initialisation.

    // Create space to store the penalties from the previous failed iteration.
#ifdef USE_ASTAR_SOLUTION_CACHING
    pricerdata->previous_segments.resize(pricerdata->N);
    pricerdata->previous_earliest_finish.resize(pricerdata->N);
    pricerdata->previous_latest_finish.resize(pricerdata->N);
    pricerdata->previous_agent_part_dual.resize(pricerdata->N, -std::numeric_limits<Float>::infinity());
    pricerdata->previous_edge_penalties.resize(pricerdata->N);
    pricerdata->previous_time_finish_penalties.resize(pricerdata->N);
#ifdef USE_GOAL_CONFLICTS
    pricerdata->previous_goal_crossings.resize(pricerdata->N);
#endif
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
            for (auto var : agent_vars[a])
            {
                // Get the variable value.
                debug_assert(var);
                const auto var_val = SCIPgetSolVal(scip, nullptr, var);

                // Set.
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
    auto& vars = SCIPprobdataGetVars(probdata);

    // Get constraints.
    const auto& agent_part = SCIPprobdataGetAgentPartConss(probdata);
    const auto& vertex_conflicts_conss = vertex_conflicts_get_constraints(probdata);
    const auto& edge_conflicts_conss = edge_conflicts_get_constraints(probdata);

    // Get cuts.
    const auto& two_agent_robust_cuts = SCIPprobdataGetTwoAgentRobustCuts(probdata);
#ifdef USE_GOAL_CONFLICTS
    const auto& goal_conflicts = SCIPprobdataGetGoalConflicts(probdata);
#endif
#ifdef USE_PATH_LENGTH_NOGOODS
    const auto& path_length_nogoods = SCIPprobdataGetPathLengthNogoods(probdata);
#endif
#ifdef USE_RECTANGLE_CLIQUE_CONFLICTS
    const auto& rectangle_clique_conflicts_conss = rectangle_clique_conflicts_get_constraints(probdata);
#endif

    // Get constraints for branching decisions.
    const auto n_vertex_branching_conss = SCIPconshdlrGetNConss(pricerdata->vertex_branching_conshdlr);
    auto vertex_branching_conss = SCIPconshdlrGetConss(pricerdata->vertex_branching_conshdlr);
    debug_assert(n_vertex_branching_conss == 0 || vertex_branching_conss);
//    const auto n_wait_branching_conss = SCIPconshdlrGetNConss(pricerdata->wait_branching_conshdlr);
//    auto wait_branching_conss = SCIPconshdlrGetConss(pricerdata->wait_branching_conshdlr);
//    debug_assert(n_wait_branching_conss == 0 || wait_branching_conss);
    const auto n_length_branching_conss = SCIPconshdlrGetNConss(pricerdata->length_branching_conshdlr);
    auto length_branching_conss = SCIPconshdlrGetConss(pricerdata->length_branching_conshdlr);
    debug_assert(n_length_branching_conss == 0 || length_branching_conss);

    // Get solver.
    auto& astar = SCIPprobdataGetAStar(probdata);
    auto& restab = astar.reservation_table();
    auto& edge_penalties = astar.edge_penalties();
#ifdef USE_GOAL_CONFLICTS
    auto& goal_crossings = astar.goal_crossings();
#endif
#ifdef USE_RECTANGLE_CLIQUE_CONFLICTS
    auto& rectangle_crossings = astar.rectangle_crossings();
#endif

    // Print used paths.
#ifdef PRINT_DEBUG
    print_used_paths(scip);
#endif

    // Use debug solution.
//    {
//        static int iter = 0;
//        ++iter;
//        if (iter == 1)
//        {
//            // Get dual variable values of agent partition constraints.
//            auto agent_part_dual = pricerdata->agent_part_dual;
//            for (Agent a = 0; a < N; ++a)
//            {
//                // Get the constraint.
//                auto cons = agent_part[a];
//                debug_assert(cons);
//
//                // Check that the constraint is not (locally) disabled/redundant.
//                debug_assert(SCIPconsIsEnabled(cons));
//
//                // Check that no variable is fixed to one.
//                debug_assert(SCIPgetNFixedonesSetppc(scip, cons) == 0);
//
//                // Store dual value.
//                agent_part_dual[a] = is_farkas ? SCIPgetDualfarkasSetppc(scip, cons) : SCIPgetDualsolSetppc(scip, cons);
//                debug_assert(SCIPisGE(scip, agent_part_dual[a], 0.0));
//            }
//
//            // Get input paths.
//            Vector<Vector<Pair<Position,Position>>> input_paths_xy{
//            };
//            Vector<Agent> input_paths_agent(N);
//            std::iota(input_paths_agent.begin(), input_paths_agent.end(), 0);
//
//            // Convert input paths to node IDs.
//            Vector<Vector<Edge>> input_paths;
//            for (size_t idx = 0; idx < input_paths_xy.size(); ++idx)
//            {
//                constexpr Position padding = 1;
//
//                const auto& input_path_xy = input_paths_xy[idx];
//                const auto a = input_paths_agent[idx];
//
//                auto& path = input_paths.emplace_back();
//                for (Int idx = 0; idx < static_cast<Int>(input_path_xy.size() - 1); ++idx)
//                {
//                    const auto [x1, y1] = input_path_xy[idx];
//                    const auto [x2, y2] = input_path_xy[idx+1];
//                    const auto i = map.get_id(x1 + padding, y1 + padding);
//                    const auto j = map.get_id(x2 + padding, y2 + padding);
//                    const auto d = map.get_direction(i, j);
//                    path.push_back(Edge{i,d});
//                }
//                {
//                    Int idx = input_path_xy.size() - 1;
//                    const auto [x1, y1] = input_path_xy[idx];
//                    const auto i = map.get_id(x1 + padding, y1 + padding);
//                    const auto d = Direction::INVALID;
//                    path.push_back(Edge{i,d});
//                }
//                release_assert(path.front().n == agents[a].start,
//                               "({},{}) != ({},{})",
//                               map.get_x(path.front().n),
//                               map.get_y(path.front().n),
//                               map.get_x(agents[a].start),
//                               map.get_y(agents[a].start));
//                release_assert(path.back().n == agents[a].goal,
//                               "({},{}) != ({},{})",
//                               map.get_x(path.back().n),
//                               map.get_y(path.back().n),
//                               map.get_x(agents[a].goal),
//                               map.get_y(agents[a].goal));
//            }
//
//            // Search.
////            for (const auto a : Vector<Agent>{38})
//            for (const auto a : input_paths_agent)
//            {
//                const auto input_path = input_paths[a];
//
//                astar.edge_penalties().reset();
//                astar.goal_crossings().clear();
//                astar.set_verbose();
//                const auto [path, path_cost] = astar.calculate_cost<true>(input_path);
//
//                release_assert(path.size() == input_path.size());
//                for (size_t idx = 0; idx < path.size(); ++idx)
//                {
//                    release_assert(path[idx].n == input_path[idx].n);
//                }
//
//                if (path_cost >= agent_part_dual[a])
//                {
//                    continue;
//                }
//
//                const auto agent_vars = SCIPprobdataGetAgentVars(probdata);
//                bool found = false;
//                for (const auto var : agent_vars[a])
//                {
//                    const auto vardata = SCIPvarGetData(var);
//                    const auto path2 = SCIPvardataGetPath(vardata);
//                    const auto path2_length = SCIPvardataGetPathLength(vardata);
//                    if (Vector<Edge>(path2, path2 + path2_length) == input_path)
//                    {
//                        found = true;
//                    }
//                }
//                if (found)
//                {
//                    continue;
//                }
//
//                // Add column.
//                SCIP_VAR* var = nullptr;
//                SCIP_CALL(SCIPprobdataAddPricedVar(scip, probdata, a, input_path.size(), input_path.data(), &var));
//                debug_assert(var);
//                println("Adding debug column {} {} {} {}",
//                        a, path_cost, agent_part_dual[a], format_path(probdata, input_path.size(), input_path.data()));
//            }
//            astar.set_verbose(false);
//            *result = SCIP_SUCCESS;
//            return SCIP_OKAY;
//        }
//        else
//        {
//            *result = SCIP_DIDNOTRUN;
//            return SCIP_OKAY;
//        }
//    }

    // Find the makespan.
    Time makespan = 0;
    for (auto var : vars)
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

    // Get dual variable values of agent partition constraints.
    auto agent_part_dual = pricerdata->agent_part_dual;
    for (Agent a = 0; a < N; ++a)
    {
        // Get the constraint.
        auto cons = agent_part[a];
        debug_assert(cons);

        // Check that the constraint is not (locally) disabled/redundant.
        debug_assert(SCIPconsIsEnabled(cons));

        // Check that no variable is fixed to one.
        debug_assert(SCIPgetNFixedonesSetppc(scip, cons) == 0);

        // Store dual value.
        agent_part_dual[a] = is_farkas ? SCIPgetDualfarkasSetppc(scip, cons) : SCIPgetDualsolSetppc(scip, cons);
        debug_assert(SCIPisGE(scip, agent_part_dual[a], 0.0));
    }

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
    Vector<bool> agent_priced(N);
#ifdef PRINT_DEBUG
    Int nb_new_cols = 0;
#endif
    bool found = false;
    for (Int order_idx = 0;
         order_idx < N && (!found || order[order_idx].must_price) && !SCIPisStopped(scip);
         ++order_idx)
    {
        // Get agent.
        const auto a = order[order_idx].a;
        const auto start = agents[a].start;
        const auto goal = agents[a].goal;
        SCIP_Real path_cost = 0.0;
        Vector<Edge> path;

        // Clear previous run.
        auto& time_finish_penalties = astar.time_finish_penalties();
        time_finish_penalties.clear();

        // Set up reservation table. Reserve vertices in use by all other agents and
        // reserve vertices in all new paths.
        restab.clear_reservations();
        for (auto var : vars)
        {
            // Make reservations.
            debug_assert(var);
            const auto var_val = SCIPgetSolVal(scip, nullptr, var);
            if (SCIPisPositive(scip, var_val))
            {
                // Reserve if different agent.
                auto vardata = SCIPvarGetData(var);
                const auto path_a = SCIPvardataGetAgent(vardata);
                if (path_a != a)
                {
                    // Get the path.
                    const auto path_length = SCIPvardataGetPathLength(vardata);
                    const auto path = SCIPvardataGetPath(vardata);

                    // Update reservation table.
                    Time t = 0;
                    {
                        const auto n = path[t].n;
                        restab.reserve(NodeTime{n, t});
                        restab.reserve(NodeTime{n, t + 1});
                    }
                    for (t = 1; t < path_length; ++t)
                    {
                        const auto n = path[t].n;
                        restab.reserve(NodeTime{n, t - 1});
                        restab.reserve(NodeTime{n, t});
                        restab.reserve(NodeTime{n, t + 1});
                    }
                    const auto n = path[path_length - 1].n;
                    for (++t; t < makespan; ++t)
                    {
                        restab.reserve(NodeTime{n, t});
                    }
                }
            }
        }
        for (Int idx = 0; idx < order_idx; ++idx)
        {
            auto var = order[idx].new_var;
            if (var)
            {
                // Get the path.
                auto vardata = SCIPvarGetData(var);
                debug_assert(SCIPvardataGetAgent(vardata) != a);
                const auto path_length = SCIPvardataGetPathLength(vardata);
                const auto path = SCIPvardataGetPath(vardata);

                // Update reservation table.
                Time t = 0;
                {
                    const auto n = path[t].n;
                    restab.reserve(NodeTime{n, t});
                    restab.reserve(NodeTime{n, t + 1});
                }
                for (t = 1; t < path_length; ++t)
                {
                    const auto n = path[t].n;
                    restab.reserve(NodeTime{n, t - 1});
                    restab.reserve(NodeTime{n, t});
                    restab.reserve(NodeTime{n, t + 1});
                }
                const auto n = path[path_length - 1].n;
                for (++t; t < makespan; ++t)
                {
                    restab.reserve(NodeTime{n, t});
                }
            }
        }

        // Modify edge costs for two-agent robust cuts.
        edge_penalties = global_edge_penalties;
        for (const auto& cut : two_agent_robust_cuts)
            if (a == cut.a1() || a == cut.a2())
            {
                const auto dual = is_farkas ?
                                  SCIProwGetDualfarkas(cut.row()) :
                                  SCIProwGetDualsol(cut.row());
                debug_assert(SCIPisFeasLE(scip, dual, 0.0));
                if (SCIPisFeasLT(scip, dual, 0.0))
                {
                    // Add the dual variable value to the edges.
                    if (cut.is_same_time())
                    {
                        const auto t = cut.t();
                        for (auto [it, end] = cut.edges(a); it != end; ++it)
                        {
                            const auto e = *it;
                            auto& penalties = edge_penalties.get_edge_penalties(e.n, t);
                            penalties.d[e.d] -= dual;
                        }
                    }
                    else
                    {
                        for (auto [it, end] = cut.edge_times(a); it != end; ++it)
                        {
                            const auto n = it->n;
                            const auto d = it->d;
                            const auto t = it->t;
                            auto& penalties = edge_penalties.get_edge_penalties(n, t);
                            penalties.d[d] -= dual;
                        }
                    }
                }
            }

        // Add goal crossings. If a2 uses the goal of a1 at or after time t, incur the penalty.
#ifdef USE_GOAL_CONFLICTS
        goal_crossings.clear();
        for (const auto& [row, a1, a2, nt] : goal_conflicts)
            if (a == a2)
            {
                const auto dual = is_farkas ?
                                  SCIProwGetDualfarkas(row) :
                                  SCIProwGetDualsol(row);
                debug_assert(SCIPisFeasLE(scip, dual, 0.0));
                if (SCIPisFeasLT(scip, dual, 0.0))
                {
                    auto& goal = goal_crossings.emplace_back();
                    goal.dual = dual;
                    goal.nt = nt;
                }
            }
#endif

        // Modify edge costs for rectangle clique conflicts.
#ifdef USE_RECTANGLE_CLIQUE_CONFLICTS
        rectangle_crossings.clear();
        for (const auto& conflict : rectangle_clique_conflicts_conss)
            if (a == conflict.a1 || a == conflict.a2)
            {
                const auto dual = is_farkas ?
                                  SCIProwGetDualfarkas(conflict.row) :
                                  SCIProwGetDualsol(conflict.row);
                debug_assert(SCIPisFeasLE(scip, dual, 0.0));
                if (SCIPisFeasLT(scip, dual, 0.0))
                {
                    // Add a rectangle crossing penalty.
                    auto& rectangle = rectangle_crossings.emplace_back();
                    const auto [it1, it2, it3] = conflict.agent_in_out_edges(a);
                    rectangle.dual = dual;
                    rectangle.mid = it2 - it1;
                    rectangle.end = it3 - it1;
                    rectangle.other_dir = a == conflict.a1 ?
                                          conflict.in2_begin()->e.d :
                                          conflict.in1_begin()->e.d;
                    rectangle.edges = UniquePtr<EdgeTime[]>(new EdgeTime[rectangle.end]);
                    std::copy(it1, it3, rectangle.edges.get());
                }
            }
#endif

        // Modify edge costs for vertex branching decisions.
        Vector<NodeTime> segments;
        segments.push_back(NodeTime{start, 0});
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
            if (branch_a == a && dir == VertexBranchDirection::Use)
            {
                // End the path segment.
                segments.push_back(nt);
            }
            else if (branch_a == a || dir == VertexBranchDirection::Use)
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
        }

        // Modify edge costs for wait branching decisions.
        Time earliest_finish = 0;
//        for (Int c = 0; c < n_wait_branching_conss; ++c)
//        {
//            // Get the constraint.
//            auto cons = wait_branching_conss[c];
//            debug_assert(cons);
//
//            // Ignore constraints that are not active since these are not on the current
//            // active path of the search tree or constraints for a different agent.
//            if (!SCIPconsIsActive(cons) || SCIPgetWaitBranchingAgent(cons) != a)
//                continue;
//
//            // Enforce the decision.
//            const auto dir = SCIPgetWaitBranchingDirection(cons);
//            const auto t = SCIPgetWaitBranchingTime(cons);
//            if (dir == WaitBranchDirection::MustWait)
//            {
//                for (Coord x = 0; x < width; ++x)
//                    for (Coord y = 0; y < height; ++y)
//                    {
//                        const auto n = map.get_id(x, y);
//                        auto& duals = edge_duals.get_edge_duals(n, t);
//                        duals.north = std::numeric_limits<Cost>::infinity();
//                        duals.south = std::numeric_limits<Cost>::infinity();
//                        duals.east = std::numeric_limits<Cost>::infinity();
//                        duals.west = std::numeric_limits<Cost>::infinity();
//                    }
//
//                if (t + 1 > earliest_finish)
//                    earliest_finish = t + 1;
//            }
//            else
//            {
//                for (Coord x = 0; x < width; ++x)
//                    for (Coord y = 0; y < height; ++y)
//                    {
//                        const auto n = map.get_id(x, y);
//                        auto& duals = edge_duals.get_edge_duals(n, t);
//                        duals.wait = std::numeric_limits<Cost>::infinity();
//                    }
//            }
//            edge_duals_changed = true;
//        }

        // Sort segments by time.
        std::sort(segments.begin(), segments.end(), [](const auto& a, const auto& b)
        {
            return a.t < b.t;
        });
#ifdef DEBUG
        for (size_t idx = 0; idx < segments.size() - 1; ++idx)
        {
            debug_assert(segments[idx].t < segments[idx + 1].t);
        }
#endif

        // Modify edge costs for length branching decisions.
        debug_assert(astar.max_path_length() >= 1);
        Time latest_finish = astar.max_path_length() - 1;
        for (Int c = 0; c < n_length_branching_conss; ++c)
        {
            // Get the constraint.
            auto cons = length_branching_conss[c];
            debug_assert(cons);

            // Ignore constraints that are not active since these are not on the current
            // active path of the search tree.
            if (!SCIPconsIsActive(cons))
                continue;

            // Enforce the decision if the same agent. Disable crossing if different
            // agent.
            const auto branch_a = SCIPgetLengthBranchingAgent(cons);
            const auto dir = SCIPgetLengthBranchingDirection(cons);
            const auto nt = SCIPgetLengthBranchingNodeTime(cons);
            if (branch_a == a)
            {
                if (dir == LengthBranchDirection::LEq && nt.t < latest_finish)
                {
                    latest_finish = nt.t;
                }
                else if (dir == LengthBranchDirection::GEq && nt.t > earliest_finish)
                {
                    earliest_finish = nt.t;
                }
            }
            else if (dir == LengthBranchDirection::LEq)
            {
                // Don't use the vertex.
                for (Time t = nt.t; t < astar.max_path_length(); ++t)
                {
                    const auto prev_time = t - 1;
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
            }
        }
        debug_assert(latest_finish >= segments.back().t);

        // Print time constraints.
//#ifdef PRINT_DEBUG
//        {
//            debugln("   Modified edge costs for agent {}", a);
//            const uint32_t size = gm.height() * gm.width();
//            for (uint32_t node1 = 0; node1 < size; ++node1)
//            {
//                uint32_t x, y;
//                gm.to_unpadded_xy(node1, x, y);
//                const auto& constraints = time_cons.get_constraint_set(node1);
//                for (const auto& constraint : constraints)
//                {
//                    const auto t = constraint.timestep_;
//                    if (constraint.e_[Direction::NORTH] != 1)
//                    {
//                        debugln("      (({},{}),({},{})) time {} edge cost {:.4f}",
//                                x, y, x, y - 1, t,
//                                constraint.e_[Direction::NORTH]);
//                    }
//                    if (constraint.e_[Direction::SOUTH] != 1)
//                    {
//                        debugln("      (({},{}),({},{})) time {} edge cost {:.4f}",
//                                x, y, x, y + 1, t,
//                                constraint.e_[Direction::SOUTH]);
//                    }
//                    if (constraint.e_[Direction::EAST] != 1)
//                    {
//                        debugln("      (({},{}),({},{})) time {} edge cost {:.4f}",
//                                x, y, x + 1, y, t,
//                                constraint.e_[Direction::EAST]);
//                    }
//                    if (constraint.e_[Direction::WEST] != 1)
//                    {
//                        debugln("      (({},{}),({},{})) time {} edge cost {:.4f}",
//                                x, y, x - 1, y, t,
//                                constraint.e_[Direction::WEST]);
//                    }
//                    if (constraint.e_[Direction::WAIT] != 1)
//                    {
//                        debugln("      (({},{}),({},{})) time {} edge cost {:.4f}",
//                                x, y, x, y, t,
//                                constraint.e_[Direction::WAIT]);
//                    }
//                }
//            }
//        }
//#endif

        // Start timer.
#ifdef PRINT_DEBUG
        const auto start_time = std::chrono::high_resolution_clock::now();
#endif

        // Solve each segment.
//        bool skipping = false;
        size_t idx = 0;
        for (; idx < segments.size() - 1; ++idx)
        {
            // Get the start and goal of this segment.
            const auto segment_start = segments[idx];
            const auto segment_goal = segments[idx + 1];

            // Print.
#ifdef PRINT_DEBUG
            {
                const auto [x1, y1] = map.get_xy(segments[idx].n);
                const auto [x2, y2] = map.get_xy(segments[idx + 1].n);
                debugln("   Solving segment {} for agent {} from ({},{}) at {} to ({},{}) at {}",
                        idx, a, x1, y1, segments[idx].t, x2, y2, segments[idx + 1].t);
            }
#endif

            // Solve.
            const auto max_cost = agent_part_dual[a] - path_cost - EPS;
            const auto [segment, segment_cost] = astar.solve<is_farkas>(segment_start,
                                                                        segment_goal.n,
                                                                        segment_goal.t,
                                                                        segment_goal.t,
                                                                        max_cost);

            // Advance to the next agent if no path is found.
            if (segment.empty())
            {
                goto AGENT_FAILED;
            }

            // Get the solution.
            path_cost += segment_cost;
            for (auto it = segment.begin(); it != segment.end() - 1; ++it)
            {
                const auto d = map.get_direction(it->n, (it + 1)->n);
                path.push_back(Edge{it->n, d});
            }
        }

        // Solve from the last segment to the goal.
        {
            // Get the start and goal of this segment.
            const auto segment_start = segments[idx];

            // Modify edge costs for vertex conflicts at the goal after the agent has completed its
            // path. Incur a penalty for staying at the goal.
            for (const auto& [nt, vertex_conflict] : vertex_conflicts_conss)
            {
                const auto& [row] = vertex_conflict;
                if (nt.n == goal)
                {
                    const auto dual = is_farkas ?
                                      SCIProwGetDualfarkas(row) :
                                      SCIProwGetDualsol(row);
                    debug_assert(SCIPisFeasLE(scip, dual, 0.0));
                    if (SCIPisFeasLT(scip, dual, 0.0))
                    {
                        // Add the penalty if the a1 finishes before time t. The timestep of the
                        // conflict is accounted for in the global edge duals, so this loop has
                        // terminal condition < instead of <=.
                        if (static_cast<Time>(time_finish_penalties.size()) < nt.t)
                        {
                            time_finish_penalties.resize(nt.t);
                        }
                        for (Time t = 0; t < nt.t; ++t)
                        {
                            time_finish_penalties[t] -= dual;
                        }
                    }
                }
            }

            // Modify edge costs for wait-edge conflicts at the goal. Incur a penalty for staying at the goal.
#ifdef USE_WAITEDGE_CONFLICTS
            for (const auto& [et, edge_conflict] : edge_conflicts_conss)
            {
                const auto& [row, edges, conflict_time] = edge_conflict;
                const auto e = edges[2];
                debug_assert(e.d == Direction::WAIT);
                if (e.n == goal)
                {
                    const auto dual = is_farkas ?
                                      SCIProwGetDualfarkas(row) :
                                      SCIProwGetDualsol(row);
                    debug_assert(SCIPisFeasLE(scip, dual, 0.0));
                    if (SCIPisFeasLT(scip, dual, 0.0))
                    {
                        if (static_cast<Time>(time_finish_penalties.size()) < conflict_time + 1)
                        {
                            time_finish_penalties.resize(conflict_time + 1);
                        }
                        for (Time t = 0; t <= conflict_time; ++t)
                        {
                            time_finish_penalties[t] -= dual;
                        }
                    }
                }
            }
#endif

            // Modify edge costs for waits in robust cuts. Incur a penalty for staying at the goal.
            for (const auto& cut : two_agent_robust_cuts)
                if (a == cut.a1() || a == cut.a2())
                {
                    const auto dual = is_farkas ?
                                      SCIProwGetDualfarkas(cut.row()) :
                                      SCIProwGetDualsol(cut.row());
                    debug_assert(SCIPisFeasLE(scip, dual, 0.0));
                    if (SCIPisFeasLT(scip, dual, 0.0))
                    {
                        // Add the dual variable value to the edges.
                        if (cut.is_same_time())
                        {
                            const auto conflict_time = cut.t();
                            for (auto [it, end] = cut.edges(a); it != end; ++it)
                            {
                                const auto e = *it;
                                if (e.n == goal && e.d == Direction::WAIT)
                                {
                                    if (static_cast<Time>(time_finish_penalties.size()) < conflict_time + 1)
                                    {
                                        time_finish_penalties.resize(conflict_time + 1);
                                    }
                                    for (Time t = 0; t <= conflict_time; ++t)
                                    {
                                        time_finish_penalties[t] -= dual;
                                    }
                                }
                            }
                        }
                        else
                        {
                            for (auto [it, end] = cut.edge_times(a); it != end; ++it)
                            {
                                const auto n = it->n;
                                const auto d = it->d;
                                const auto conflict_time = it->t;
                                if (n == goal && d == Direction::WAIT)
                                {
                                    if (static_cast<Time>(time_finish_penalties.size()) < conflict_time + 1)
                                    {
                                        time_finish_penalties.resize(conflict_time + 1);
                                    }
                                    for (Time t = 0; t <= conflict_time; ++t)
                                    {
                                        time_finish_penalties[t] -= dual;
                                    }
                                }
                            }
                        }
                    }
                }

            // Modify edge costs for goal conflicts. If a1 finishes at or before time t, incur the penalty.
#ifdef USE_GOAL_CONFLICTS
            for (const auto& [row, a1, a2, nt] : goal_conflicts)
                if (a == a1)
                {
                    debug_assert(nt.n == goal);

                    const auto dual = is_farkas ? SCIProwGetDualfarkas(row) : SCIProwGetDualsol(row);
                    debug_assert(SCIPisFeasLE(scip, dual, 0.0));
                    if (SCIPisFeasLT(scip, dual, 0.0))
                    {
                        if (static_cast<Time>(time_finish_penalties.size()) < nt.t + 1)
                        {
                            time_finish_penalties.resize(nt.t + 1);
                        }
                        for (Time t = 0; t <= nt.t; ++t)
                        {
                            time_finish_penalties[t] -= dual;
                        }
                    }
                }
#endif

            // Modify edge costs for path length nogoods. If agent a finishes at or before time t, incur the penalty.
#ifdef USE_PATH_LENGTH_NOGOODS
            for (const auto& [row, latest_finish_times] : path_length_nogoods)
                for (const auto& [nogood_a, nogood_t] : latest_finish_times)
                    if (a == nogood_a)
                    {
                        const auto dual = is_farkas ? SCIProwGetDualfarkas(row) : SCIProwGetDualsol(row);
                        debug_assert(SCIPisFeasLE(scip, dual, 0.0));
                        if (SCIPisFeasLT(scip, dual, 0.0))
                        {
                            if (static_cast<Time>(time_finish_penalties.size()) < nogood_t + 1)
                            {
                                time_finish_penalties.resize(nogood_t + 1);
                            }
                            for (Time t = 0; t <= nogood_t; ++t)
                            {
                                time_finish_penalties[t] -= dual;
                            }
                        }
                    }
#endif

            // Print.
#ifdef PRINT_DEBUG
            {
                const auto [x1, y1] = map.get_xy(segments[idx].n);
                const auto [x2, y2] = map.get_xy(goal);
                debugln("   Solving last segment {} for agent {} from ({},{}) at {} to "
                        "({},{})",
                        idx, a, x1, y1, segments[idx].t, x2, y2);
            }
#endif

            // Skip running A* if the penalties in the last iteration of this agent have stayed the same or worsened.
#ifdef USE_ASTAR_SOLUTION_CACHING
            {
//                println("Agent {}", a);
//                println("Previous segments:");
//                println("{}", fmt::join(pricerdata->previous_segments[a], ","));
//                println("Current segments:");
//                println("{}", fmt::join(segments, ","));
//                println("Previous agent dual:");
//                println("{}", pricerdata->previous_agent_part_dual[a]);
//                println("Current agent dual:");
//                println("{}", agent_part_dual[a]);
//                println("Previous edge penalties:");
//                for (const auto& [nt, e] : pricerdata->previous_edge_penalties[a])
//                {
//                    println("({},{}) time {}, north {}, south {}, east {}, west {}, wait {}",
//                            map.get_x(nt.n), map.get_y(nt.n), nt.t, e.d[0], e.d[1], e.d[2], e.d[3], e.d[4]);
//                }
//                println("Current edge penalties:");
//                for (const auto& [nt, e] : astar.edge_penalties())
//                {
//                    println("({},{}) time {}, north {}, south {}, east {}, west {}, wait {}",
//                            map.get_x(nt.n), map.get_y(nt.n), nt.t, e.d[0], e.d[1], e.d[2], e.d[3], e.d[4]);
//                }
//                println("Previous time finish penalties:");
//                for (const auto x : pricerdata->previous_time_finish_penalties[a])
//                {
//                    println("{}", x);
//                }
//                println("Current time finish penalties:");
//                for (const auto x : astar.time_finish_penalties())
//                {
//                    println("{}", x);
//                }
//                println("Previous goal crossing:");
//                for (const auto [dual, nt] : pricerdata->previous_goal_crossings[a])
//                {
//                    println("{} {}", dual, nt);
//                }
//                println("Current goal crossing:");
//                for (const auto [dual, nt] : astar.goal_crossings())
//                {
//                    println("{} {}", dual, nt);
//                }

                if (segments != pricerdata->previous_segments[a])
                {
                    goto SOLVE;
                }

                if (earliest_finish != pricerdata->previous_earliest_finish[a])
                {
                    goto SOLVE;
                }

                if (latest_finish != pricerdata->previous_latest_finish[a])
                {
                    goto SOLVE;
                }

                if (agent_part_dual[a] > pricerdata->previous_agent_part_dual[a])
                {
                    goto SOLVE;
                }

                if (time_finish_penalties.size() != pricerdata->previous_time_finish_penalties[a].size())
                {
                    goto SOLVE;
                }
                for (Time t = 0; t < static_cast<Time>(time_finish_penalties.size()); ++t)
                    if (time_finish_penalties[t] < pricerdata->previous_time_finish_penalties[a][t])
                    {
                        goto SOLVE;
                    }

#ifdef USE_GOAL_CONFLICTS
                if (goal_crossings.size() != pricerdata->previous_goal_crossings[a].size())
                {
                    goto SOLVE;
                }
                for (Int idx = 0; idx < static_cast<Int>(goal_crossings.size()); ++idx)
                    if (goal_crossings[idx].nt != pricerdata->previous_goal_crossings[a][idx].nt ||
                        goal_crossings[idx].dual > pricerdata->previous_goal_crossings[a][idx].dual)
                    {
                        goto SOLVE;
                    }
#endif

                for (const auto& [nt, previous_edge_costs] : pricerdata->previous_edge_penalties[a])
                {
                    const auto it = edge_penalties.find(nt);
                    if (it == edge_penalties.end())
                    {
                        goto SOLVE;
                    }
                    const auto& current_edge_costs = it->second;
                    if (current_edge_costs.north < previous_edge_costs.north ||
                        current_edge_costs.south < previous_edge_costs.south ||
                        current_edge_costs.east < previous_edge_costs.east ||
                        current_edge_costs.west < previous_edge_costs.west ||
                        current_edge_costs.wait < previous_edge_costs.wait)
                    {
                        goto SOLVE;
                    }
                }

//                skipping = true;
                goto AGENT_SUCCESS;
            }

            SOLVE:
#endif
            // Solve.
            const auto max_cost = agent_part_dual[a] - path_cost - EPS;
            const auto [segment, segment_cost] = astar.solve<is_farkas>(segment_start,
                                                                        goal,
                                                                        earliest_finish,
                                                                        latest_finish,
                                                                        max_cost);

            // Advance to next agent if no path is found.
            if (segment.empty())
            {
                goto AGENT_FAILED;
            }

            // Get the solution.
            path_cost += segment_cost;
            for (auto it = segment.begin(); it != segment.end(); ++it)
            {
                const auto d = it != segment.end() - 1 ?
                               map.get_direction(it->n, (it + 1)->n) :
                               Direction::INVALID;
                path.push_back(Edge{it->n, d});
            }
        }

        // Add a column only if the path has negative reduced cost.
        min_reduced_cost = std::min(min_reduced_cost, path_cost - agent_part_dual[a]);
        if (SCIPisSumLT(scip, path_cost - agent_part_dual[a], 0.0))
        {
            // Print.
            debugln("      Found path with length {}, reduced cost {:.6f} ({})",
                    path.size(),
                    path_cost - agent_part_dual[a],
                    format_path(probdata, path.size(), path.data()));

            // Add column.
//            debug_assert(!skipping);
            SCIP_VAR* var = nullptr;
            SCIP_CALL(SCIPprobdataAddPricedVar(scip,
                                               probdata,
                                               a,
                                               path.size(),
                                               path.data(),
                                               &var));
            debug_assert(var);
            found = true;
            order[order_idx].new_var = var;
            pricerdata->price_priority[a]++;
#ifdef PRINT_DEBUG
            nb_new_cols++;
#endif
            goto AGENT_SUCCESS;
        }

        // Store the penalties of the run.
        AGENT_FAILED:
#ifdef USE_ASTAR_SOLUTION_CACHING
        pricerdata->previous_segments[a] = segments;
        pricerdata->previous_earliest_finish[a] = earliest_finish;
        pricerdata->previous_latest_finish[a] = latest_finish;
        pricerdata->previous_agent_part_dual[a] = agent_part_dual[a];
        pricerdata->previous_edge_penalties[a].clear();
        for (const auto& [nt, edge_costs] : astar.edge_penalties())
            if (edge_costs.used)
            {
                pricerdata->previous_edge_penalties[a][nt] = edge_costs;
            }
        pricerdata->previous_time_finish_penalties[a] = time_finish_penalties;
#ifdef USE_GOAL_CONFLICTS
        pricerdata->previous_goal_crossings[a] = goal_crossings;
#endif
#endif

        // End of this agent.
        AGENT_SUCCESS:
        agent_priced[a] = true;

        // End timer.
#ifdef PRINT_DEBUG
        const auto end_time = std::chrono::high_resolution_clock::now();
        const auto duration =
            std::chrono::duration<double>(end_time - start_time).count();
        debugln("      Done in {:.4f} seconds", duration);
#endif
    }

    // Print.
    debugln("   Added {} new columns", nb_new_cols);

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

// Inject a warm-start solution
SCIP_RETCODE add_initial_solution(
    SCIP* scip    // SCIP
)
{
    // Get problem data.
    auto probdata = SCIPgetProbData(scip);
    const auto N = SCIPprobdataGetN(probdata);
    const auto& map = SCIPprobdataGetMap(probdata);
    const auto& agents = SCIPprobdataGetAgentsData(probdata);

    // Get shortest path solver.
    auto& astar = SCIPprobdataGetAStar(probdata);
    auto& restab = astar.reservation_table();
    auto& edge_penalties = astar.edge_penalties();
    restab.clear_reservations();
#ifdef USE_GOAL_CONFLICTS
    auto& goal_crossings = astar.goal_crossings();
    release_assert(goal_crossings.empty(), "Cannot have goal crossings in warm-start solution");
#endif

    // Find a path for each agent.
    for (Agent a = 0; a < N; ++a)
    {
        // Set edge costs.
        edge_penalties.reset();

        // Solve.
        const auto start = NodeTime{agents[a].start, 0};
        const auto goal = agents[a].goal;
        const Time earliest_finish = 0;
        const Time latest_finish = astar.max_path_length() - 1;
        const auto [segment, path_cost] = astar.solve<false>(start,
                                                             goal,
                                                             earliest_finish,
                                                             latest_finish,
                                                             std::numeric_limits<Cost>::max());

        // Get the solution.
        Vector<Edge> path;
        for (auto it = segment.begin(); it != segment.end(); ++it)
        {
            const auto d = it != segment.end() - 1 ?
                           map.get_direction(it->n, (it + 1)->n) :
                           Direction::INVALID;
            path.push_back(Edge{it->n, d});
        }

        // Print.
        debugln("      Found path with length {}, cost {:.6f} ({})",
                path.size(),
                path_cost,
                format_path(probdata, path.size(), path.data()));

        // Add column.
        SCIP_VAR* var = nullptr;
        SCIP_CALL(SCIPprobdataAddInitialVar(scip,
                                            probdata,
                                            a,
                                            path.size(),
                                            path.data(),
                                            &var));
        debug_assert(var);
    }

    return SCIP_OKAY;
}
