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
#ifdef USE_GOAL_CONFLICTS
#include "Separator_GoalConflicts.h"
#endif
#include "Constraint_VertexBranching.h"
//#include "Constraint_WaitBranching.h"
#include "Constraint_LengthBranching.h"
#include <chrono>
//#include <numeric>

#include "trufflehog/Instance.h"
#include "trufflehog/AStar.h"

// Pricer properties
#define PRICER_NAME               "trufflehog"
#define PRICER_DESC       "Truffle Hog pricer"
#define PRICER_PRIORITY                      0
#define PRICER_DELAY                      TRUE  // Only call pricer if all problem variables have non-negative reduced costs

struct PricingOrder
{
    Agent a;
    bool must_price;
    SCIP_VAR* new_var;
};

// Pricer data
struct SCIP_PricerData
{
    Agent N;                                     // Number of agents
    SCIP_Real* price_priority;                   // Pricing priority of each agent
    PricingOrder* order;                         // Order of agents to price
    SCIP_Real* agent_part_dual;                  // Dual variable values of agent partition constraints

    SCIP_CONSHDLR* vertex_branching_conshdlr;    // Constraint handler for vertex branching
//    SCIP_CONSHDLR* wait_branching_conshdlr;      // Constraint handler for wait branching
    SCIP_CONSHDLR* length_branching_conshdlr;    // Constraint handler for length branching
};

// Initialize pricer (called after the problem was transformed)
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
    debug_assert(pricerdata->N > 0);

    // Create array for agent priority in pricing.
    SCIP_CALL(SCIPallocBlockMemoryArray(scip, &pricerdata->price_priority, pricerdata->N));
    memset(pricerdata->price_priority, 0, sizeof(SCIP_Real) * pricerdata->N);

    // Create array for order of agents to price.
    SCIP_CALL(SCIPallocBlockMemoryArray(scip, &pricerdata->order, pricerdata->N));
    // Overwritten in each run. No need for initialisation.

    // Create array for dual variable values of agent partition constraints.
    SCIP_CALL(SCIPallocBlockMemoryArray(scip, &pricerdata->agent_part_dual, pricerdata->N));
    // Overwritten in each run. No need for initialisation.

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

    // Set pointer to pricer data.
    SCIPpricerSetData(pricer, pricerdata);
    SCIPprobdataSetPricerData(probdata, pricerdata);

    // Done.
    return SCIP_OKAY;
}

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
    SCIPfreeBlockMemoryArray(scip, &pricerdata->order, pricerdata->N);
    SCIPfreeBlockMemoryArray(scip, &pricerdata->price_priority, pricerdata->N);
    pricerdata->~SCIP_PricerData();
    SCIPfreeBlockMemory(scip, &pricerdata);

    // Done.
    return SCIP_OKAY;
}

// Compute ordering of agents to price
void calculate_agents_order(
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
    auto order = pricerdata->order;
    for (Agent a = 0; a < N; ++a)
    {
        // Must price an agent if it is using an artificial variable.
        bool must_price = SCIPisPositive(scip, SCIPgetSolVal(scip, nullptr, dummy_vars[a]));

        // Must price an agent if it is fractional.
        if (!must_price)
        {
            for (auto var : agent_vars[a])
            {
                // Get the variable value.
                debug_assert(var);
                const auto var_val = SCIPgetSolVal(scip, nullptr, var);

                // Set.
                if (!SCIPisIntegral(scip, var_val))
                {
                    must_price = true;
                    break;
                }
            }
        }

        // Store.
        order[a] = {a, must_price, nullptr};
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
}

// Get the direction of movement from one node to the next
inline Direction get_direction(
    const NodeTime nt1,    // Outgoing node
    const NodeTime nt2,    // Incoming node
    const Map& map         // Map
)
{
    // Check.
    debug_assert(
        nt2.n == map.get_north(nt1.n) ||
        nt2.n == map.get_south(nt1.n) ||
        nt2.n == map.get_east(nt1.n) ||
        nt2.n == map.get_west(nt1.n) ||
        nt2.n == map.get_wait(nt1.n)
    );

    // Return direction.
    if (nt2.n == map.get_north(nt1.n))
    {
        return Direction::NORTH;
    }
    else if (nt2.n == map.get_south(nt1.n))
    {
        return Direction::SOUTH;
    }
    else if (nt2.n == map.get_east(nt1.n))
    {
        return Direction::EAST;
    }
    else if (nt2.n == map.get_west(nt1.n))
    {
        return Direction::WEST;
    }
    else
    {
        return Direction::WAIT;
    }
}

static
SCIP_RETCODE run_trufflehog_pricer(
    SCIP* scip,             // SCIP
    SCIP_PRICER* pricer,    // Pricer
    SCIP_RESULT* result,    // Output result
    SCIP_Bool*,             // Output flag to indicate early branching is required
    SCIP_Real*              // Output lower bound
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
        debugln("Starting pricer for feasible master problem at node {}, depth {}, obj "
                "{}:",
                SCIPnodeGetNumber(SCIPgetCurrentNode(scip)),
                SCIPgetDepth(scip),
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

    // Get variables.
    auto& vars = SCIPprobdataGetVars(probdata);

    // Get constraints.
    const auto& agent_part = SCIPprobdataGetAgentPartConss(probdata);
    const auto& vertex_conflicts_conss = vertex_conflicts_get_constraints(probdata);
    const auto& edge_conflicts_conss = edge_conflicts_get_constraints(probdata);

    // Get cuts.
    const auto& two_agent_robust_cuts = SCIPprobdataGetTwoAgentRobustCuts(probdata);
#ifdef USE_GOAL_CONFLICTS
    const auto& goal_conflicts_conss = goal_conflicts_get_constraints(probdata);
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
    auto& pathfinder = SCIPprobdataGetPathfinder(probdata);
    auto& restab = pathfinder.reservation_table();
    auto& edge_penalties = pathfinder.edge_penalties();
#ifdef USE_GOAL_CONFLICTS
    auto& goal_crossings = pathfinder.goal_crossings();
#endif
    static_assert(std::numeric_limits<Cost>::has_quiet_NaN);

    // Print used paths.
#ifdef PRINT_DEBUG
    print_used_paths(scip);
#endif

    // Use debug solution.
//    {
//        Vector<Vector<Pair<Position,Position>>> init_paths{
//            {     {13,8},    {12,8},    {11,8},    {10,8},     {9,8},     {8,8},     {7,8},     {6,8},     {6,9},    {6,10},    {6,11}, },
//            {     {3,17},    {4,17},    {5,17},    {6,17},    {7,17},    {8,17},    {9,17},    {9,18},   {10,18},   {11,18},   {12,18},   {13,18},   {14,18},   {15,18},   {15,19},   {16,19},   {17,19},   {18,19}, },
//            {     {6,12},    {5,12},    {4,12},    {3,12},    {3,13},    {2,13},    {2,14},    {2,15}, },
//            {     {5,16},    {5,15},    {5,14},    {5,13},    {4,13},    {4,12},    {3,12},    {3,11},    {3,10},     {3,9},     {3,8}, },
//            {     {16,1},    {15,1},    {15,2},    {15,3},    {15,4},    {14,4},    {14,5},    {14,6},    {13,6},    {13,7},    {13,8},    {13,9},    {12,9},    {11,9},    {10,9},     {9,9},     {8,9}, },
//            {    {17,13},   {16,13},   {16,14},   {16,15},   {16,16},   {15,16}, },
//            {     {9,19},    {9,18},    {9,17},    {9,16},    {9,15},    {9,14},    {9,13},    {9,12},    {9,11},    {8,11},    {8,10},    {7,10},     {7,9},     {7,8},     {6,8},     {5,8},     {5,7},     {5,6},     {5,5},     {5,4},     {5,3},     {5,2},     {4,2},     {3,2},     {3,1},     {2,1}, },
//            {     {8,16},    {8,17},    {8,18},    {7,18},    {6,18},    {5,18},    {4,18},    {3,18},    {2,18},    {1,18}, },
//            {      {7,4},     {7,5},     {7,6},     {7,7},     {7,8},     {7,9},    {7,10},    {7,11},    {7,12},    {7,13},    {8,13},    {9,13}, },
//            {      {7,4},     {7,4},     {7,5},     {7,6},     {7,7},     {7,8},     {7,9},    {7,10},    {7,11},    {7,12},    {7,13},    {8,13},    {9,13}, },
//            {    {12,17},   {12,16},   {11,16},   {11,15},   {11,14},   {11,13},   {10,13},    {9,13},    {9,12},    {9,11},    {8,11},    {8,10},     {8,9},     {8,8}, },
//            {      {2,4},     {2,5},     {3,5},     {4,5},     {5,5},     {6,5},     {7,5},     {7,6},     {7,7},     {8,7}, },
//            {    {16,15},   {16,16},   {15,16},   {14,16},   {13,16},   {12,16},   {11,16}, },
//            {    {15,13},   {14,13},   {13,13},   {13,12},   {13,11},   {13,10},    {13,9},    {12,9},    {12,8},    {12,7},    {12,6},    {12,5},    {11,5},    {10,5},    {10,4}, },
//            {      {6,5},     {6,6},     {7,6},     {8,6},     {9,6},    {10,6},    {10,7},    {11,7},    {11,8},    {11,9},   {11,10},   {12,10},   {13,10},   {13,11},   {14,11},   {14,12}, },
//            {      {6,5},     {6,6},     {6,7},     {6,8},     {6,9},     {7,9},     {8,9},     {9,9},    {10,9},   {10,10},   {11,10},   {11,11},   {12,11},   {13,11},   {14,11},   {14,12}, },
//            {     {19,7},    {19,8},    {18,8},    {17,8},    {16,8},    {16,9},    {15,9},    {14,9},    {13,9},    {12,9},    {11,9},    {10,9},     {9,9},     {8,9},     {7,9},    {7,10},    {7,11}, },
//            {    {20,14},   {20,15},   {20,16},   {20,17},   {19,17},   {19,18},   {18,18},   {18,19},   {17,19},   {16,19},   {15,19},   {14,19}, },
//            {     {6,18},    {7,18},    {7,17},    {8,17},    {8,16},    {9,16},    {9,15},   {10,15},   {11,15},   {11,14},   {11,13},   {12,13},   {13,13},   {13,12},   {13,11},   {13,10},    {13,9},    {14,9}, },
//            {     {13,9},    {12,9},    {11,9},    {10,9},     {9,9},     {8,9},    {8,10},    {8,11},    {7,11},    {7,12},    {6,12},    {5,12},    {5,13},    {5,14},    {5,15}, },
//            {     {13,9},    {12,9},    {11,9},    {10,9},     {9,9},     {8,9},    {8,10},    {7,10},    {6,10},    {6,11},    {6,12},    {5,12},    {5,13},    {5,14},    {5,15}, },
//            {     {6,13},    {7,13},    {7,12},    {7,11},    {8,11},    {9,11},   {10,11}, },
//            {     {12,6},    {11,6},    {10,6},    {10,7},     {9,7},     {9,8},     {8,8},     {7,8},     {7,9},    {7,10},    {7,11},    {7,12},    {7,13},    {6,13},    {6,14},    {5,14}, },
//        };
////        Vector<Agent> path_agents{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 11, 12, 13, 13, 14, 15, 16, 17, 17, 18, 19 };
//        Vector<Agent> path_agents(N);
//        std::iota(path_agents.begin(), path_agents.end(), 0);
//
//        static bool done = false;
//        if (!done)
//        {
//            constexpr Position padding = 1;
//
//            for (Int idx = 0; idx < init_paths.size(); ++idx)
//            {
//                const auto& init_path = init_paths.at(idx);
//                const auto a = path_agents.at(idx);
////                const auto tmp = astar.calculate_cost<true>(init_path);
//
//                Vector<Edge> path;
//                for (Int idx = 0; idx < init_path.size() - 1; ++idx)
//                {
//                    const auto [x1, y1] = init_path[idx];
//                    const auto [x2, y2] = init_path[idx+1];
//                    const auto i = map.get_id(x1 + padding, y1 + padding);
//                    const auto j = map.get_id(x2 + padding, y2 + padding);
//                    const auto d = get_direction(NodeTime{i,idx}, NodeTime{j,idx+1}, map);
//                    path.push_back(Edge{i,d});
//                }
//                {
//                    Int idx = init_path.size() - 1;
//                    const auto [x1, y1] = init_path[idx];
//                    const auto i = map.get_id(x1 + padding, y1 + padding);
//                    const auto d = Direction::INVALID;
//                    path.push_back(Edge{i,d});
//                }
//                debug_assert(path.front().n == agents[a].start);
//                debug_assert(path.back().n == agents[a].goal);
//
//                // Add column.
//                SCIP_VAR* var = nullptr;
//                SCIP_CALL(SCIPprobdataAddPricedVar(scip,
//                                                   probdata,
//                                                   a,
//                                                   path.size(),
//                                                   path.data(),
//                                                   &var));
//                debug_assert(var);
//            }
//            done = true;
//        }
//        *result = SCIP_SUCCESS;
//        return SCIP_OKAY;
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
            makespan = path_length;
    }

    // Print dual values.
#ifdef PRINT_DEBUG
    print_agent_part_dual(scip, is_farkas);
    print_vertex_conflicts_dual(scip, is_farkas);
    print_edge_conflicts_dual(scip, is_farkas);
    print_two_agent_robust_cuts_dual(scip, is_farkas);
#ifdef USE_GOAL_CONFLICTS
    print_goal_conflicts_dual(scip, is_farkas);
#endif
#endif

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
        agent_part_dual[a] = is_farkas ?
                             SCIPgetDualfarkasSetppc(scip, cons) :
                             SCIPgetDualsolSetppc(scip, cons);
        debug_assert(SCIPisGE(scip, agent_part_dual[a], 0.0));
    }

    // Make edge penalties for all agents.
    EdgePenalties global_edge_penalties;

    // Input dual values for vertex conflicts.
    for (const auto [row, nt] : vertex_conflicts_conss)
    {
        const auto dual = is_farkas ? SCIProwGetDualfarkas(row) : SCIProwGetDualsol(row);
        debug_assert(SCIPisLE(scip, dual, 0.0));
        if (SCIPisLT(scip, dual, 0.0))
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
    for (const auto& [row, edges, t] : edge_conflicts_conss)
    {
        const auto dual = is_farkas ? SCIProwGetDualfarkas(row) : SCIProwGetDualsol(row);
        debug_assert(SCIPisLE(scip, dual, 0.0));
        if (SCIPisLT(scip, dual, 0.0))
        {
            // Add the dual variable value to the edges.
            for (const auto e : edges)
            {
                auto& penalties = global_edge_penalties.get_edge_penalties(e.n, t);
                penalties.d[e.d] -= dual;
            }
        }
    }

    // Create order of agents to solve.
    auto order = pricerdata->order;
    calculate_agents_order(scip, probdata, pricerdata);
    for (Agent a = 0; a < N; ++a)
    {
        pricerdata->price_priority[a] /= PRICE_PRIORITY_DECAY_FACTOR;
    }

    // Price each agent.
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
        auto& time_finish_penalties = pathfinder.time_finish_penalties();
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
                debug_assert(SCIPisLE(scip, dual, 0.0));
                if (SCIPisLT(scip, dual, 0.0))
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
        for (const auto& [row, a1, a2, nt] : goal_conflicts_conss)
            if (a == a2)
            {
                const auto dual = is_farkas ?
                                  SCIProwGetDualfarkas(row) :
                                  SCIProwGetDualsol(row);
                debug_assert(SCIPisLE(scip, dual, 0.0));
                if (SCIPisLT(scip, dual, 0.0))
                {
                    auto& goal = goal_crossings.emplace_back();
                    goal.dual = dual;
                    goal.nt = nt;
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
                    penalties.north = NAN;
                }
                {
                    const auto n = map.get_north(nt.n);
                    auto& penalties = edge_penalties.get_edge_penalties(n, prev_time);
                    penalties.south = NAN;
                }
                {
                    const auto n = map.get_west(nt.n);
                    auto& penalties = edge_penalties.get_edge_penalties(n, prev_time);
                    penalties.east = NAN;
                }
                {
                    const auto n = map.get_east(nt.n);
                    auto& penalties = edge_penalties.get_edge_penalties(n, prev_time);
                    penalties.west = NAN;
                }
                {
                    const auto n = map.get_wait(nt.n);
                    auto& penalties = edge_penalties.get_edge_penalties(n, prev_time);
                    penalties.wait = NAN;
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
//                        duals.north = NAN;
//                        duals.south = NAN;
//                        duals.east = NAN;
//                        duals.west = NAN;
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
//                        duals.wait = NAN;
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
        debug_assert(pathfinder.max_path_length() >= 1);
        Time latest_finish = pathfinder.max_path_length() - 1;
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
                for (Time t = nt.t; t < pathfinder.max_path_length(); ++t)
                {
                    const auto prev_time = t - 1;
                    {
                        const auto n = map.get_south(nt.n);
                        auto& penalties = edge_penalties.get_edge_penalties(n, prev_time);
                        penalties.north = NAN;
                    }
                    {
                        const auto n = map.get_north(nt.n);
                        auto& penalties = edge_penalties.get_edge_penalties(n, prev_time);
                        penalties.south = NAN;
                    }
                    {
                        const auto n = map.get_west(nt.n);
                        auto& penalties = edge_penalties.get_edge_penalties(n, prev_time);
                        penalties.east = NAN;
                    }
                    {
                        const auto n = map.get_east(nt.n);
                        auto& penalties = edge_penalties.get_edge_penalties(n, prev_time);
                        penalties.west = NAN;
                    }
                    {
                        const auto n = map.get_wait(nt.n);
                        auto& penalties = edge_penalties.get_edge_penalties(n, prev_time);
                        penalties.wait = NAN;
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
            const auto max_cost = agent_part_dual[a] - path_cost;
            const auto [segment, segment_cost] = pathfinder.solve(segment_start,
                                                                        segment_goal.n,
                                                                        segment_goal.t,
                                                                        segment_goal.t,
                                                                        max_cost);

            // Advance to the next agent if no path is found.
            if (segment.empty())
            {
                goto NEXT_AGENT;
            }

            // Get the solution.
            path_cost += segment_cost;
            for (auto it = segment.begin(); it != segment.end() - 1; ++it)
            {
                const auto d = get_direction(*it, *(it + 1), map);
                path.push_back(Edge{it->n, d});
            }
        }

        // Solve from the last segment to the goal.
        {
            // Get the start and goal of this segment.
            const auto segment_start = segments[idx];

            // Modify edge costs for vertex conflicts at the goal after the agent has completed its
            // path. Incur a penalty for staying at the goal.
            for (const auto [row, nt] : vertex_conflicts_conss)
                if (nt.n == goal)
                {
                    const auto dual = is_farkas ?
                                      SCIProwGetDualfarkas(row) :
                                      SCIProwGetDualsol(row);
                    debug_assert(SCIPisLE(scip, dual, 0.0));
                    if (SCIPisLT(scip, dual, 0.0))
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

            // Modify edge costs for goal conflicts. If a1 finishes at or before time t, incur the
            // penalty.
#ifdef USE_GOAL_CONFLICTS
            for (const auto& [row, a1, a2, nt] : goal_conflicts_conss)
                if (a == a1)
                {
                    debug_assert(nt.n == goal);

                    const auto dual = is_farkas ?
                                      SCIProwGetDualfarkas(row) :
                                      SCIProwGetDualsol(row);
                    debug_assert(SCIPisLE(scip, dual, 0.0));
                    if (SCIPisLT(scip, dual, 0.0))
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

            // Solve.
            const auto max_cost = agent_part_dual[a] - path_cost;
            const auto [segment, segment_cost] = pathfinder.solve(segment_start,
                                                                        goal,
                                                                        earliest_finish,
                                                                        latest_finish,
                                                                        max_cost);

            // Advance to next agent if no path is found.
            if (segment.empty())
            {
                goto NEXT_AGENT;
            }

            // Get the solution.
            path_cost += segment_cost;
            for (auto it = segment.begin(); it != segment.end(); ++it)
            {
                const auto d = it != segment.end() - 1 ?
                               get_direction(*it, *(it + 1), map) :
                               Direction::INVALID;
                path.push_back(Edge{it->n, d});
            }
        }

        // Add a column only if the path has negative reduced cost.
        if (SCIPisSumLT(scip, path_cost - agent_part_dual[a], 0.0))
        {
            // Print.
            debugln("      Found path with length {}, reduced cost {:.6f} ({})",
                    path.size(),
                    path_cost - agent_part_dual[a],
                    format_path(probdata, path.size(), path.data()));

            // Add column.
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
        }

        // End of this agent.
        NEXT_AGENT:;

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

    // Done.
    *result = SCIP_SUCCESS;
    return SCIP_OKAY;
}

// Reduced cost pricing for feasible master problem
static
SCIP_DECL_PRICERREDCOST(pricerTruffleHogRedCost)
{
    return run_trufflehog_pricer(scip, pricer, result, stopearly, lowerbound);
}

// Farkas pricing for infeasible master problem
static
SCIP_DECL_PRICERFARKAS(pricerTruffleHogFarkas)
{
    unreachable();
}

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
