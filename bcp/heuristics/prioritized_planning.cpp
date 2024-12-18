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

#ifdef USE_PRIORITIZED_PLANNING_PRIMAL_HEURISTIC

// #define PRINT_DEBUG

#include "heuristics/prioritized_planning.h"
#include "problem/problem.h"
#include "problem/variable_data.h"
#include "constraints/nodetime.h"
#include "constraints/edgetime.h"
#include "branching/vertex_branching_constraint.h"
//#include "branching/wait_branching_constraint.h"
#include "branching/length_branching_constraint.h"
#include <chrono>
#include <numeric>
#include <algorithm>
#include <random>
#include <limits>

#define HEUR_NAME             "mapf-pp"
#define HEUR_DESC             "MAPF prioritized planning"
#define HEUR_DISPCHAR         'P'
#define HEUR_PRIORITY         10000
#define HEUR_FREQ             1
#define HEUR_FREQOFS          0
#define HEUR_MAXDEPTH         -1
#define HEUR_TIMING           SCIP_HEURTIMING_AFTERLPNODE
#define HEUR_USESSUBSCIP      FALSE    // Does the heuristic use a secondary SCIP instance?

struct PrioritizedPlanningData
{
    SCIP_CONSHDLR* vertex_branching_conshdlr;           // Constraint handler for vertex branching
//    SCIP_CONSHDLR* wait_branching_conshdlr;           // Constraint handler for wait branching
    SCIP_CONSHDLR* length_branching_conshdlr;           // Constraint handler for length branching

    std::mt19937 rng{0};
};

// Initialize primal heuristic (called after the problem was transformed)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_HEURINIT(heurInitPrioritizedPlanning)
{
    // Check.
    debug_assert(scip);
    debug_assert(heur);

    // Create heuristic data.
    PrioritizedPlanningData* heurdata;
    SCIP_CALL(SCIPallocBlockMemory(scip, &heurdata));
    new (heurdata) PrioritizedPlanningData;

    // Find constraint handler for branching decisions.
    heurdata->vertex_branching_conshdlr = SCIPfindConshdlr(scip, "vertex_branching");
    release_assert(heurdata->vertex_branching_conshdlr,
                   "Constraint handler for vertex branching is missing");
//    heurdata->wait_branching_conshdlr = SCIPfindConshdlr(scip, "wait_branching");
//    release_assert(heurdata->wait_branching_conshdlr,
//                   "Constraint handler for wait branching is missing");
    heurdata->length_branching_conshdlr = SCIPfindConshdlr(scip, "length_branching");
    release_assert(heurdata->length_branching_conshdlr,
                   "Constraint handler for length branching rule is missing");

    // Set pointer to pricer data.
    SCIPheurSetData(heur, reinterpret_cast<SCIP_HeurData*>(heurdata));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Free primal heuristic
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_HEURFREE(heurFreePrioritizedPlanning)
{
    // Check.
    debug_assert(scip);
    debug_assert(heur);

    // Get heuristic data.
    auto heurdata = reinterpret_cast<PrioritizedPlanningData*>(SCIPheurGetData(heur));
    debug_assert(heurdata);

    // Deallocate.
    heurdata->~PrioritizedPlanningData();
    SCIPfreeBlockMemory(scip, &heurdata);

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

void block_path(const Edge* const path,
                const Time path_length,
                EdgePenalties& global_edge_penalties,
                HashMap<Node, Time>& node_latest_visit_time,
                const Time max_path_length,
                const Map& map)
{
    // Update latest time to visit a node.
    for (Time t = 0; t < path_length; ++t)
    {
        auto [it, success] = node_latest_visit_time.emplace(Node{path[t].n}, t);
        if (!success)
        {
            it->second = std::max(it->second, t);
        }
    }

    // Block vertices in future paths.
    {
        Node path_n;
        Time t = 1;
        for (; t < path_length; ++t)
        {
            // Don't use the vertex.
            path_n = path[t].n;
            const auto prev_time = t - 1;
            {
                const auto n = map.get_south(path_n);
                auto& penalties = global_edge_penalties.get_edge_penalties(n, prev_time);
                penalties.north = std::numeric_limits<Cost>::infinity();
            }
            {
                const auto n = map.get_north(path_n);
                auto& penalties = global_edge_penalties.get_edge_penalties(n, prev_time);
                penalties.south = std::numeric_limits<Cost>::infinity();
            }
            {
                const auto n = map.get_west(path_n);
                auto& penalties = global_edge_penalties.get_edge_penalties(n, prev_time);
                penalties.east = std::numeric_limits<Cost>::infinity();
            }
            {
                const auto n = map.get_east(path_n);
                auto& penalties = global_edge_penalties.get_edge_penalties(n, prev_time);
                penalties.west = std::numeric_limits<Cost>::infinity();
            }
            {
                const auto n = map.get_wait(path_n);
                auto& penalties = global_edge_penalties.get_edge_penalties(n, prev_time);
                penalties.wait = std::numeric_limits<Cost>::infinity();
            }
        }
        for (; t < max_path_length; ++t)
        {
            // Don't use the vertex.
            const auto prev_time = t - 1;
            {
                const auto n = map.get_south(path_n);
                auto& penalties = global_edge_penalties.get_edge_penalties(n, prev_time);
                penalties.north = std::numeric_limits<Cost>::infinity();
            }
            {
                const auto n = map.get_north(path_n);
                auto& penalties = global_edge_penalties.get_edge_penalties(n, prev_time);
                penalties.south = std::numeric_limits<Cost>::infinity();
            }
            {
                const auto n = map.get_west(path_n);
                auto& penalties = global_edge_penalties.get_edge_penalties(n, prev_time);
                penalties.east = std::numeric_limits<Cost>::infinity();
            }
            {
                const auto n = map.get_east(path_n);
                auto& penalties = global_edge_penalties.get_edge_penalties(n, prev_time);
                penalties.west = std::numeric_limits<Cost>::infinity();
            }
            {
                const auto n = map.get_wait(path_n);
                auto& penalties = global_edge_penalties.get_edge_penalties(n, prev_time);
                penalties.wait = std::numeric_limits<Cost>::infinity();
            }
        }
    }

    // Block edges in future paths.
    for (Time t = 0; t < path_length - 1; ++t)
    {
        const auto e = map.get_opposite_edge(path[t]);
        auto& penalties = global_edge_penalties.get_edge_penalties(e.n, t);
        penalties.d[e.d] = std::numeric_limits<Cost>::infinity();
    }
}

// Execution method of primal heuristic
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_HEUREXEC(heurExecPrioritizedPlanning)
{
    // Do not run if the node is infeasible.
    if (nodeinfeasible)
    {
        *result = SCIP_DIDNOTRUN;
        return SCIP_OKAY;
    }

    // Print.
    // debugln("Starting prioritized planning primal heuristic at node {}, depth {}:",
    //         SCIPnodeGetNumber(SCIPgetCurrentNode(scip)),
    //         SCIPgetDepth(scip));

    // Initialize.
    *result = SCIP_DIDNOTFIND;

    // Get problem data.
    auto probdata = SCIPgetProbData(scip);
    const auto N = SCIPprobdataGetN(probdata);
    const auto& map = SCIPprobdataGetMap(probdata);
    const auto& agents = SCIPprobdataGetAgentsData(probdata);

    // Update variable values.
    update_variable_values(scip);

    // Get heuristic data.
    auto heurdata = reinterpret_cast<PrioritizedPlanningData*>(SCIPheurGetData(heur));
    debug_assert(heurdata);

    // Get variables.
    const auto& agent_vars = SCIPprobdataGetAgentVars(probdata);

    // Get constraints for branching decisions.
    const auto n_vertex_branching_conss = SCIPconshdlrGetNConss(heurdata->vertex_branching_conshdlr);
    auto vertex_branching_conss = SCIPconshdlrGetConss(heurdata->vertex_branching_conshdlr);
    debug_assert(n_vertex_branching_conss == 0 || vertex_branching_conss);
    const auto n_length_branching_conss = SCIPconshdlrGetNConss(heurdata->length_branching_conshdlr);
    auto length_branching_conss = SCIPconshdlrGetConss(heurdata->length_branching_conshdlr);
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

    // Reset unused costs.
    cost_offset = -astar.max_path_length() * 1e2;
    finish_time_penalties.clear();
#ifdef USE_GOAL_CONFLICTS
    goal_penalties.clear();
#endif

    // Create order of agents to solve.
    Vector<SCIP_VAR*> vars(N, nullptr);
    Vector<Agent> order;
    order.reserve(N);
    EdgePenalties global_edge_penalties;
    HashMap<Node, Time> node_latest_visit_time;
    for (Agent a = 0; a < N; ++a)
    {
        for (const auto& [var, var_val] : agent_vars[a])
        {
            debug_assert(var_val == SCIPgetSolVal(scip, nullptr, var));
            if (SCIPisEQ(scip, var_val, 1.0))
            {
                // Use this variable in the solution.
                vars[a] = var;

                // Get the path.
                auto vardata = SCIPvarGetData(var);
                const auto path_length = SCIPvardataGetPathLength(vardata);
                const auto path = SCIPvardataGetPath(vardata);

                // Block the path.
                block_path(path,
                           path_length,
                           global_edge_penalties,
                           node_latest_visit_time,
                           astar.max_path_length(),
                           map);

                // Advance to next agent.
                goto NEXT_AGENT;
            }
        }
        order.push_back(a);
        NEXT_AGENT:;
    }
    std::shuffle(order.begin(), order.end(), heurdata->rng);

    // Find a path for each agent.
    Vector<Vector<Edge>> paths(N);
    for (const auto a : order)
    {
        // Set up start and end points.
        start = agents[a].start;
        goal = agents[a].goal;

        // Modify edge costs for vertex branching decisions.
        waypoints.clear();
        edge_penalties = global_edge_penalties;
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

        // Delay the goal if the node has been visited by another agent.
        if (const auto it = node_latest_visit_time.find(goal);it != node_latest_visit_time.end())
        {
            earliest_goal_time = std::max(earliest_goal_time, it->second + 1);
        }

        // Preprocess input data.
        astar.preprocess_input();

        // Start timer.
// #ifdef PRINT_DEBUG
//         const auto start_time = std::chrono::high_resolution_clock::now();
// #endif

        // Solve.
        astar.before_solve(); // TODO: Merge back in.
        const auto [path_vertices, path_cost] = astar.solve<false>();

        // End timer.
// #ifdef PRINT_DEBUG
//         const auto end_time = std::chrono::high_resolution_clock::now();
//         const auto duration = std::chrono::duration<double>(end_time - start_time).count();
//         debugln("    Done in {:.4f} seconds", duration);
// #endif

        // Exit if no path is found.
        if (path_vertices.empty())
        {
            return SCIP_OKAY;
        }

        // Get the path.
        auto& path = paths[a];
        for (auto it = path_vertices.begin(); it != path_vertices.end(); ++it)
        {
            const auto d = it != path_vertices.end() - 1 ?
                            map.get_direction(it->n, (it + 1)->n) :
                            Direction::INVALID;
            path.push_back(Edge{it->n, d});
        }
        const auto path_length = path.size();

        // Print.
        // debugln("    Found path with length {} ({})", path.size(), format_path(probdata, path.size(), path.data()));

        // Block the path for future agents.
        block_path(path.data(),
                   path_length,
                   global_edge_penalties,
                   node_latest_visit_time,
                   astar.max_path_length(),
                   map);
    }

    // Check.
// #ifdef DEBUG
//     {
//         HashMap<NodeTime, SCIP_Real> used_vertices;
//         HashMap<EdgeTime, SCIP_Real> used_edges;

//         for (Agent a = 0; a < N; ++a)
//         {
//             // Get the path.
//             const auto& path = paths[a];
//             const auto path_length = path.size();

//             // Store everything except the last vertex.
//             Time t = 0;
//             for (; t < static_cast<Time>(path_length) - 1; ++t)
//             {
//                 // Store the vertex.
//                 {
//                     const NodeTime nt{path[t].n, t};
//                     used_vertices[nt] += 1;
//                 }

//                 // Store the edge.
//                 {
//                     const EdgeTime et{path[t], t};
//                     used_edges[et] += 1;
//                 }
//             }

//             // Store the edges after the agent reaches its goal.
//             const auto n = path[t].n;
//             for (; t < astar.max_path_length(); ++t)
//             {
//                 // Store the vertex.
//                 {
//                     const NodeTime nt{n, t};
//                     used_vertices[nt] += 1;
//                 }

//                 // Store the edge.
//                 {
//                     const EdgeTime et{n, Direction::WAIT, t};
//                     used_edges[et] += 1;
//                 }
//             }

//             // Store the last vertex.
//             {
//                 const NodeTime nt{n, t};
//                 used_vertices[nt] += 1;
//             }
//         }
//         for (const auto [nt, val] : used_vertices)
//         {
//             release_assert(val == 1, "({},{}) {}", map.get_x(nt.n), map.get_y(nt.n), nt.t);
//         }
//         for (const auto [et, val] : used_edges)
//         {
//             release_assert(val == 1, "({},{}) {} {}", map.get_x(et.n), map.get_y(et.n), et.t, et.et.e);
//         }
//     }
// #endif

    // Create a solution.
    SCIP_SOL* sol;
    SCIP_CALL(SCIPcreateSol(scip, &sol, heur));
#ifdef PRINT_DEBUG
    Cost sol_cost = 0;
#endif
    for (Agent a = 0; a < N; ++a)
    {
        // Create the variable if it doesn't already exist.
        if (!vars[a])
        {
            // Get the path.
            const auto& path = paths[a];
#ifdef DEBUG
            debug_assert(!path.empty());
#endif

            // Find the path if it already exists.
            for (const auto& [var, _] : agent_vars[a])
            {
                debug_assert(var);
                auto vardata = SCIPvarGetData(var);
                const auto existing_path_length = SCIPvardataGetPathLength(vardata);
                const auto existing_path = SCIPvardataGetPath(vardata);
                if (std::equal(path.begin(), path.end(), existing_path, existing_path + existing_path_length))
                {
                    vars[a] = var;
                    goto ADD_VAR_TO_SOL;
                }
            }

            // Add the path if not yet added.
            SCIP_CALL(SCIPprobdataAddInitialVar(scip,
                                                probdata,
                                                a,
                                                path.size(),
                                                path.data(),
                                                &vars[a]));
            debug_assert(vars[a]);
        }

        // Put the variable in the solution.
        ADD_VAR_TO_SOL:
        SCIP_CALL(SCIPsetSolVal(scip, sol, vars[a], 1.0));

        // Increment cost.
#ifdef PRINT_DEBUG
        sol_cost += SCIPvarGetObj(vars[a]);
#endif
    }
    debugln("Prioritized planning primal heuristic found solution with cost {}", sol_cost);

    // Inject solution.
    SCIP_Bool success;
    SCIP_CALL(SCIPtrySol(scip, sol, FALSE, FALSE, FALSE, TRUE, TRUE, &success));
    if (success)
    {
        *result = SCIP_FOUNDSOL;
    }

    // Deallocate.
    SCIP_CALL(SCIPfreeSol(scip, &sol));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Create the prioritized planning primal heuristic and include it in SCIP
SCIP_RETCODE SCIPincludeHeurPrioritizedPlanning(
    SCIP* scip
)
{
    // Create heuristic.
    SCIP_HEUR* heur;
    SCIP_CALL(SCIPincludeHeurBasic(scip,
                                   &heur,
                                   HEUR_NAME,
                                   HEUR_DESC,
                                   HEUR_DISPCHAR,
                                   HEUR_PRIORITY,
                                   HEUR_FREQ,
                                   HEUR_FREQOFS,
                                   HEUR_MAXDEPTH,
                                   HEUR_TIMING,
                                   HEUR_USESSUBSCIP,
                                   heurExecPrioritizedPlanning,
                                   nullptr));
    debug_assert(heur);

    // Set callbacks.
    SCIP_CALL(SCIPsetHeurInit(scip, heur, heurInitPrioritizedPlanning));
    SCIP_CALL(SCIPsetHeurFree(scip, heur, heurFreePrioritizedPlanning));

    // Done.
    return SCIP_OKAY;
}

#endif
