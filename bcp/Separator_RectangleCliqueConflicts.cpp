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

#ifdef USE_RECTANGLE_CLIQUE_CONFLICTS

//#define PRINT_DEBUG

#include "Separator_RectangleCliqueConflicts.h"
#include "ProblemData.h"
#include "VariableData.h"
#include <algorithm>

#define SEPA_NAME                             "rectangle_clique"
#define SEPA_DESC     "Separator for rectangle clique conflicts"
#define SEPA_PRIORITY                                          1 // priority of the constraint handler for separation
#define SEPA_FREQ                                              1 // frequency for separating cuts; zero means to separate only in the root node
#define SEPA_MAXBOUNDDIST                                    1.0
#define SEPA_USESSUBSCIP                                   FALSE // does the separator use a secondary SCIP instance? */
#define SEPA_DELAY                                          TRUE // should separation method be delayed, if other separators found cuts? */

// Data for rectangle clique conflicts
struct RectangleCliqueConflictsSepaData
{
    Vector<RectangleConflict> conflicts;
};

SCIP_RETCODE rectangle_clique_conflicts_create_cut(
    SCIP* scip,                                    // SCIP
    SCIP_SEPA* sepa,                               // Separator
    RectangleCliqueConflictsSepaData& sepadata,    // Separator data
    const Vector<SCIP_VAR*>& a1_vars,              // Array of variables for agent 1
    const Vector<SCIP_VAR*>& a2_vars,              // Array of variables for agent 2
#if defined(DEBUG) or defined(PRINT_DEBUG)
    const Time start_t,                            // Time before entry
    const Position start_x1,                       // Start coordinate of agent 1
    const Position start_y1,                       // Start coordinate of agent 1
    const Position start_x2,                       // Start coordinate of agent 2
    const Position start_y2,                       // Start coordinate of agent 2
    const Time end_t,                              // Time after exit
    const Position end_x1,                         // End coordinate of agent 1
    const Position end_y1,                         // End coordinate of agent 1
    const Position end_x2,                         // End coordinate of agent 2
    const Position end_y2,                         // End coordinate of agent 2
#endif
    RectangleConflict& conflict,                   // Output rectangle conflict
    SCIP_Result* result                            // Output result
)
{
    // Create constraint name.
#ifdef DEBUG
    const auto name = fmt::format(
        "rectangle_clique_conflict({},{},({},({},{}),({},{})),({},({},{}),({},{})))",
        conflict.a1, conflict.a2,
        start_t, start_x1, start_y1, start_x2, start_y2,
        end_t, end_x1, end_y1, end_x2, end_y2);
#endif

    // Create a row.
    SCIP_ROW* row = nullptr;
    SCIP_CALL(SCIPcreateEmptyRowSepa(scip,
                                     &row,
                                     sepa,
#ifdef DEBUG
                                     name.c_str(),
#else
                                     "",
#endif
                                     -SCIPinfinity(scip),
                                     1.0,
                                     FALSE,
                                     TRUE,
                                     FALSE));
    debug_assert(row);

    // Add variables to the constraint.
#ifdef DEBUG
    SCIP_Real lhs = 0.0;
#endif
    SCIP_CALL(SCIPcacheRowExtensions(scip, row));
    for (auto var : a1_vars)
    {
        // Get the path.
        debug_assert(var);
        auto vardata = SCIPvarGetData(var);
        debug_assert(conflict.a1 == SCIPvardataGetAgent(vardata));
        const auto path_length = SCIPvardataGetPathLength(vardata);
        const auto path = SCIPvardataGetPath(vardata);

        // Determine if the rectangle is used.
        Int count = 0;
        for (auto [it, end] = conflict.agent1_edges(); it != end; ++it)
        {
            const auto& [e, t] = *it;
            if (t < path_length - 1 && path[t] == e)
            {
                count++;
            }
        }
        debug_assert(count <= 2);
        if (count == 2)
        {
#ifdef DEBUG
            lhs += SCIPgetVarSol(scip, var);
#endif
            SCIP_CALL(SCIPaddVarToRow(scip, row, var, 1));
        }

        // Print.
#ifdef PRINT_DEBUG
        const auto var_val = SCIPgetSolVal(scip, nullptr, var);
        if (SCIPisPositive(scip, var_val))
        {
            auto probdata = SCIPgetProbData(scip);
            debugln("      val: {:7.4f}:, agent: {:2d}, coeff: {:2d}, path: {}",
                    var_val, conflict.a1, 1,
                    format_path_spaced(probdata, path_length, path));
        }
#endif
    }
    for (auto var : a2_vars)
    {
        // Get the path.
        debug_assert(var);
        auto vardata = SCIPvarGetData(var);
        debug_assert(conflict.a2 == SCIPvardataGetAgent(vardata));
        const auto path_length = SCIPvardataGetPathLength(vardata);
        const auto path = SCIPvardataGetPath(vardata);

        // Determine if the rectangle is used.
        Int count = 0;
        for (auto [it, end] = conflict.agent2_edges(); it != end; ++it)
        {
            const auto& [e, t] = *it;
            if (t < path_length - 1 && path[t] == e)
            {
                count++;
            }
        }
        debug_assert(count <= 2);
        if (count == 2)
        {
#ifdef DEBUG
            lhs += SCIPgetVarSol(scip, var);
#endif
            SCIP_CALL(SCIPaddVarToRow(scip, row, var, 1));
        }

        // Print.
#ifdef PRINT_DEBUG
        const auto var_val = SCIPgetSolVal(scip, nullptr, var);
        if (SCIPisPositive(scip, var_val))
        {
            auto probdata = SCIPgetProbData(scip);
            debugln("      val: {:7.4f}:, agent: {:2d}, coeff: {:2d}, path: {}",
                    var_val, conflict.a2, 1,
                    format_path_spaced(probdata, path_length, path));
        }
#endif
    }
    SCIP_CALL(SCIPflushRowExtensions(scip, row));
    debug_assert(SCIPisSumGT(scip, lhs, 1.0));

    // Print.
    debugln("   Creating cut for rectangle clique conflict by agent {} from ({},{}) to "
            "({},{}) and agent {} from ({},{}) to ({},{}) between times {} and {} in "
            "branch-and-bound node {}",
            conflict.a1, start_x1, start_y1, end_x1, end_y1,
            conflict.a2, start_x2, start_y2, end_x2, end_y2,
            start_t, end_t,
            SCIPnodeGetNumber(SCIPgetCurrentNode(scip)));

    // Add the row to the LP.
    SCIP_Bool infeasible;
    SCIP_CALL(SCIPaddRow(scip, row, TRUE, &infeasible));

    // Set status.
    if (infeasible)
    {
        *result = SCIP_CUTOFF;
    }
    else
    {
        *result = SCIP_SEPARATED;
    }

    // Store the constraint.
    conflict.row = row;
    sepadata.conflicts.push_back(std::move(conflict));

    // Done.
    return SCIP_OKAY;
}

static
RectangleConflict find_rectangle(
    SCIP* scip,                                                   // SCIP
    const Map& map,                                               // Map
    const Vector<HashTable<EdgeTime, SCIP_Real>>& agent_edges,    // Edge weights for each agent
    const NodeTime nt,                                            // Node-time of the conflict
    const Agent a1,                                               // Agent 1
    const Agent a2,                                               // Agent 2
    SCIP_VAR* var1,                                               // Path of agent 1
    SCIP_VAR* var2,                                               // Path of agent 2
    const Int a1_nvars,                                           // Number of variables for agent 1
    const Int a2_nvars,                                           // Number of variables for agent 2
    SCIP_VAR** a1_vars,                                           // Array of variables for agent 1
    SCIP_VAR** a2_vars                                            // Array of variables for agent 2
#if defined(DEBUG) or defined(PRINT_DEBUG)
  , SCIP_ProbData* probdata,                                      // Problem data
    Time& output_start_t,                                         // Time before entry
    Time& output_end_t,                                           // Time after exit
    Position& output_start_x1,                                    // Start coordinate of agent 1
    Position& output_start_y1,                                    // Start coordinate of agent 1
    Position& output_start_x2,                                    // Start coordinate of agent 2
    Position& output_start_y2,                                    // Start coordinate of agent 2
    Position& output_end_x1,                                      // End coordinate of agent 1
    Position& output_end_y1,                                      // End coordinate of agent 1
    Position& output_end_x2,                                      // End coordinate of agent 2
    Position& output_end_y2                                       // End coordinate of agent 2
#endif
)
{
    // Create output.
    RectangleConflict conflict;
    conflict.a1 = a1;
    conflict.a2 = a2;

    // Get the path of agent 1.
    auto vardata1 = SCIPvarGetData(var1);
    const auto path_length1 = SCIPvardataGetPathLength(vardata1);
    const auto path1 = SCIPvardataGetPath(vardata1);

    // Get the path of agent 2.
    auto vardata2 = SCIPvarGetData(var2);
    const auto path_length2 = SCIPvardataGetPathLength(vardata2);
    const auto path2 = SCIPvardataGetPath(vardata2);

    // Print.
#ifdef PRINT_DEBUG
    {
        auto probdata = SCIPgetProbData(scip);
        debugln("---------");
        fmt::print("                  ");
        for (Time t = 0; t < std::max(path_length1, path_length2); ++t)
            fmt::print("{:10d}", t);
        debugln("");
        debugln("   agent {:2d}, path {}",
                a1, format_path_spaced(probdata, path_length1, path1));
        debugln("   agent {:2d}, path {}",
                a2, format_path_spaced(probdata, path_length2, path2));
    }
#endif

    // Get the length of the shorter path.
    const auto min_path_length = std::min(path_length1, path_length2);

    // Get time of the vertex conflict.
    const auto conflict_time = nt.t;

    // A rectangle conflict can only occur if the two paths visit the same vertex.
    if (!(0 < conflict_time && conflict_time < min_path_length - 1 &&
          path1[conflict_time].n == nt.n &&
          path2[conflict_time].n == nt.n))
    {
        return conflict;
    }

    // Get the movement directions.
    Direction y_dir = Direction::INVALID;
    Direction x_dir = Direction::INVALID;
    for (Time t = conflict_time; t < min_path_length; ++t)
    {
        if (path1[t].d == Direction::NORTH || path1[t].d == Direction::SOUTH)
        {
            y_dir = path1[t].d;
            break;
        }
        else if (path2[t].d == Direction::NORTH || path2[t].d == Direction::SOUTH)
        {
            y_dir = path2[t].d;
            break;
        }
        else if (path1[t].d == Direction::WAIT || path2[t].d == Direction::WAIT)
        {
            return conflict;
        }
    }
    for (Time t = conflict_time; t < min_path_length; ++t)
    {
        if (path1[t].d == Direction::EAST || path1[t].d == Direction::WEST)
        {
            x_dir = path1[t].d;
            break;
        }
        else if (path2[t].d == Direction::EAST || path2[t].d == Direction::WEST)
        {
            x_dir = path2[t].d;
            break;
        }
        else if (path1[t].d == Direction::WAIT || path2[t].d == Direction::WAIT)
        {
            return conflict;
        }
    }
    if (y_dir == Direction::INVALID || x_dir == Direction::INVALID)
    {
        return conflict;
    }

    // Find the first time when the direction changes.
    Time start_t = conflict_time;
    Time end_t = conflict_time;
    for (; start_t >= 0; --start_t)
        if ((path1[start_t].d != y_dir && path1[start_t].d != x_dir) ||
            (path2[start_t].d != y_dir && path2[start_t].d != x_dir))
        {
            break;
        }
    start_t++;
    for (; end_t < min_path_length - 1; ++end_t)
        if ((path1[end_t].d != y_dir && path1[end_t].d != x_dir) ||
            (path2[end_t].d != y_dir && path2[end_t].d != x_dir))
        {
            break;
        }
    if (end_t <= start_t + 2)
    {
        return conflict;
    }
    release_assert(0 <= start_t && end_t > start_t + 2 && end_t < min_path_length);

    // Cannot find a rectangle conflict if the start location of the two agents are the
    // same.
    if (path1[start_t].n == path2[start_t].n)
    {
        return conflict;
    }

    // Check.
#ifdef DEBUG
    for (Time t = start_t; t < end_t; ++t)
    {
        debug_assert(path1[t].d != Direction::WAIT);
        debug_assert(path2[t].d != Direction::WAIT);
    }
#endif

    // Get the coordinates of those times.
    const auto [start_x1, start_y1] = map.get_xy(path1[start_t].n);
    const auto [start_x2, start_y2] = map.get_xy(path2[start_t].n);
    const auto [end_x1, end_y1] = map.get_xy(path1[end_t].n);
    const auto [end_x2, end_y2] = map.get_xy(path2[end_t].n);

    // Compute the rectangle.
    compute_rectangle(map,
                      start_t, end_t,
                      start_x1, start_y1, start_x2, start_y2,
                      end_x1, end_y1, end_x2, end_y2,
                      conflict);

    // Determine if the cut is violated.
    if (!conflict.empty())
    {
        SCIP_Real lhs = 0.0;
        for (Int v = 0; v < a1_nvars; ++v)
        {
            // Get the variable.
            auto var = a1_vars[v];
            debug_assert(var);

            // Get the path.
            auto vardata = SCIPvarGetData(var);
            debug_assert(conflict.a1 == SCIPvardataGetAgent(vardata));
            const auto path_length = SCIPvardataGetPathLength(vardata);
            const auto path = SCIPvardataGetPath(vardata);

            // Determine if the rectangle is used.
            Int count = 0;
            for (auto [it, end] = conflict.agent1_edges(); it != end; ++it)
            {
                const auto& [e, t] = *it;
                if (t < path_length - 1 && path[t] == e)
                {
                    count++;
                }
            }
            debug_assert(count <= 2);
            if (count == 2)
            {
                lhs += SCIPgetVarSol(scip, var);
            }
        }
        for (Int v = 0; v < a2_nvars; ++v)
        {
            // Get the variable.
            auto var = a2_vars[v];
            debug_assert(var);

            // Get the path.
            auto vardata = SCIPvarGetData(var);
            debug_assert(conflict.a2 == SCIPvardataGetAgent(vardata));
            const auto path_length = SCIPvardataGetPathLength(vardata);
            const auto path = SCIPvardataGetPath(vardata);

            // Determine if the rectangle is used.
            Int count = 0;
            for (auto [it, end] = conflict.agent2_edges(); it != end; ++it)
            {
                const auto& [e, t] = *it;
                if (t < path_length - 1 && path[t] == e)
                {
                    count++;
                }
            }
            debug_assert(count <= 2);
            if (count == 2)
            {
                lhs += SCIPgetVarSol(scip, var);
            }
        }
        if (!SCIPisSumGT(scip, lhs, 1.0 + CUT_VIOLATION))
        {
            conflict.edges.clear();
        }
    }

    // Store coordinates.
#if defined(DEBUG) or defined(PRINT_DEBUG)
    output_start_t = start_t;
    output_end_t = end_t;
    output_start_x1 = start_x1;
    output_start_y1 = start_y1;
    output_start_x2 = start_x2;
    output_start_y2 = start_y2;
    output_end_x1 = end_x1;
    output_end_y1 = end_y1;
    output_end_x2 = end_x2;
    output_end_y2 = end_y2;
#endif

    // Return.
    return conflict;
}

// Separator
static
SCIP_RETCODE rectangle_clique_conflicts_separate(
    SCIP* scip,            // SCIP
    SCIP_SEPA* sepa,       // Separator
    SCIP_RESULT* result    // Pointer to store the result of the separation call
)
{
    // Print.
    debugln("Starting separator for rectangle clique conflicts on solution with obj "
            "{:.6f}:",
            SCIPgetSolOrigObj(scip, nullptr));

    // Print paths.
#ifdef PRINT_DEBUG
    print_used_paths(scip);
#endif

    // Get separator data.
    auto sepadata = reinterpret_cast<RectangleCliqueConflictsSepaData*>(SCIPsepaGetData(sepa));
    debug_assert(sepadata);

    // Get problem data.
    auto probdata = SCIPgetProbData(scip);
    const auto N = SCIPprobdataGetN(probdata);
    const auto& map = SCIPprobdataGetMap(probdata);

    // Get variables.
    const auto& agent_vars = SCIPprobdataGetAgentVars2(probdata);

    // Get conflicting paths and vertices of each agent.
    Vector<Vector<SCIP_VAR*>> agent_paths(N);
    Vector<Vector<NodeTime>> agent_vertices(N);
    Vector<HashTable<EdgeTime, SCIP_Real>> agent_edges(N);
    for (Agent a = 0; a < N; ++a)
    {
        // Get agent-specific data.
        auto& agent_paths_a = agent_paths[a];
        auto& agent_vertices_a = agent_vertices[a];
        auto& agent_edges_a = agent_edges[a];

        // Calculate the number of times a vertex is used by summing the columns.
        for (auto var : agent_vars[a])
        {
            // Get the path.
            debug_assert(var);
            auto vardata = SCIPvarGetData(var);
            const auto path_length = SCIPvardataGetPathLength(vardata);
            const auto path = SCIPvardataGetPath(vardata);

            // Get the variable value.
            const auto var_val = SCIPgetSolVal(scip, nullptr, var);

            // Append the path.
            if (SCIPisPositive(scip, var_val))
            {
                agent_paths_a.push_back(var);

                {
                    const EdgeTime et{path[0], 0};
                    agent_edges_a[et] += var_val;
                }

                for (Time t = 1; t < path_length; ++t)
                {
                    {
                        const NodeTime nt{path[t].n, t};
                        if (std::find(agent_vertices_a.begin(),
                                      agent_vertices_a.end(),
                                      nt) == agent_vertices_a.end())
                        {
                            agent_vertices_a.push_back(nt);
                        }
                    }

                    {
                        const EdgeTime et{path[t], t};
                        agent_edges_a[et] += var_val;
                    }
                }
            }
        }

        // Sort.
        std::sort(agent_vertices_a.begin(),
                  agent_vertices_a.end(),
                  [](const NodeTime a, const NodeTime b)
                  {
                      return (a.t < b.t) || (a.t == b.t && a.n < b.n);
                  });

        // Print.
//#ifdef PRINT_DEBUG
//        if (!agent_vertices_a.empty())
//        {
//            debugln("   Used vertices for agent {}:", a);
//            for (const auto tn : agent_vertices_a)
//            {
//                uint32_t x, y;
//                gm.to_unpadded_xy(tn.id, x, y);
//                debugln("      agent {:2d}, x {:2d}, y {:2d}, t {:3d}",
//                        a, x, y, tn.t);
//            }
//        }
//#endif
    }

    // Find conflicts.
    Vector<NodeTime> common_vertices;
    for (Agent a1 = 0; a1 < N - 1; ++a1)
        for (Agent a2 = a1 + 1; a2 < N; ++a2)
        {
            // Find common vertices.
            common_vertices.clear();
            std::set_intersection(agent_vertices[a1].begin(), agent_vertices[a1].end(),
                                  agent_vertices[a2].begin(), agent_vertices[a2].end(),
                                  std::back_inserter(common_vertices),
                                  [](const NodeTime a, const NodeTime b)
                                  {
                                      return (a.t < b.t) || (a.t == b.t && a.n < b.n);
                                  });

            // Find a conflict.
            for (const auto nt : common_vertices)
            {
                // Print.
                debugln("Checking conflict at ({},{}) time {}",
                        map.get_x(nt.n), map.get_y(nt.n), nt.t);

                // Check every path.
                for (auto p1 : agent_paths[a1])
                    for (auto p2 : agent_paths[a2])
                    {
#if defined(DEBUG) or defined(PRINT_DEBUG)
                        Time start_t;
                        Time end_t;
                        Position start_x1;
                        Position start_y1;
                        Position start_x2;
                        Position start_y2;
                        Position end_x1;
                        Position end_y1;
                        Position end_x2;
                        Position end_y2;
#endif
                        auto conflict = find_rectangle(scip,
                                                       map,
                                                       agent_edges,
                                                       nt,
                                                       a1,
                                                       a2,
                                                       p1,
                                                       p2,
                                                       agent_vars[a1],
                                                       agent_vars[a2]
#if defined(DEBUG) or defined(PRINT_DEBUG)
                                                     , probdata,
                                                       start_t,
                                                       end_t,
                                                       start_x1,
                                                       start_y1,
                                                       start_x2,
                                                       start_y2,
                                                       end_x1,
                                                       end_y1,
                                                       end_x2,
                                                       end_y2
#endif
                                                       );

                        // Stop if no rectangle was found.
                        if (conflict.empty())
                            continue;

                        // Create the cut.
                        SCIP_CALL(rectangle_clique_conflicts_create_cut(scip,
                                                                        sepa,
                                                                        *sepadata,
                                                                        agent_vars[a1],
                                                                        agent_vars[a2],
#if defined(DEBUG) or defined(PRINT_DEBUG)
                                                                        start_t,
                                                                        start_x1,
                                                                        start_y1,
                                                                        start_x2,
                                                                        start_y2,
                                                                        end_t,
                                                                        end_x1,
                                                                        end_y1,
                                                                        end_x2,
                                                                        end_y2,
#endif
                                                                        conflict,
                                                                        result));
                        goto NEXT_AGENT;
                    }
            }
        NEXT_AGENT:;
        }

    // Done.
    return SCIP_OKAY;
}

// Copy method for separator
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_SEPACOPY(sepaCopyRectangleCliqueConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);

    // Include separator.
    SCIP_SEPA* sepa_copy;
    SCIP_CALL(SCIPincludeSepaRectangleCliqueConflicts(scip, &sepa_copy));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Free rows
static
SCIP_DECL_SEPAEXITSOL(sepaExitsolRectangleCliqueConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);

    // Get separator data.
    auto sepadata = reinterpret_cast<RectangleCliqueConflictsSepaData*>(SCIPsepaGetData(sepa));
    debug_assert(sepadata);

    // Free row for each rectangle clique conflict.
    for (auto& conflict : sepadata->conflicts)
    {
        SCIP_CALL(SCIPreleaseRow(scip, &conflict.row));
    }
    sepadata->conflicts.clear();

    // Done.
    return SCIP_OKAY;
}

// Free separator data
static
SCIP_DECL_SEPAFREE(sepaFreeRectangleCliqueConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);

    // Get separator data.
    auto sepadata = reinterpret_cast<RectangleCliqueConflictsSepaData*>(SCIPsepaGetData(sepa));
    debug_assert(sepadata);

    // Free memory.
    sepadata->~RectangleCliqueConflictsSepaData();
    SCIPfreeBlockMemory(scip, &sepadata);

    // Done.
    return SCIP_OKAY;
}

// Separation method for LP solutions
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_SEPAEXECLP(sepaExeclpRectangleCliqueConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);

    // Start.
    *result = SCIP_DIDNOTFIND;

    // Start separator.
    SCIP_CALL(rectangle_clique_conflicts_separate(scip, sepa, result));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Creates separator for rectangle clique conflicts constraints and include it in SCIP
SCIP_RETCODE SCIPincludeSepaRectangleCliqueConflicts(
    SCIP* scip,         // SCIP
    SCIP_SEPA** sepa    // Output pointer to separator
)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);

    // Create separator data.
    RectangleCliqueConflictsSepaData* sepadata = nullptr;
    SCIP_CALL(SCIPallocBlockMemory(scip, &sepadata));
    debug_assert(sepadata);
    new(sepadata) RectangleCliqueConflictsSepaData;
    sepadata->conflicts.reserve(500);

    // Include separator.
    SCIP_CALL(SCIPincludeSepaBasic(scip,
                                   sepa,
                                   SEPA_NAME,
                                   SEPA_DESC,
                                   SEPA_PRIORITY,
                                   SEPA_FREQ,
                                   SEPA_MAXBOUNDDIST,
                                   SEPA_USESSUBSCIP,
                                   SEPA_DELAY,
                                   sepaExeclpRectangleCliqueConflicts,
                                   nullptr,
                                   reinterpret_cast<SCIP_SEPADATA*>(sepadata)));
    debug_assert(*sepa);

    // Set callbacks.
    SCIP_CALL(SCIPsetSepaCopy(scip, *sepa, sepaCopyRectangleCliqueConflicts));
    SCIP_CALL(SCIPsetSepaFree(scip, *sepa, sepaFreeRectangleCliqueConflicts));
    SCIP_CALL(SCIPsetSepaExitsol(scip, *sepa, sepaExitsolRectangleCliqueConflicts));

    // Done.
    return SCIP_OKAY;
}

SCIP_RETCODE rectangle_clique_conflicts_add_var(
    SCIP* scip,                 // SCIP
    SCIP_SEPA* sepa,            // Separator for rectangle clique conflicts
    SCIP_VAR* var,              // Variable
    const Agent a,              // Agent
    const Time path_length,     // Path length
    const Edge* const path      // Path
)
{
    // Get separator data.
    debug_assert(sepa);
    auto sepadata = reinterpret_cast<RectangleCliqueConflictsSepaData*>(SCIPsepaGetData(sepa));
    debug_assert(sepadata);

    // Check.
    debug_assert(var);
    debug_assert(SCIPvarIsTransformed(var));

    // Add variable to constraints.
    for (const auto& conflict : sepadata->conflicts)
        if (a == conflict.a1 || a == conflict.a2)
        {
            // Determine if the rectangle is used.
            Int count = 0;
            for (auto [it, end] = conflict.agent_edges(a); it != end; ++it)
            {
                const auto& [e, t] = *it;
                if (t < path_length - 1 && path[t] == e)
                {
                    count++;
                }
            }
            debug_assert(count <= 2);
            if (count == 2)
            {
                SCIP_CALL(SCIPaddVarToRow(scip, conflict.row, var, 1));
            }
        }

    // Return.
    return SCIP_OKAY;
}

const Vector<RectangleConflict>& rectangle_clique_conflicts_get_constraints(
    SCIP_ProbData* probdata    // Problem data
)
{
    auto sepa = SCIPprobdataGetRectangleCliqueConflictsSepa(probdata);
    debug_assert(sepa);
    auto sepadata = reinterpret_cast<RectangleCliqueConflictsSepaData*>(SCIPsepaGetData(sepa));
    debug_assert(sepadata);
    return sepadata->conflicts;
}

#endif
