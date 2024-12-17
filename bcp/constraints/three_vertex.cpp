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

#ifdef USE_THREEVERTEX_CONFLICTS

// #define PRINT_DEBUG

#include "constraints/three_vertex.h"
#include "problem/problem.h"
#include "problem/variable_data.h"

#define SEPA_NAME         "three_vertex"
#define SEPA_DESC         "Separator for three vertex conflicts"
#define SEPA_PRIORITY     103      // priority of the constraint handler for separation
#define SEPA_FREQ         1        // frequency for separating cuts; zero means to separate only in the root node
#define SEPA_MAXBOUNDDIST 1.0
#define SEPA_USESSUBSCIP  FALSE    // does the separator use a secondary SCIP instance? */
#define SEPA_DELAY        FALSE    // should separation method be delayed, if other separators found cuts? */

SCIP_RETCODE threevertex_conflicts_create_cut(
    SCIP* scip,                 // SCIP
    SCIP_ProbData* probdata,    // Problem data
    SCIP_SEPA* sepa,            // Separator
    const Agent a1,             // Agent 1
    const Agent a2,             // Agent 2
    const EdgeTime a1_et1,      // Edge 1 of agent 1
    const EdgeTime a1_et2,      // Edge 2 of agent 1
    const NodeTime a2_v1,       // Vertex 1 of agent 2
    const NodeTime a2_v2,       // Vertex 2 of agent 2
    const NodeTime a2_v3,       // Vertex 3 of agent 2
    SCIP_Result* result         // Output result
)
{
    // Get problem data.
    const auto& map = SCIPprobdataGetMap(probdata);

    // Create constraint name.
#ifdef DEBUG
    const auto [a1_et1_x1, a1_et1_y1] = map.get_xy(a1_et1.n);
    const auto [a1_et1_x2, a1_et1_y2] = map.get_destination_xy(a1_et1);

    const auto [a1_et2_x1, a1_et2_y1] = map.get_xy(a1_et2.n);
    const auto [a1_et2_x2, a1_et2_y2] = map.get_destination_xy(a1_et2);

    const auto [a2_v1_x, a2_v1_y] = map.get_xy(a2_v1.n);
    const auto [a2_v2_x, a2_v2_y] = map.get_xy(a2_v2.n);
    const auto [a2_v3_x, a2_v3_y] = map.get_xy(a2_v3.n);

    auto name = fmt::format("threevertex_conflict("
                            "{},{},"
                            "(({},{}),({},{}),{}),"
                            "(({},{}),({},{}),{}),"
                            "(({},{}),{}),"
                            "(({},{}),{}),"
                            "(({},{}),{})"
                            ")",
                            a1, a2,
                            a1_et1_x1, a1_et1_y1, a1_et1_x2, a1_et1_y2, a1_et1.t,
                            a1_et2_x1, a1_et2_y1, a1_et2_x2, a1_et2_y2, a1_et2.t,
                            a2_v1_x, a2_v1_y, a2_v1.t,
                            a2_v2_x, a2_v2_y, a2_v2.t,
                            a2_v3_x, a2_v3_y, a2_v3.t);
#endif

    // Create data for the cut.
    TwoAgentRobustCut cut(scip, a1, a2, 2, 15
#ifdef DEBUG
        , std::move(name)
#endif
    );
    cut.a1_edge_time(0) = a1_et1;
    cut.a1_edge_time(1) = a1_et2;
    {
        debug_assert(a2_v1.t >= 1);
        const auto prev_time = a2_v1.t - 1;
        cut.a2_edge_time(0) = EdgeTime{map.get_south(a2_v1.n), Direction::NORTH, prev_time};
        cut.a2_edge_time(1) = EdgeTime{map.get_north(a2_v1.n), Direction::SOUTH, prev_time};
        cut.a2_edge_time(2) = EdgeTime{map.get_west(a2_v1.n), Direction::EAST, prev_time};
        cut.a2_edge_time(3) = EdgeTime{map.get_east(a2_v1.n), Direction::WEST, prev_time};
        cut.a2_edge_time(4) = EdgeTime{map.get_wait(a2_v1.n), Direction::WAIT, prev_time};
    }
    {
        debug_assert(a2_v2.t >= 1);
        const auto prev_time = a2_v2.t - 1;
        cut.a2_edge_time(5) = EdgeTime{map.get_south(a2_v2.n), Direction::NORTH, prev_time};
        cut.a2_edge_time(6) = EdgeTime{map.get_north(a2_v2.n), Direction::SOUTH, prev_time};
        cut.a2_edge_time(7) = EdgeTime{map.get_west(a2_v2.n), Direction::EAST, prev_time};
        cut.a2_edge_time(8) = EdgeTime{map.get_east(a2_v2.n), Direction::WEST, prev_time};
        cut.a2_edge_time(9) = EdgeTime{map.get_wait(a2_v2.n), Direction::WAIT, prev_time};
    }
    {
        debug_assert(a2_v3.t >= 1);
        const auto prev_time = a2_v3.t - 1;
        cut.a2_edge_time(10) = EdgeTime{map.get_south(a2_v3.n), Direction::NORTH, prev_time};
        cut.a2_edge_time(11) = EdgeTime{map.get_north(a2_v3.n), Direction::SOUTH, prev_time};
        cut.a2_edge_time(12) = EdgeTime{map.get_west(a2_v3.n), Direction::EAST, prev_time};
        cut.a2_edge_time(13) = EdgeTime{map.get_east(a2_v3.n), Direction::WEST, prev_time};
        cut.a2_edge_time(14) = EdgeTime{map.get_wait(a2_v3.n), Direction::WAIT, prev_time};
    }

    // Store the cut.
    SCIP_CALL(SCIPprobdataAddTwoAgentRobustCut(scip, probdata, sepa, std::move(cut), 2, result));

    // Done.
    return SCIP_OKAY;
}

// Separator
static
SCIP_RETCODE threevertex_conflicts_separate(
    SCIP* scip,            // SCIP
    SCIP_SEPA* sepa,       // Separator
    SCIP_RESULT* result    // Pointer to store the result of the separation call
)
{
    // Print.
    debugln("Starting separator for three-vertex conflicts on solution with obj {:.6f}:",
            SCIPgetSolOrigObj(scip, nullptr));

    // Print paths.
#ifdef PRINT_DEBUG
    print_used_paths(scip);
#endif

    // Get problem data.
    auto probdata = SCIPgetProbData(scip);
    const auto N = SCIPprobdataGetN(probdata);
    const auto& map = SCIPprobdataGetMap(probdata);

    // Skip this separator if an earlier separator found cuts.
    auto& found_cuts = SCIPprobdataGetFoundCutsIndicator(probdata);
    if (found_cuts)
    {
        return SCIP_OKAY;
    }

    // Get the vertices and edges fractionally used by each agent.
    const auto& fractional_vertices = SCIPprobdataGetFractionalVertices(probdata);
    const auto& fractional_move_edges = SCIPprobdataGetFractionalMoveEdges(probdata);

    // Find conflicts.
    for (Agent a1 = 0; a1 < N; ++a1)
    {
        // Get the edges of agent 1.
        const auto& fractional_move_edges_a1 = fractional_move_edges[a1];

        // Loop through the first edge of agent 1.
        for (const auto& [a1_et1, a1_et1_val] : fractional_move_edges_a1)
        {
            // Get the vertices of the edge.
            const auto a1_et1_orig = a1_et1.n;
            const auto [a1_et1_orig_x, a1_et1_orig_y] = map.get_xy(a1_et1.n);
            const auto a1_et1_dest = map.get_destination(a1_et1);

            // Loop through the second edge of agent 1.
            for (const auto& [a1_et2, a1_et2_val] : fractional_move_edges_a1)
            {
                // Get the vertices of the edge.
                const auto a1_et2_orig = a1_et2.n;
                const auto a1_et2_dest = map.get_destination(a1_et2);
                const auto [a1_et2_dest_x, a1_et2_dest_y] = map.get_xy(a1_et2_dest);

                // Continue if the two edges are one timestep apart and incompatible.
                if (a1_et2.t == a1_et1.t + 1 && a1_et2_orig != a1_et1_dest)
                {
                    // Loop through the second agent.
                    for (Agent a2 = 0; a2 < N; ++a2)
                        if (a1 != a2)
                        {
                            // Get the vertices of agent 2.
                            const auto& fractional_vertices_a2 = fractional_vertices[a2];

                            // Check if agent 2 uses the two vertices of agent 1.
                            const NodeTime a2_v1{a1_et1_orig, a1_et1.t};
                            const NodeTime a2_v2{a1_et2_dest, a1_et2.t + 1};
                            const auto a2_v1_it = fractional_vertices_a2.find(a2_v1);
                            const auto a2_v2_it = fractional_vertices_a2.find(a2_v2);
                            if (a2_v1_it != fractional_vertices_a2.end() && a2_v2_it != fractional_vertices_a2.end())
                            {
                                // Get the values of the vertices.
                                const auto a2_v1_val = a2_v1_it->second;
                                const auto a2_v2_val = a2_v2_it->second;

                                // Find the third vertex.
                                NodeTime final_a2_v3{Node(), std::numeric_limits<Time>::max()};
#ifdef PRINT_DEBUG
                                Float final_lhs = std::numeric_limits<Float>::quiet_NaN();
#endif
                                for (const auto& [a2_v3, a2_v3_val] : fractional_vertices_a2)
                                    if (a2_v3.t <= a2_v1.t && a2_v3.t < final_a2_v3.t)
                                    {
                                        const auto [a2_v3_x, a2_v3_y] = map.get_xy(a2_v3.n);

                                        // Get the time and distance apart from the first vertex.
                                        const auto time1 = a2_v1.t - a2_v3.t;
                                        const auto distance1 = abs(a1_et1_orig_x - a2_v3_x) +
                                                               abs(a1_et1_orig_y - a2_v3_y);

                                        // Get the time and distance apart from the second edge.
                                        const auto time2 = a2_v2.t - a2_v3.t;
                                        const auto distance2 = abs(a1_et2_dest_x - a2_v3_x) +
                                                               abs(a1_et2_dest_y - a2_v3_y);

                                        // Store the third edge if the cut is violated.
                                        const auto lhs = a1_et1_val + a1_et2_val +
                                                         a2_v1_val + a2_v2_val + a2_v3_val;
                                        if (distance1 > time1 && distance2 > time2 &&
                                            SCIPisSumGT(scip, lhs, 2 + CUT_VIOLATION))
                                        {
                                            final_a2_v3 = a2_v3;
#ifdef PRINT_DEBUG
                                            final_lhs = lhs;
#endif
                                        }
                                    }

                                    // Create the cut if the third vertex of agent 2 is found.
                                    if (final_a2_v3.t < std::numeric_limits<Time>::max())
                                    {
                                        // Print.
#ifdef PRINT_DEBUG
                                        {
                                            const auto [a1_et1_x1, a1_et1_y1] = map.get_xy(a1_et1.n);
                                            const auto [a1_et1_x2, a1_et1_y2] = map.get_destination_xy(a1_et1);

                                            const auto [a1_et2_x1, a1_et2_y1] = map.get_xy(a1_et2.n);
                                            const auto [a1_et2_x2, a1_et2_y2] = map.get_destination_xy(a1_et2);

                                            const auto [a2_v1_x, a2_v1_y] = map.get_xy(a2_v1.n);
                                            const auto [a2_v2_x, a2_v2_y] = map.get_xy(a2_v2.n);
                                            const auto [a2_v3_x, a2_v3_y] = map.get_xy(final_a2_v3.n);

                                            debugln("   Creating three-vertex conflict cut on edges "
                                                    "(({},{}),({},{}),{}) and (({},{}),({},{}),{}) for agent {} and "
                                                    "vertices (({},{}),{}), (({},{}),{}) and (({},{}),{}) for agent {} "
                                                    "with value {} in "
                                                    "branch-and-bound node {}",
                                                    a1_et1_x1, a1_et1_y1, a1_et1_x2, a1_et1_y2, a1_et1.t,
                                                    a1_et2_x1, a1_et2_y1, a1_et2_x2, a1_et2_y2, a1_et2.t,
                                                    a1,
                                                    a2_v1_x, a2_v1_y, a2_v1.t,
                                                    a2_v2_x, a2_v2_y, a2_v2.t,
                                                    a2_v3_x, a2_v3_y, final_a2_v3.t,
                                                    a2,
                                                    final_lhs,
                                                    SCIPnodeGetNumber(SCIPgetCurrentNode(scip)));
                                        }
#endif

                                        // Create cut.
                                        SCIP_CALL(threevertex_conflicts_create_cut(scip,
                                                                                   probdata,
                                                                                   sepa,
                                                                                   a1,
                                                                                   a2,
                                                                                   a1_et1,
                                                                                   a1_et2,
                                                                                   a2_v1,
                                                                                   a2_v2,
                                                                                   final_a2_v3,
                                                                                   result));
                                        found_cuts = true;
                                    }
                            }
                        }
                }
            }
        }
    }

    // Done.
    return SCIP_OKAY;
}

// Copy method for separator
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_SEPACOPY(sepaCopyThreeVertexConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);

    // Include separator.
    SCIP_CALL(SCIPincludeSepaThreeVertexConflicts(scip));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Separation method for LP solutions
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_SEPAEXECLP(sepaExeclpThreeVertexConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);
    debug_assert(result);

    // Start.
    *result = SCIP_DIDNOTFIND;

    // Start separator.
    SCIP_CALL(threevertex_conflicts_separate(scip, sepa, result));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Create separator for three-vertex conflicts constraints and include it in SCIP
SCIP_RETCODE SCIPincludeSepaThreeVertexConflicts(
    SCIP* scip    // SCIP
)
{
    // Check.
    debug_assert(scip);

    // Include separator.
    SCIP_Sepa* sepa = nullptr;
    SCIP_CALL(SCIPincludeSepaBasic(scip,
                                   &sepa,
                                   SEPA_NAME,
                                   SEPA_DESC,
                                   SEPA_PRIORITY,
                                   SEPA_FREQ,
                                   SEPA_MAXBOUNDDIST,
                                   SEPA_USESSUBSCIP,
                                   SEPA_DELAY,
                                   sepaExeclpThreeVertexConflicts,
                                   nullptr,
                                   nullptr));
    debug_assert(sepa);

    // Set callbacks.
    SCIP_CALL(SCIPsetSepaCopy(scip, sepa, sepaCopyThreeVertexConflicts));

    // Done.
    return SCIP_OKAY;
}

#endif
