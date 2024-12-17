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

#ifdef USE_VERTEX_FOUREDGE_CONFLICTS

// #define PRINT_DEBUG

#include "constraints/vertex_four_edge.h"
#include "problem/problem.h"
#include "problem/variable_data.h"

#define SEPA_NAME         "vertex_four_edge"
#define SEPA_DESC         "Separator for vertex four edge conflicts"
#define SEPA_PRIORITY     102      // priority of the constraint handler for separation
#define SEPA_FREQ         1        // frequency for separating cuts; zero means to separate only in the root node
#define SEPA_MAXBOUNDDIST 1.0
#define SEPA_USESSUBSCIP  FALSE    // does the separator use a secondary SCIP instance? */
#define SEPA_DELAY        FALSE    // should separation method be delayed, if other separators found cuts? */

SCIP_RETCODE vertexfouredge_conflicts_create_cut(
    SCIP* scip,                        // SCIP
    SCIP_ProbData* probdata,           // Problem data
    SCIP_SEPA* sepa,                   // Separator
    const Agent a1,                    // Agent 1
    const Agent a2,                    // Agent 2
    const EdgeTime a1_et1,             // Edge-time 1 of agent 1
    const EdgeTime a1_et2,             // Edge-time 2 of agent 1
    const Vector<NodeTime>& a1_nts,    // Node-times of agent 1
    const EdgeTime a2_et1,             // Edge-time 1 of agent 2
    const EdgeTime a2_et2,             // Edge-time 2 of agent 2
    SCIP_Result* result                // Output result
)
{
    // Get map.
    const auto& map = SCIPprobdataGetMap(probdata);

    // Create constraint name.
#ifdef DEBUG
    const auto [a1_et1_x1, a1_et1_y1] = map.get_xy(a1_et1.n);
    const auto [a1_et1_x2, a1_et1_y2] = map.get_destination_xy(a1_et1);

    const auto [a1_et2_x1, a1_et2_y1] = map.get_xy(a1_et2.n);
    const auto [a1_et2_x2, a1_et2_y2] = map.get_destination_xy(a1_et2);

    const auto a1_nt = a1_nts.front();
    const auto [a1_nt_x1, a1_nt_y1] = map.get_xy(a1_nt.n);

    const auto [a2_et1_x1, a2_et1_y1] = map.get_xy(a2_et1.n);
    const auto [a2_et1_x2, a2_et1_y2] = map.get_destination_xy(a2_et1);

    const auto [a2_et2_x1, a2_et2_y1] = map.get_xy(a2_et2.n);
    const auto [a2_et2_x2, a2_et2_y2] = map.get_destination_xy(a2_et2);

    auto name = fmt::format("vertexfouredge_conflict("
                            "{},(({},{}),({},{}),{}),(({},{}),({},{}),{}),(({},{}),{}),"
                            "{},(({},{}),({},{}),{}),(({},{}),({},{}),{}))",
                            a1,
                            a1_et1_x1, a1_et1_y1, a1_et1.t, a1_et1_x2, a1_et1_y2,
                            a1_et2_x1, a1_et2_y1, a1_et2.t, a1_et2_x2, a1_et2_y2,
                            a1_nt_x1, a1_nt_y1, a1_nt.t,
                            a2,
                            a2_et1_x1, a2_et1_y1, a2_et1.t, a2_et1_x2, a2_et1_y2,
                            a2_et2_x1, a2_et2_y1, a2_et2.t, a2_et2_x2, a2_et2_y2);
#endif

    // Create data for the cut.
    TwoAgentRobustCut cut(scip,
                          a1,
                          a2,
                          2 + 5 * a1_nts.size(),
                          2
#ifdef DEBUG
                        , std::move(name)
#endif
    );
    cut.a1_edge_time(0) = a1_et1;
    cut.a1_edge_time(1) = a1_et2;
    {
        Int idx = 2;
        for (const auto a1_nt : a1_nts)
        {
            debug_assert(a1_nt.t >= 1);
            const auto prev_time = a1_nt.t - 1;
            cut.a1_edge_time(idx + 0) = EdgeTime{map.get_south(a1_nt.n), Direction::NORTH, prev_time};
            cut.a1_edge_time(idx + 1) = EdgeTime{map.get_north(a1_nt.n), Direction::SOUTH, prev_time};
            cut.a1_edge_time(idx + 2) = EdgeTime{map.get_west(a1_nt.n), Direction::EAST, prev_time};
            cut.a1_edge_time(idx + 3) = EdgeTime{map.get_east(a1_nt.n), Direction::WEST, prev_time};
            cut.a1_edge_time(idx + 4) = EdgeTime{map.get_wait(a1_nt.n), Direction::WAIT, prev_time};
            idx += 5;
        }
    }
    cut.a2_edge_time(0) = a2_et1;
    cut.a2_edge_time(1) = a2_et2;

    // Store the cut.
    SCIP_CALL(SCIPprobdataAddTwoAgentRobustCut(scip, probdata, sepa, std::move(cut), 2, result));

    // Done.
    return SCIP_OKAY;
}

// Separator
static
SCIP_RETCODE vertexfouredge_conflicts_separate(
    SCIP* scip,            // SCIP
    SCIP_SEPA* sepa,       // Separator
    SCIP_RESULT* result    // Pointer to store the result of the separation call
)
{
    // Print.
    debugln("Starting separator for vertex four-edge conflicts on solution with obj {:.6f}:",
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

    // Get the edges fractionally used by each agent.
    const auto& fractional_vertices = SCIPprobdataGetFractionalVertices(probdata);
    const auto& fractional_move_edges = SCIPprobdataGetFractionalMoveEdges(probdata);
    const auto& fractional_edges = SCIPprobdataGetFractionalEdges(probdata);

    // Find conflicts.
    for (Agent a1 = 0; a1 < N; ++a1)
    {
        // Get the vertices and edges of agent 1.
        const auto& fractional_vertices_a1 = fractional_vertices[a1];
        const auto& fractional_move_edges_a1 = fractional_move_edges[a1];

        // Loop through the first edge of agent 1.
        for (const auto& [a1_et1, a1_et1_val] : fractional_move_edges_a1)
        {
            // Get the coordinates of the first edge of agent 1.
            const auto [a1_et1_x, a1_et1_y] = map.get_xy(a1_et1.n);

            // Get the destination of the first edge of agent 1.
            const auto a1_et1_dest = map.get_destination(a1_et1);

            // Get the first edge of agent 2.
            const EdgeTime a2_et1{map.get_opposite_edge(a1_et1.et.e), a1_et1.t};

            // Loop through the second edge of agent 1.
            for (const auto& [a1_et2, a1_et2_val] : fractional_move_edges_a1)
                if (a1_et2.t == a1_et1.t + 1)
                {
                    // Get the coordinates of the second edge of agent 1.
                    const auto [a1_et2_x, a1_et2_y] = map.get_xy(a1_et2.n);

                    // Find cut of type A.
                    if (a1_et2.n == a1_et1_dest)
                    {
                        // Get the second edge of agent 2.
                        const EdgeTime a2_et2{map.get_opposite_edge(a1_et2.et.e), a1_et2.t};
                        if (a2_et2.n != a1_et1.n)
                        {
                            // Found the four edges. Look for an agent 2 that uses these edges.
                            for (Agent a2 = 0; a2 < N; ++a2)
                                if (a1 != a2)
                                {
                                    // Get the value of the edges of agent 2.
                                    const auto& fractional_move_edges_a2 = fractional_move_edges[a2];
                                    const auto a2_et1_it = fractional_move_edges_a2.find(a2_et1);
                                    const auto a2_et2_it = fractional_move_edges_a2.find(a2_et2);
                                    if (a2_et1_it == fractional_move_edges_a2.end() || a2_et2_it == fractional_move_edges_a2.end())
                                    {
                                        continue;
                                    }
                                    const auto a2_et1_val = a2_et1_it->second;
                                    const auto a2_et2_val = a2_et2_it->second;

                                    // Get the vertices of agent 1.
                                    for (Time t = 1; t <= a1_et1.t; ++t)
                                    {
                                        // Get all incompatible vertices at time t.
                                        Vector<NodeTime> a1_nts;
                                        Float a1_nts_val = 0;
                                        for (const auto& [a1_nt, a1_nt_val] : fractional_vertices_a1)
                                            if (a1_nt.t == t)
                                            {
                                                const auto [a1_nt_x, a1_nt_y] = map.get_xy(a1_nt.n);
                                                const auto dist1 = std::abs(a1_nt_x - a1_et1_x) + std::abs(a1_nt_y - a1_et1_y);
                                                const auto dist2 = std::abs(a1_nt_x - a1_et2_x) + std::abs(a1_nt_y - a1_et2_y);
                                                if (dist1 > a1_et1.t - a1_nt.t && dist2 > a1_et2.t - a1_nt.t)
                                                {
                                                    a1_nts.push_back(a1_nt);
                                                    a1_nts_val += a1_nt_val;
                                                }
                                            }

                                        // Create the constraint if violated.
                                        const auto lhs = a1_et1_val + a1_et2_val + a2_et1_val + a2_et2_val + a1_nts_val;
                                        if (a1_nts_val > 0 && SCIPisSumGT(scip, lhs, 2.0 + CUT_VIOLATION))
                                        {
                                            // Print.
#ifdef PRINT_DEBUG
                                            {
                                                    const auto [a1_et1_x1, a1_et1_y1] = map.get_xy(a1_et1.n);
                                                    const auto [a1_et1_x2, a1_et1_y2] = map.get_destination_xy(a1_et1);

                                                    const auto [a1_et2_x1, a1_et2_y1] = map.get_xy(a1_et2.n);
                                                    const auto [a1_et2_x2, a1_et2_y2] = map.get_destination_xy(a1_et2);

                                                    const auto a1_nt = a1_nts.front();
                                                    const auto [a1_nt_x1, a1_nt_y1] = map.get_xy(a1_nt.n);

                                                    const auto [a2_et1_x1, a2_et1_y1] = map.get_xy(a2_et1.n);
                                                    const auto [a2_et1_x2, a2_et1_y2] = map.get_destination_xy(a2_et1);

                                                    const auto [a2_et2_x1, a2_et2_y1] = map.get_xy(a2_et2.n);
                                                    const auto [a2_et2_x2, a2_et2_y2] = map.get_destination_xy(a2_et2);

                                                    debugln("   Creating vertex four-edge conflict cut (type A) on "
                                                            "(({},{}),({},{}),{}), (({},{}),({},{}),{}) and (({},{}),{}) for agent {} "
                                                            "and edges "
                                                            "(({},{}),({},{}),{}) and (({},{}),({},{}),{}) for agent {} "
                                                            "with value {} in branch-and-bound node {}",
                                                            a1_et1_x1, a1_et1_y1, a1_et1_x2, a1_et1_y2, a1_et1.t,
                                                            a1_et2_x1, a1_et2_y1, a1_et2_x2, a1_et2_y2, a1_et2.t,
                                                            a1_nt_x1, a1_nt_y1, a1_nt.t,
                                                            a1,
                                                            a2_et1_x1, a2_et1_y1, a2_et1_x2, a2_et1_y2, a2_et1.t,
                                                            a2_et2_x1, a2_et2_y1, a2_et2_x2, a2_et2_y2, a2_et2.t,
                                                            a2,
                                                            lhs,
                                                            SCIPnodeGetNumber(SCIPgetCurrentNode(scip)));
                                                }
#endif

                                            // Create cut.
                                            SCIP_CALL(vertexfouredge_conflicts_create_cut(scip,
                                                                                          probdata,
                                                                                          sepa,
                                                                                          a1,
                                                                                          a2,
                                                                                          a1_et1,
                                                                                          a1_et2,
                                                                                          a1_nts,
                                                                                          a2_et1,
                                                                                          a2_et2,
                                                                                          result));
                                            found_cuts = true;
                                            goto NEXT_AGENT;
                                        }
                                    }
                                }
                        }
                    }
                    // Find cut of type B.
                    else if (a1_et2.et.e == a1_et1.et.e)
                    {
                        // Get the second edge of agent 2.
                        const EdgeTime a2_et2{a2_et1.n, Direction::WAIT, a2_et1.t + 1};

                        // Found the four edges. Look for an agent 2 that uses these edges.
                        for (Agent a2 = 0; a2 < N; ++a2)
                            if (a1 != a2)
                            {
                                // Get the value of the edges of agent 2.
                                const auto& fractional_move_edges_a2 = fractional_move_edges[a2];
                                const auto& fractional_edges_a2 = fractional_edges[a2];
                                const auto a2_et1_it = fractional_move_edges_a2.find(a2_et1);
                                const auto a2_et2_it = fractional_edges_a2.find(a2_et2);
                                if (a2_et1_it == fractional_move_edges_a2.end() || a2_et2_it == fractional_edges_a2.end())
                                {
                                    continue;
                                }
                                const auto a2_et1_val = a2_et1_it->second;
                                const auto a2_et2_val = a2_et2_it->second;

                                // Get a vertex of agent 1.
                                for (Time t = 1; t <= a1_et1.t; ++t)
                                {
                                    // Get all vertices at time t.
                                    Vector<NodeTime> a1_nts;
                                    Float a1_nts_val = 0;
                                    for (const auto& [a1_nt, a1_nt_val] : fractional_vertices_a1)
                                        if (a1_nt.t == t)
                                        {
                                            a1_nts.push_back(a1_nt);
                                            a1_nts_val += a1_nt_val;
                                        }

                                    // Create the constraint if violated.
                                    const auto lhs = a1_et1_val + a1_et2_val + a2_et1_val + a2_et2_val + a1_nts_val;
                                    if (a1_nts_val > 0 && SCIPisSumGT(scip, lhs, 2.0 + CUT_VIOLATION))
                                    {
                                        // Print.
#ifdef PRINT_DEBUG
                                        {
                                            const auto [a1_et1_x1, a1_et1_y1] = map.get_xy(a1_et1.n);
                                            const auto [a1_et1_x2, a1_et1_y2] = map.get_destination_xy(a1_et1);

                                            const auto [a1_et2_x1, a1_et2_y1] = map.get_xy(a1_et2.n);
                                            const auto [a1_et2_x2, a1_et2_y2] = map.get_destination_xy(a1_et2);

                                            const auto a1_nt = a1_nts.front();
                                            const auto [a1_nt_x1, a1_nt_y1] = map.get_xy(a1_nt.n);

                                            const auto [a2_et1_x1, a2_et1_y1] = map.get_xy(a2_et1.n);
                                            const auto [a2_et1_x2, a2_et1_y2] = map.get_destination_xy(a2_et1);

                                            const auto [a2_et2_x1, a2_et2_y1] = map.get_xy(a2_et2.n);
                                            const auto [a2_et2_x2, a2_et2_y2] = map.get_destination_xy(a2_et2);

                                            debugln("   Creating vertex four-edge conflict cut (type B) on "
                                                    "(({},{}),{},({},{})), (({},{}),{},({},{})) and (({},{}),{}) for agent {} "
                                                    "and edges "
                                                    "(({},{}),{},({},{})) and (({},{}),{},({},{})) for agent {} "
                                                    "with value {} in branch-and-bound node {}",
                                                    a1_et1_x1, a1_et1_y1, a1_et1.t, a1_et1_x2, a1_et1_y2,
                                                    a1_et2_x1, a1_et2_y1, a1_et2.t, a1_et2_x2, a1_et2_y2,
                                                    a1_nt_x1, a1_nt_y1, a1_nt.t,
                                                    a1,
                                                    a2_et1_x1, a2_et1_y1, a2_et1.t, a2_et1_x2, a2_et1_y2,
                                                    a2_et2_x1, a2_et2_y1, a2_et2.t, a2_et2_x2, a2_et2_y2,
                                                    a2,
                                                    lhs,
                                                    SCIPnodeGetNumber(SCIPgetCurrentNode(scip)));
                                        }
#endif

                                        // Create cut.
                                        SCIP_CALL(vertexfouredge_conflicts_create_cut(scip,
                                                                                          probdata,
                                                                                          sepa,
                                                                                          a1,
                                                                                          a2,
                                                                                          a1_et1,
                                                                                          a1_et2,
                                                                                          a1_nts,
                                                                                          a2_et1,
                                                                                          a2_et2,
                                                                                          result));
                                        found_cuts = true;
                                        goto NEXT_AGENT;
                                    }
                                }
                            }
                    }
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
SCIP_DECL_SEPACOPY(sepaCopyVertexFourEdgeConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);

    // Include separator.
    SCIP_CALL(SCIPincludeSepaVertexFourEdgeConflicts(scip));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Separation method for LP solutions
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_SEPAEXECLP(sepaExeclpVertexFourEdgeConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);
    debug_assert(result);

    // Start.
    *result = SCIP_DIDNOTFIND;

    // Start separator.
    SCIP_CALL(vertexfouredge_conflicts_separate(scip, sepa, result));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Create separator for vertex four-edge conflicts constraints and include it in SCIP
SCIP_RETCODE SCIPincludeSepaVertexFourEdgeConflicts(
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
                                   sepaExeclpVertexFourEdgeConflicts,
                                   nullptr,
                                   nullptr));
    debug_assert(sepa);

    // Set callbacks.
    SCIP_CALL(SCIPsetSepaCopy(scip, sepa, sepaCopyVertexFourEdgeConflicts));

    // Done.
    return SCIP_OKAY;
}

#endif
