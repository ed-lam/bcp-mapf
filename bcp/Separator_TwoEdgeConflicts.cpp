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

#ifdef USE_TWOEDGE_CONFLICTS

//#define PRINT_DEBUG

#include "Separator_TwoEdgeConflicts.h"
#include "ProblemData.h"
#include "VariableData.h"

#define SEPA_NAME                           "twoedge_conflicts"
#define SEPA_DESC            "Separator for two-edge conflicts"
#define SEPA_PRIORITY                                   +550000 // priority of the constraint handler for separation
#define SEPA_FREQ                                             1 // frequency for separating cuts; zero means to separate only in the root node
#define SEPA_MAXBOUNDDIST                                   1.0
#define SEPA_USESSUBSCIP                                  FALSE // does the separator use a secondary SCIP instance? */
#define SEPA_DELAY                                        FALSE // should separation method be delayed, if other separators found cuts? */

SCIP_RETCODE twoedge_conflicts_create_cut(
    SCIP* scip,                 // SCIP
    SCIP_ProbData* probdata,    // Problem data
    SCIP_SEPA* sepa,            // Separator
    const Agent a1,             // Agent 1
    const Agent a2,             // Agent 2
    const Edge a1_e1,           // Edge 1 of agent 1
    const Edge a1_e2,           // Edge 2 of agent 1
    const Edge a2_e1,           // Edge 1 of agent 2
    const Edge a2_e2,           // Edge 2 of agent 2
    const Time t,               // Time
    SCIP_Result* result         // Output result
)
{
    // Create constraint name.
#ifdef DEBUG
    const auto& map = SCIPprobdataGetMap(probdata);
    const auto [x1, y1] = map.get_xy(a1_e1.n);
    const auto [x2, y2] = map.get_destination_xy(a1_e1);

    const auto [x3, y3] = map.get_xy(a1_e2.n);
    const auto [x4, y4] = map.get_destination_xy(a1_e2);

    auto name = fmt::format("twoedge_conflict("
                           "{},{},",
                           "(({},{}),({},{})),"
                           "(({},{}),({},{})),"
                           "{})",
                           a1, a2,
                           x1, y1, x2, y2,
                           x3, y3, x4, y4,
                           t);
#endif

    // Create data for the cut.
    TwoAgentRobustCut cut(scip, a1, a2, t, 2, 2
#ifdef DEBUG
        , std::move(name)
#endif
    );
    cut.edges_a1(0) = a1_e1;
    cut.edges_a1(1) = a1_e2;
    cut.edges_a2(0) = a2_e1;
    cut.edges_a2(1) = a2_e2;

    // Store the cut.
    SCIP_CALL(SCIPprobdataAddTwoAgentRobustCut(scip, probdata, sepa, std::move(cut), 1, result));

    // Done.
    return SCIP_OKAY;
}

// Separator
static
SCIP_RETCODE twoedge_conflicts_separate(
    SCIP* scip,            // SCIP
    SCIP_SEPA* sepa,       // Separator
    SCIP_RESULT* result    // Pointer to store the result of the separation call
)
{
    // Print.
    debugln("Starting separator for two-edge conflicts on solution with obj {:.6f}:",
            SCIPgetSolOrigObj(scip, nullptr));

    // Print paths.
#ifdef PRINT_DEBUG
    print_used_paths(scip);
#endif

    // Get problem data.
    auto probdata = SCIPgetProbData(scip);
    const auto N = SCIPprobdataGetN(probdata);
    const auto& map = SCIPprobdataGetMap(probdata);

    // Get the edges fractionally used by each agent.
    const auto& agent_edges = get_agent_fractional_edges_no_waits(scip);

    // Find conflicts.
    for (Agent a1 = 0; a1 < N - 1; ++a1)
    {
        // Get the edges of agent 1.
        const auto& agent_edges_a1 = agent_edges[a1];

        // Loop through the second agent.
        for (Agent a2 = a1 + 1; a2 < N; ++a2)
        {
            // Get the edges of agent 2.
            const auto& agent_edges_a2 = agent_edges[a2];

            // Loop through all edges of agent 1.
            for (const auto [a1_et1, a1_et1_val] : agent_edges_a1)
            {
                // Get the edge.
                const auto t = a1_et1.t;
                const auto a1_e1 = a1_et1.et.e;
                debug_assert(a1_e1.d != Direction::WAIT);

                // Loop through the second edge.
                Array<Edge, 4> a1_e2s{Edge(map.get_south(a1_e1.n), Direction::NORTH),
                                      Edge(map.get_north(a1_e1.n), Direction::SOUTH),
                                      Edge(map.get_west(a1_e1.n), Direction::EAST),
                                      Edge(map.get_east(a1_e1.n), Direction::WEST)};
                for (const auto a1_e2 : a1_e2s)
                    if (a1_e2 != a1_e1)
                    {
                        // Get the second edge of agent 1.
                        const auto a1_et2_it = agent_edges_a1.find(EdgeTime{a1_e2, t});
                        const auto a1_et2_val = a1_et2_it != agent_edges_a1.end() ?
                                                a1_et2_it->second :
                                                0.0;

                        // Get the first edge of agent 2.
                        debug_assert(a1_e1.d != Direction::WAIT);
                        const auto a2_e1 = map.get_opposite_edge(a1_e1);
                        const auto a2_et1_it = agent_edges_a2.find(EdgeTime{a2_e1, t});
                        const auto a2_et1_val = a2_et1_it != agent_edges_a2.end() ?
                                                a2_et1_it->second :
                                                0.0;

                        // Get the second edge of agent 2.
                        debug_assert(a1_e2.d != Direction::WAIT);
                        const auto a2_e2 = map.get_opposite_edge(a1_e2);
                        const auto a2_et2_it = agent_edges_a2.find(EdgeTime{a2_e2, t});
                        const auto a2_et2_val = a2_et2_it != agent_edges_a2.end() ?
                                                a2_et2_it->second :
                                                0.0;

                        // Determine if there is a conflict.
                        const auto lhs = a1_et1_val + a1_et2_val + a2_et1_val + a2_et2_val;
                        if (SCIPisGT(scip, lhs, 1.0))
                        {
                            // Print.
#ifdef PRINT_DEBUG
                            {
                                const auto [a1_e1_x1, a1_e1_y1] = map.get_xy(a1_e1.n);
                                const auto [a1_e1_x2, a1_e1_y2] = map.get_destination_xy(a1_e1);

                                const auto [a1_e2_x1, a1_e2_y1] = map.get_xy(a1_e2.n);
                                const auto [a1_e2_x2, a1_e2_y2] = map.get_destination_xy(a1_e2);

                                const auto [a2_e1_x1, a2_e1_y1] = map.get_xy(a2_e1.n);
                                const auto [a2_e1_x2, a2_e1_y2] = map.get_destination_xy(a2_e1);

                                const auto [a2_e2_x1, a2_e2_y1] = map.get_xy(a2_e2.n);
                                const auto [a2_e2_x2, a2_e2_y2] = map.get_destination_xy(a2_e2);

                                debugln("   Creating two-edge conflict cut on edges "
                                        "(({},{}),({},{})) and (({},{}),({},{})) for agent {} "
                                        "and "
                                        "(({},{}),({},{})) and (({},{}),({},{})) for agent {} "
                                        "at time {} "
                                        "with value {} in "
                                        "branch-and-bound node {}",
                                        a1_e1_x1, a1_e1_y1, a1_e1_x2, a1_e1_y2,
                                        a1_e2_x1, a1_e2_y1, a1_e2_x2, a1_e2_y2,
                                        a1,
                                        a2_e1_x1, a2_e1_y1, a2_e1_x2, a2_e1_y2,
                                        a2_e2_x1, a2_e2_y1, a2_e2_x2, a2_e2_y2,
                                        a2,
                                        t,
                                        lhs,
                                        SCIPnodeGetNumber(SCIPgetCurrentNode(scip)));
                            }
#endif

                            // Create cut.
                            SCIP_CALL(twoedge_conflicts_create_cut(scip,
                                                                   probdata,
                                                                   sepa,
                                                                   a1,
                                                                   a2,
                                                                   a1_e1,
                                                                   a1_e2,
                                                                   a2_e1,
                                                                   a2_e2,
                                                                   t,
                                                                   result));
                            goto NEXT_AGENT;
                        }
                    }
            }
            NEXT_AGENT:;
        }
    }

    // Done.
    return SCIP_OKAY;
}

// Copy method for separator
static
SCIP_DECL_SEPACOPY(sepaCopyTwoEdgeConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);

    // Include separator.
    SCIP_SEPA* sepa_copy;
    SCIP_CALL(SCIPincludeSepaTwoEdgeConflicts(scip, &sepa_copy));

    // Done.
    return SCIP_OKAY;
}

// Separation method for LP solutions
static
SCIP_DECL_SEPAEXECLP(sepaExeclpTwoEdgeConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);
    debug_assert(result);

    // Start.
    *result = SCIP_DIDNOTFIND;

    // Start separator.
    SCIP_CALL(twoedge_conflicts_separate(scip, sepa, result));

    // Done.
    return SCIP_OKAY;
}

// Create separator for two-edge conflicts constraints and include it in SCIP
SCIP_RETCODE SCIPincludeSepaTwoEdgeConflicts(
    SCIP* scip,         // SCIP
    SCIP_SEPA** sepa    // Output pointer to separator
)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);

    // Include separator.
    *sepa = nullptr;
    SCIP_CALL(SCIPincludeSepaBasic(scip,
                                   sepa,
                                   SEPA_NAME,
                                   SEPA_DESC,
                                   SEPA_PRIORITY,
                                   SEPA_FREQ,
                                   SEPA_MAXBOUNDDIST,
                                   SEPA_USESSUBSCIP,
                                   SEPA_DELAY,
                                   sepaExeclpTwoEdgeConflicts,
                                   nullptr,
                                   nullptr));
    debug_assert(*sepa);

    // Set callbacks.
    SCIP_CALL(SCIPsetSepaCopy(scip, *sepa, sepaCopyTwoEdgeConflicts));

    // Done.
    return SCIP_OKAY;
}

#endif
