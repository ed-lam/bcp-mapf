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

#ifdef USE_AGENTWAITEDGE_CONFLICTS

//#define PRINT_DEBUG

#include "Separator_AgentWaitEdgeConflicts.h"
#include "ProblemData.h"
#include "VariableData.h"

#define SEPA_NAME                             "agent_wait_edge"
#define SEPA_DESC     "Separator for agent wait edge conflicts"
#define SEPA_PRIORITY                                   +550000 // priority of the constraint handler for separation
#define SEPA_FREQ                                             1 // frequency for separating cuts; zero means to separate only in the root node
#define SEPA_MAXBOUNDDIST                                   1.0
#define SEPA_USESSUBSCIP                                  FALSE // does the separator use a secondary SCIP instance? */
#define SEPA_DELAY                                        FALSE // should separation method be delayed, if other separators found cuts? */

SCIP_RETCODE agentwaitedge_conflicts_create_cut(
    SCIP* scip,                 // SCIP
    SCIP_ProbData* probdata,    // Problem data
    SCIP_SEPA* sepa,            // Separator
    const Agent a1,             // Agent 1
    const Agent a2,             // Agent 2
    const EdgeTime a1_et1,      // Edge 1 of agent 1
    const EdgeTime a1_et2,      // Edge 2 of agent 1
    const EdgeTime a2_et1,      // Edge 1 of agent 2
    const EdgeTime a2_et2,      // Edge 2 of agent 2
    const EdgeTime a2_et3,      // Edge 3 of agent 2
    const EdgeTime a2_et4,      // Edge 4 of agent 2
    const EdgeTime a2_et5,      // Edge 5 of agent 2
    const EdgeTime a2_et6,      // Edge 6 of agent 2
    SCIP_Result* result         // Output result
)
{
    // Create constraint name.
#ifdef DEBUG
    const auto& map = SCIPprobdataGetMap(probdata);

    const auto [a1_et1_x1, a1_et1_y1] = map.get_xy(a1_et1.n);
    const auto [a1_et1_x2, a1_et1_y2] = map.get_destination_xy(a1_et1.et.e);

    const auto [a1_et2_x1, a1_et2_y1] = map.get_xy(a1_et2.n);
    const auto [a1_et2_x2, a1_et2_y2] = map.get_destination_xy(a1_et2.et.e);

    const auto [a2_et1_x1, a2_et1_y1] = map.get_xy(a2_et1.n);
    const auto [a2_et1_x2, a2_et1_y2] = map.get_destination_xy(a2_et1.et.e);

    const auto [a2_et2_x1, a2_et2_y1] = map.get_xy(a2_et2.n);
    const auto [a2_et2_x2, a2_et2_y2] = map.get_destination_xy(a2_et2.et.e);

    const auto [a2_et3_x1, a2_et3_y1] = map.get_xy(a2_et3.n);
    const auto [a2_et3_x2, a2_et3_y2] = map.get_destination_xy(a2_et3.et.e);

    const auto [a2_et4_x1, a2_et4_y1] = map.get_xy(a2_et4.n);
    const auto [a2_et4_x2, a2_et4_y2] = map.get_destination_xy(a2_et4.et.e);

    const auto [a2_et5_x1, a2_et5_y1] = map.get_xy(a2_et5.n);
    const auto [a2_et5_x2, a2_et5_y2] = map.get_destination_xy(a2_et5.et.e);

    const auto [a2_et6_x1, a2_et6_y1] = map.get_xy(a2_et6.n);
    const auto [a2_et6_x2, a2_et6_y2] = map.get_destination_xy(a2_et6.et.e);

    auto name = fmt::format("agentwaitedge_conflict("
                            "{},{},"
                            "(({},{}),({},{}),{}),"
                            "(({},{}),({},{}),{}),"
                            "(({},{}),({},{}),{}),"
                            "(({},{}),({},{}),{}),"
                            "(({},{}),({},{}),{}),"
                            "(({},{}),({},{}),{}),"
                            "(({},{}),({},{}),{}),"
                            "(({},{}),({},{}),{})"
                            ")",
                            a1, a2,
                            a1_et1_x1, a1_et1_y1, a1_et1_x2, a1_et1_y2, a1_et1.t,
                            a1_et2_x1, a1_et2_y1, a1_et2_x2, a1_et2_y2, a1_et2.t,
                            a2_et1_x1, a2_et1_y1, a2_et1_x2, a2_et1_y2, a2_et1.t,
                            a2_et2_x1, a2_et2_y1, a2_et2_x2, a2_et2_y2, a2_et2.t,
                            a2_et3_x1, a2_et3_y1, a2_et3_x2, a2_et3_y2, a2_et3.t,
                            a2_et4_x1, a2_et4_y1, a2_et4_x2, a2_et4_y2, a2_et4.t,
                            a2_et5_x1, a2_et5_y1, a2_et5_x2, a2_et5_y2, a2_et5.t,
                            a2_et6_x1, a2_et6_y1, a2_et6_x2, a2_et6_y2, a2_et6.t);
#endif

    // Create data for the cut.
    TwoAgentRobustCut cut(scip, a1, a2, 2, 6
#ifdef DEBUG
        , std::move(name)
#endif
    );
    cut.edge_times_a1(0) = a1_et1;
    cut.edge_times_a1(1) = a1_et2;
    cut.edge_times_a2(0) = a2_et1;
    cut.edge_times_a2(1) = a2_et2;
    cut.edge_times_a2(2) = a2_et3;
    cut.edge_times_a2(3) = a2_et4;
    cut.edge_times_a2(4) = a2_et5;
    cut.edge_times_a2(5) = a2_et6;

    // Store the cut.
    SCIP_CALL(SCIPprobdataAddTwoAgentRobustCut(scip, probdata, sepa, std::move(cut), 1, result));

    // Done.
    return SCIP_OKAY;
}

Vector<Array<EdgeTime, 5>>
get_a2_et23456(
    const EdgeTime a1_et2,
    const EdgeTime a2_et1,
    const Map& map
)
{
    Vector<Array<EdgeTime, 5>> a2_et23456;

    const auto n = a1_et2.n;
    const auto t = a1_et2.t;
    if (n != a2_et1.n)
    {
        // One timestep before.
        if (t > 0)
        {
            a2_et23456.push_back(Array<EdgeTime, 5>{EdgeTime{map.get_south(n), Direction::NORTH, t - 1},
                                                    EdgeTime{map.get_north(n), Direction::SOUTH, t - 1},
                                                    EdgeTime{map.get_west(n), Direction::EAST, t - 1},
                                                    EdgeTime{map.get_east(n), Direction::WEST, t - 1},
                                                    EdgeTime{map.get_wait(n), Direction::WAIT, t - 1}});
        }

        // Same timestep.
        a2_et23456.push_back(Array<EdgeTime, 5>{EdgeTime{n, Direction::NORTH, t},
                                                EdgeTime{n, Direction::SOUTH, t},
                                                EdgeTime{n, Direction::EAST, t},
                                                EdgeTime{n, Direction::WEST, t},
                                                EdgeTime{n, Direction::WAIT, t}});
    }

    if (n != map.get_destination(a2_et1.et.e))
    {
        // Same timestep.
        a2_et23456.push_back(Array<EdgeTime, 5>{EdgeTime{map.get_south(n), Direction::NORTH, t},
                                                EdgeTime{map.get_north(n), Direction::SOUTH, t},
                                                EdgeTime{map.get_west(n), Direction::EAST, t},
                                                EdgeTime{map.get_east(n), Direction::WEST, t},
                                                EdgeTime{map.get_wait(n), Direction::WAIT, t}});

        // One timestep after.
        a2_et23456.push_back(Array<EdgeTime, 5>{EdgeTime{n, Direction::NORTH, t + 1},
                                                EdgeTime{n, Direction::SOUTH, t + 1},
                                                EdgeTime{n, Direction::EAST, t + 1},
                                                EdgeTime{n, Direction::WEST, t + 1},
                                                EdgeTime{n, Direction::WAIT, t + 1}});
    }

    return a2_et23456;
}

// Separator
static
SCIP_RETCODE agentwaitedge_conflicts_separate(
    SCIP* scip,            // SCIP
    SCIP_SEPA* sepa,       // Separator
    SCIP_RESULT* result    // Pointer to store the result of the separation call
)
{
    // Print.
    debugln("Starting separator for agent wait-edge conflicts on solution with obj {:.6f}:",
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
    const auto& agent_edges = SCIPprobdataGetAgentFractionalEdgesNoWaits(probdata);
    const auto& agent_edges_with_waits = SCIPprobdataGetAgentFractionalEdges(probdata);

    // Find conflicts.
    for (Agent a1 = 0; a1 < N; ++a1)
    {
        // Get the edges of agent 1.
        const auto& agent_edges_a1 = agent_edges[a1];
        const auto& agent_edges_with_waits_a1 = agent_edges_with_waits[a1];

        // Loop through the first edge of agent 1.
        for (const auto [a1_et1, a1_et1_val] : agent_edges_a1)
        {
            debug_assert(a1_et1.d != Direction::WAIT);

            // Get the first edge of agent 2.
            const EdgeTime a2_et1{map.get_opposite_edge(a1_et1.et.e), a1_et1.t};

            // Loop through the second agent.
            for (Agent a2 = 0; a2 < N; ++a2)
                if (a2 != a1)
                {
                    // Get the edges of agent 2.
                    const auto& agent_edges_a2 = agent_edges[a2];
                    const auto& agent_edges_with_waits_a2 = agent_edges_with_waits[a2];

                    // Get the values of the first edge of agent 2.
                    const auto a2_et1_it = agent_edges_a2.find(a2_et1);
                    if (a2_et1_it == agent_edges_a2.end())
                    {
                        continue;
                    }
                    const auto a2_et1_val = a2_et1_it->second;

                    // Loop through the second edge of agent 1.
                    const Array<EdgeTime, 2> a1_et2s{EdgeTime{a1_et1.n, Direction::WAIT, a1_et1.t},
                                                     EdgeTime{map.get_destination(a1_et1.et.e), Direction::WAIT, a1_et1.t}};
                    for (const auto a1_et2 : a1_et2s)
                    {
                        // Get the values of the second edge of agent 1.
                        const auto a1_et2_it = agent_edges_with_waits_a1.find(a1_et2);
                        if (a1_et2_it == agent_edges_with_waits_a1.end())
                        {
                            continue;
                        }
                        const auto a1_et2_val = a1_et2_it->second;

                        // Get the remaining edges of agent 2.
                        for (const auto [a2_et2, a2_et3, a2_et4, a2_et5, a2_et6] : get_a2_et23456(a1_et2, a2_et1, map))
                        {
                            // Get the values of the edges of agent 2.
                            const auto a2_et2_it = agent_edges_with_waits_a2.find(a2_et2);
                            const auto a2_et3_it = agent_edges_with_waits_a2.find(a2_et3);
                            const auto a2_et4_it = agent_edges_with_waits_a2.find(a2_et4);
                            const auto a2_et5_it = agent_edges_with_waits_a2.find(a2_et5);
                            const auto a2_et6_it = agent_edges_with_waits_a2.find(a2_et6);
                            const auto a2_et2_val = a2_et2_it != agent_edges_with_waits_a2.end() ? a2_et2_it->second : 0;
                            const auto a2_et3_val = a2_et3_it != agent_edges_with_waits_a2.end() ? a2_et3_it->second : 0;
                            const auto a2_et4_val = a2_et4_it != agent_edges_with_waits_a2.end() ? a2_et4_it->second : 0;
                            const auto a2_et5_val = a2_et5_it != agent_edges_with_waits_a2.end() ? a2_et5_it->second : 0;
                            const auto a2_et6_val = a2_et6_it != agent_edges_with_waits_a2.end() ? a2_et6_it->second : 0;

                            // Create the cut if violated.
                            const auto lhs = a1_et1_val + a1_et2_val +
                                a2_et1_val + a2_et2_val + a2_et3_val + a2_et4_val + a2_et5_val + a2_et6_val;
                            if (SCIPisSumGT(scip, lhs, 1.0 + CUT_VIOLATION))
                            {
                                // Print.
#ifdef PRINT_DEBUG
                                {
                                    const auto [a1_et1_x1, a1_et1_y1] = map.get_xy(a1_et1.n);
                                    const auto [a1_et1_x2, a1_et1_y2] = map.get_destination_xy(a1_et1.et.e);

                                    const auto [a1_et2_x1, a1_et2_y1] = map.get_xy(a1_et2.n);
                                    const auto [a1_et2_x2, a1_et2_y2] = map.get_destination_xy(a1_et2.et.e);

                                    const auto [a2_et1_x1, a2_et1_y1] = map.get_xy(a2_et1.n);
                                    const auto [a2_et1_x2, a2_et1_y2] = map.get_destination_xy(a2_et1.et.e);

                                    const auto [a2_et2_x1, a2_et2_y1] = map.get_xy(a2_et2.n);
                                    const auto [a2_et2_x2, a2_et2_y2] = map.get_destination_xy(a2_et2.et.e);

                                    const auto [a2_et3_x1, a2_et3_y1] = map.get_xy(a2_et3.n);
                                    const auto [a2_et3_x2, a2_et3_y2] = map.get_destination_xy(a2_et3.et.e);

                                    const auto [a2_et4_x1, a2_et4_y1] = map.get_xy(a2_et4.n);
                                    const auto [a2_et4_x2, a2_et4_y2] = map.get_destination_xy(a2_et4.et.e);

                                    const auto [a2_et5_x1, a2_et5_y1] = map.get_xy(a2_et5.n);
                                    const auto [a2_et5_x2, a2_et5_y2] = map.get_destination_xy(a2_et5.et.e);

                                    const auto [a2_et6_x1, a2_et6_y1] = map.get_xy(a2_et6.n);
                                    const auto [a2_et6_x2, a2_et6_y2] = map.get_destination_xy(a2_et6.et.e);

                                    debugln("   Creating agent wait-edge conflict cut on edges "
                                            "(({},{}),({},{}),{}) and (({},{}),({},{}),{}) for agent {} and "
                                            "(({},{}),({},{}),{}), (({},{}),({},{}),{}), "
                                            "(({},{}),({},{}),{}), (({},{}),({},{}),{}), "
                                            "(({},{}),({},{}),{}), (({},{}),({},{}),{}) for agent {} "
                                            "with value {} in "
                                            "branch-and-bound node {}",
                                            a1_et1_x1, a1_et1_y1, a1_et1_x2, a1_et1_y2, a1_et1.t,
                                            a1_et2_x1, a1_et2_y1, a1_et2_x2, a1_et2_y2, a1_et2.t,
                                            a1,
                                            a2_et1_x1, a2_et1_y1, a2_et1_x2, a2_et1_y2, a2_et1.t,
                                            a2_et2_x1, a2_et2_y1, a2_et2_x2, a2_et2_y2, a2_et2.t,
                                            a2_et3_x1, a2_et3_y1, a2_et3_x2, a2_et3_y2, a2_et3.t,
                                            a2_et4_x1, a2_et4_y1, a2_et4_x2, a2_et4_y2, a2_et4.t,
                                            a2_et5_x1, a2_et5_y1, a2_et5_x2, a2_et5_y2, a2_et5.t,
                                            a2_et6_x1, a2_et6_y1, a2_et6_x2, a2_et6_y2, a2_et6.t,
                                            a2,
                                            lhs,
                                            SCIPnodeGetNumber(SCIPgetCurrentNode(scip)));
                                }
#endif

                                // Create cut.
                                SCIP_CALL(agentwaitedge_conflicts_create_cut(scip,
                                                                             probdata,
                                                                             sepa,
                                                                             a1,
                                                                             a2,
                                                                             a1_et1,
                                                                             a1_et2,
                                                                             a2_et1,
                                                                             a2_et2,
                                                                             a2_et3,
                                                                             a2_et4,
                                                                             a2_et5,
                                                                             a2_et6,
                                                                             result));
                                goto NEXT_AGENT;
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
SCIP_DECL_SEPACOPY(sepaCopyAgentWaitEdgeConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);

    // Include separator.
    SCIP_CALL(SCIPincludeSepaAgentWaitEdgeConflicts(scip));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Separation method for LP solutions
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_SEPAEXECLP(sepaExeclpAgentWaitEdgeConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);
    debug_assert(result);

    // Start.
    *result = SCIP_DIDNOTFIND;

    // Start separator.
    SCIP_CALL(agentwaitedge_conflicts_separate(scip, sepa, result));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Create separator for agent wait-edge conflicts constraints and include it in SCIP
SCIP_RETCODE SCIPincludeSepaAgentWaitEdgeConflicts(
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
                                   sepaExeclpAgentWaitEdgeConflicts,
                                   nullptr,
                                   nullptr));
    debug_assert(sepa);

    // Set callbacks.
    SCIP_CALL(SCIPsetSepaCopy(scip, sepa, sepaCopyAgentWaitEdgeConflicts));

    // Done.
    return SCIP_OKAY;
}

#endif
