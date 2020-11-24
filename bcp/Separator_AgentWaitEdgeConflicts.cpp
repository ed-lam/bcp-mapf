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

#define SEPA_NAME                     "agentwaitedge_conflicts"
#define SEPA_DESC     "Separator for agent wait-edge conflicts"
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
    const EdgeTime a1_et3,      // Edge 3 of agent 1
    const EdgeTime a1_et4,      // Edge 4 of agent 1
    const EdgeTime a1_et5,      // Edge 5 of agent 1
    const EdgeTime a1_et6,      // Edge 6 of agent 1
    const EdgeTime a2_et1,      // Edge 1 of other agents
    const EdgeTime a2_et2,      // Edge 2 of other agents
    SCIP_Result* result         // Output result
)
{
    // Create constraint name.
#ifdef DEBUG
    const auto& map = SCIPprobdataGetMap(probdata);

    const auto [a1_et1_x1, a1_et1_y1] = map.get_xy(a1_et1.n);
    const auto [a1_et1_x2, a1_et1_y2] = map.get_destination_xy(a1_et1.et.e);

    const auto [a1_et2_x1, a1_et2_y1] = map.get_xy(a1_et2.n);

    const auto [a2_et1_x1, a2_et1_y1] = map.get_xy(a2_et1.n);
    const auto [a2_et1_x2, a2_et1_y2] = map.get_destination_xy(a2_et1.et.e);

    const auto [a2_et2_x1, a2_et2_y1] = map.get_xy(a2_et2.n);
    const auto [a2_et2_x2, a2_et2_y2] = map.get_destination_xy(a2_et2.et.e);

    auto name = fmt::format("agentwaitedge_conflict("
                            "{},{},"
                            "(({},{}),({},{}),{}),"
                            "(({},{}),{}),"
                            "(({},{}),({},{}),{}),"
                            "(({},{}),({},{}),{})"
                            ")",
                            a1, a2,
                            a1_et1_x1, a1_et1_y1, a1_et1_x2, a1_et1_y2, a1_et1.t,
                            a1_et2_x1, a1_et2_y1, a1_et2.t,
                            a2_et1_x1, a2_et1_y1, a2_et1_x2, a2_et1_y2, a2_et1.t,
                            a2_et2_x1, a2_et2_y1, a2_et2_x2, a2_et2_y2, a2_et2.t);
#endif

    // Create data for the cut.
    TwoAgentRobustCut cut(scip, a1, a2, 6, 2
#ifdef DEBUG
        , std::move(name)
#endif
    );
    cut.edge_times_a1(0) = a1_et1;
    cut.edge_times_a1(1) = a1_et2;
    cut.edge_times_a1(2) = a1_et3;
    cut.edge_times_a1(3) = a1_et4;
    cut.edge_times_a1(4) = a1_et5;
    cut.edge_times_a1(5) = a1_et6;
    cut.edge_times_a2(0) = a2_et1;
    cut.edge_times_a2(1) = a2_et2;

    // Store the cut.
    SCIP_CALL(SCIPprobdataAddTwoAgentRobustCut(scip, probdata, sepa, std::move(cut), 1, result));

    // Done.
    return SCIP_OKAY;
}

Array<Tuple<EdgeTime, EdgeTime, EdgeTime, EdgeTime, EdgeTime, Float, Float, Float, Float, Float>, 2>
get_remaining_agent_1_edges(
    const NodeTime nt,
    const HashTable<EdgeTime, SCIP_Real>& agent_edges_a1, 
    const Map& map
)
{
    // Get the first set of edges of agent 1.
    const EdgeTime a1_et2_1{nt.n, Direction::NORTH, nt.t};
    const EdgeTime a1_et3_1{nt.n, Direction::SOUTH, nt.t};
    const EdgeTime a1_et4_1{nt.n, Direction::EAST, nt.t};
    const EdgeTime a1_et5_1{nt.n, Direction::WEST, nt.t};
    const EdgeTime a1_et6_1{nt.n, Direction::WAIT, nt.t};
    const auto a1_et2_it_1 = agent_edges_a1.find(a1_et2_1);
    const auto a1_et3_it_1 = agent_edges_a1.find(a1_et3_1);
    const auto a1_et4_it_1 = agent_edges_a1.find(a1_et4_1);
    const auto a1_et5_it_1 = agent_edges_a1.find(a1_et5_1);
    const auto a1_et6_it_1 = agent_edges_a1.find(a1_et6_1);
    const auto a1_et2_val_1 = a1_et2_it_1 != agent_edges_a1.end() ? a1_et2_it_1->second : 0.0;
    const auto a1_et3_val_1 = a1_et3_it_1 != agent_edges_a1.end() ? a1_et3_it_1->second : 0.0;
    const auto a1_et4_val_1 = a1_et4_it_1 != agent_edges_a1.end() ? a1_et4_it_1->second : 0.0;
    const auto a1_et5_val_1 = a1_et5_it_1 != agent_edges_a1.end() ? a1_et5_it_1->second : 0.0;
    const auto a1_et6_val_1 = a1_et6_it_1 != agent_edges_a1.end() ? a1_et6_it_1->second : 0.0;

    // Get the second set of edges of agent 1.
    debug_assert(nt.t >= 1);
    const auto prev_time = nt.t - 1;
    const EdgeTime a1_et2_2{map.get_south(nt.n), Direction::NORTH, prev_time};
    const EdgeTime a1_et3_2{map.get_north(nt.n), Direction::SOUTH, prev_time};
    const EdgeTime a1_et4_2{map.get_west(nt.n), Direction::EAST, prev_time};
    const EdgeTime a1_et5_2{map.get_east(nt.n), Direction::WEST, prev_time};
    const EdgeTime a1_et6_2{map.get_wait(nt.n), Direction::WAIT, prev_time};
    const auto a1_et2_it_2 = agent_edges_a1.find(a1_et2_2);
    const auto a1_et3_it_2 = agent_edges_a1.find(a1_et3_2);
    const auto a1_et4_it_2 = agent_edges_a1.find(a1_et4_2);
    const auto a1_et5_it_2 = agent_edges_a1.find(a1_et5_2);
    const auto a1_et6_it_2 = agent_edges_a1.find(a1_et6_2);
    const auto a1_et2_val_2 = a1_et2_it_2 != agent_edges_a1.end() ? a1_et2_it_2->second : 0.0;
    const auto a1_et3_val_2 = a1_et3_it_2 != agent_edges_a1.end() ? a1_et3_it_2->second : 0.0;
    const auto a1_et4_val_2 = a1_et4_it_2 != agent_edges_a1.end() ? a1_et4_it_2->second : 0.0;
    const auto a1_et5_val_2 = a1_et5_it_2 != agent_edges_a1.end() ? a1_et5_it_2->second : 0.0;
    const auto a1_et6_val_2 = a1_et6_it_2 != agent_edges_a1.end() ? a1_et6_it_2->second : 0.0;

    return {
        Tuple<EdgeTime, EdgeTime, EdgeTime, EdgeTime, EdgeTime, Float, Float, Float, Float, Float>{
            a1_et2_1,
            a1_et3_1,
            a1_et4_1,
            a1_et5_1,
            a1_et6_1,
            a1_et2_val_1,
            a1_et3_val_1,
            a1_et4_val_1,
            a1_et5_val_1,
            a1_et6_val_1},
        Tuple<EdgeTime, EdgeTime, EdgeTime, EdgeTime, EdgeTime, Float, Float, Float, Float, Float>{
            a1_et2_2,
            a1_et3_2,
            a1_et4_2,
            a1_et5_2,
            a1_et6_2,
            a1_et2_val_2,
            a1_et3_val_2,
            a1_et4_val_2,
            a1_et5_val_2,
            a1_et6_val_2}
    };
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
    const auto& agent_edges = SCIPprobdataGetAgentFractionalEdges(probdata);

    // Find conflicts.
    for (Agent a1 = 0; a1 < N; ++a1)
    {
        // Get the edges of agent 1.
        const auto& agent_edges_a1 = agent_edges[a1];

        // Loop through the first edge of agent 1.
        for (const auto [a1_et1, a1_et1_val] : agent_edges_a1)
            if (a1_et1.et.e.d != Direction::WAIT && a1_et1.t >= 1)
            {
                // Get the edges of agent 2.
                debug_assert(a1_et1.et.e.d != Direction::WAIT);
                const EdgeTime a2_et1{map.get_opposite_edge(a1_et1.et.e), a1_et1.t};
                const EdgeTime a2_et2{a2_et1.n, Direction::WAIT, a2_et1.t};

                // Loop  through the remaining edges of agent 1.
                for (const auto [a1_et2, a1_et3, a1_et4, a1_et5, a1_et6, 
                                 a1_et2_val, a1_et3_val, a1_et4_val, a1_et5_val, a1_et6_val] :
                    get_remaining_agent_1_edges(a2_et1.nt(), agent_edges_a1, map))
                {
                    // Loop through the second agent.
                    for (Agent a2 = 0; a2 < N; ++a2)
                        if (a2 != a1)
                        {
                            // Get the edges of agent 2.
                            const auto& agent_edges_a2 = agent_edges[a2];

                            // Get the values of the edges of agent 2.
                            const auto a2_et1_it = agent_edges_a2.find(a2_et1);
                            const auto a2_et2_it = agent_edges_a2.find(a2_et2);
                            const auto a2_et1_val = a2_et1_it != agent_edges_a2.end() ? a2_et1_it->second : 0.0;
                            const auto a2_et2_val = a2_et2_it != agent_edges_a2.end() ? a2_et2_it->second : 0.0;

                            // Create the cut if violated.
                            const auto lhs = a1_et1_val + a1_et2_val + a1_et3_val + a1_et4_val + a1_et5_val + 
                                             a1_et6_val + a2_et1_val + a2_et2_val;
                            if (SCIPisGT(scip, lhs, 1.0))
                            {
                                // Print.
#ifdef PRINT_DEBUG
                                {
                                    const auto [a1_et1_x1, a1_et1_y1] = map.get_xy(a1_et1.n);
                                    const auto [a1_et1_x2, a1_et1_y2] = map.get_destination_xy(a1_et1.et.e);

                                    const auto [a2_et1_x1, a2_et1_y1] = map.get_xy(a2_et1.n);
                                    const auto [a2_et1_x2, a2_et1_y2] = map.get_destination_xy(a2_et1.et.e);

                                    const auto [a2_et2_x1, a2_et2_y1] = map.get_xy(a2_et2.n);
                                    const auto [a2_et2_x2, a2_et2_y2] = map.get_destination_xy(a2_et2.et.e);

                                    debugln("   Creating agent wait-edge conflict cut on edges "
                                            "(({},{}),({},{}),{}) and (({},{}),{}) for agent {} and "
                                            "(({},{}),({},{}),{}) and (({},{}),({},{}),{}) for agent {} "
                                            "with value {} in "
                                            "branch-and-bound node {}",
                                            a1_et1_x1, a1_et1_y1, a1_et1_x2, a1_et1_y2, a1_et1.t,
                                            a2_et1_x1, a2_et1_y1, a1_et2.t,
                                            a1,
                                            a2_et1_x1, a2_et1_y1, a2_et1_x2, a2_et1_y2, a2_et1.t,
                                            a2_et2_x1, a2_et2_y1, a2_et2_x2, a2_et2_y2, a2_et2.t,
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
                                                                             a1_et3,
                                                                             a1_et4,
                                                                             a1_et5,
                                                                             a1_et6,
                                                                             a2_et1,
                                                                             a2_et2,
                                                                             result));
                            }
                        }
                }
            }
    }

    // Done.
    return SCIP_OKAY;
}

// Copy method for separator
static
SCIP_DECL_SEPACOPY(sepaCopyAgentWaitEdgeConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);

    // Include separator.
    SCIP_SEPA* sepa_copy;
    SCIP_CALL(SCIPincludeSepaAgentWaitEdgeConflicts(scip, &sepa_copy));

    // Done.
    return SCIP_OKAY;
}

// Separation method for LP solutions
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

// Create separator for agent wait-edge conflicts constraints and include it in SCIP
SCIP_RETCODE SCIPincludeSepaAgentWaitEdgeConflicts(
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
                                   sepaExeclpAgentWaitEdgeConflicts,
                                   nullptr,
                                   nullptr));
    debug_assert(*sepa);

    // Set callbacks.
    SCIP_CALL(SCIPsetSepaCopy(scip, *sepa, sepaCopyAgentWaitEdgeConflicts));

    // Done.
    return SCIP_OKAY;
}

#endif
