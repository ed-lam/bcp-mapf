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

#if defined(USE_CORRIDOR_CONFLICTS) || defined(USE_WAITCORRIDOR_CONFLICTS)

//#define PRINT_DEBUG

#include "Separator_CorridorConflicts.h"
#include "ProblemData.h"
#include "VariableData.h"

#ifdef USE_WAITCORRIDOR_CONFLICTS
#define SEPA_NAME                               "wait_corridor"
#else
#define SEPA_NAME                                    "corridor"
#endif
#define SEPA_DESC            "Separator for corridor conflicts"
#define SEPA_PRIORITY                                   +550000 // priority of the constraint handler for separation
#define SEPA_FREQ                                             1 // frequency for separating cuts; zero means to separate only in the root node
#define SEPA_MAXBOUNDDIST                                   1.0
#define SEPA_USESSUBSCIP                                  FALSE // does the separator use a secondary SCIP instance? */
#define SEPA_DELAY                                        FALSE // should separation method be delayed, if other separators found cuts? */

SCIP_RETCODE corridor_conflicts_create_cut(
    SCIP* scip,                 // SCIP
    SCIP_ProbData* probdata,    // Problem data
    SCIP_SEPA* sepa,            // Separator
    const Agent a1,             // Agent 1
    const Agent a2,             // Agent 2
    const EdgeTime a1_et1,      // Edge-time 1 of agent 1
    const EdgeTime a1_et2,      // Edge-time 2 of agent 1
#ifdef USE_WAITCORRIDOR_CONFLICTS
    const EdgeTime a1_et3,      // Edge-time 3 of agent 1
    const EdgeTime a1_et4,      // Edge-time 4 of agent 1
#endif
    const EdgeTime a2_et1,      // Edge-time 1 of agent 2
    const EdgeTime a2_et2,      // Edge-time 2 of agent 2
    SCIP_Result* result         // Output result
)
{
    // Create constraint name.
#ifdef DEBUG
    const auto& map = SCIPprobdataGetMap(probdata);
    const auto [a1_x1, a1_y1] = map.get_xy(a1_et1.n);
    const auto [a1_x2, a1_y2] = map.get_xy(a1_et2.n);
    const auto [a2_x1, a2_y1] = map.get_xy(a2_et1.n);
    const auto [a2_x2, a2_y2] = map.get_xy(a2_et2.n);
    auto name = fmt::format("corridor_conflict("
                            "{},(({},{}),({},{}),{}),"
                            "{},(({},{}),({},{}),{}))",
                            a1, a1_x1, a1_y1, a1_x2, a1_y2, a1_et1.t,
                            a2, a2_x1, a2_y1, a2_x2, a2_y2, a2_et1.t);
#endif

    // Create data for the cut.
    TwoAgentRobustCut cut(scip,
                          a1,
                          a2,
#ifdef USE_WAITCORRIDOR_CONFLICTS
                          4,
#else
                          2,
#endif
                          2
#ifdef DEBUG
                        , std::move(name)
#endif
    );
    cut.edge_times_a1(0) = a1_et1;
    cut.edge_times_a1(1) = a1_et2;
#ifdef USE_WAITCORRIDOR_CONFLICTS
    cut.edge_times_a1(2) = a1_et3;
    cut.edge_times_a1(3) = a1_et4;
#endif
    cut.edge_times_a2(0) = a2_et1;
    cut.edge_times_a2(1) = a2_et2;

    // Store the cut.
    SCIP_CALL(SCIPprobdataAddTwoAgentRobustCut(scip, probdata, sepa, std::move(cut), 1, result));

    // Done.
    return SCIP_OKAY;
}

// Separator
static
SCIP_RETCODE corridor_conflicts_separate(
    SCIP* scip,            // SCIP
    SCIP_SEPA* sepa,       // Separator
    SCIP_RESULT* result    // Pointer to store the result of the separation call
)
{
    // Print.
    debugln("Starting separator for corridor conflicts on solution with obj {:.6f}:",
            SCIPgetSolOrigObj(scip, nullptr));

    // Print paths.
#ifdef PRINT_DEBUG
    print_used_paths(scip);
#endif

    // Get problem data.
    auto probdata = SCIPgetProbData(scip);
    const auto N = SCIPprobdataGetN(probdata);
    const auto& map = SCIPprobdataGetMap(probdata);

    // Create two-agent robust cuts for debugging purposes.
//    {
//        static int iter = 0;
//        ++iter;
//        if (iter == 1)
//        {
//            {
//                TwoAgentRobustCut cut(scip, 8, 10, 2, 2, "debug");
//                cut.edge_times_a1(0) = EdgeTime{map.get_id(61, 25), Direction::EAST, 24};
//                cut.edge_times_a1(1) = EdgeTime{map.get_id(62, 25), Direction::EAST, 25};
//                cut.edge_times_a2(0) = EdgeTime{map.get_id(62, 25), Direction::WEST, 24};
//                cut.edge_times_a2(1) = EdgeTime{map.get_id(61, 25), Direction::WEST, 25};
//                SCIP_CALL(SCIPprobdataAddTwoAgentRobustCut(scip, probdata, sepa, std::move(cut), 2, result));
//            }
//        }
//    }

    // Get the edges fractionally used by each agent.
    const auto& agent_edges = SCIPprobdataGetAgentFractionalEdgesNoWaits(probdata);
#ifdef USE_WAITCORRIDOR_CONFLICTS
    const auto& agent_edges_with_waits = SCIPprobdataGetAgentFractionalEdges(probdata);
#endif

    // Find conflicts.
    for (Agent a1 = 0; a1 < N - 1; ++a1)
    {
        // Get the edges of agent 1.
        const auto& agent_edges_a1 = agent_edges[a1];
#ifdef USE_WAITCORRIDOR_CONFLICTS
        const auto& agent_edges_with_waits_a1 = agent_edges_with_waits[a1];
#endif

        // Loop through the second agent.
        for (Agent a2 = a1 + 1; a2 < N; ++a2)
        {
            // Get the edges of agent 2.
            const auto& agent_edges_a2 = agent_edges[a2];
#ifdef USE_WAITCORRIDOR_CONFLICTS
            const auto& agent_edges_with_waits_a2 = agent_edges_with_waits[a2];
#endif

            // Loop through all edges of agent 1.
            for (const auto [a1_et1, a1_et1_val] : agent_edges_a1)
            {
                // See if agent 2 is using the reverse edge.
                const auto t = a1_et1.t;
                debug_assert(a1_et1.et.e.d != Direction::WAIT);
                const EdgeTime a2_et1{map.get_opposite_edge(a1_et1.et.e), t};
                const auto a2_et1_it = agent_edges_a2.find(a2_et1);
                if (a2_et1_it != agent_edges_a2.end())
                {
                    // Store the value.
                    const auto a2_et1_val = a2_et1_it->second;

                    // See if the same edge is used at either one timestep before or one timestep after.
                    const Array<Time, 2> offsets{-1, 1};
                    for (const auto offset : offsets)
                    {
                        const EdgeTime a1_et2{a1_et1.et.e, t + offset};
                        const EdgeTime a2_et2{a2_et1.et.e, t + offset};
                        const auto a1_et2_it = agent_edges_a1.find(a1_et2);
                        const auto a2_et2_it = agent_edges_a2.find(a2_et2);
                        if (a1_et2_it != agent_edges_a1.end() || a2_et2_it != agent_edges_a2.end())
                        {
                            // Store the value.
                            const auto a1_et2_val = a1_et2_it != agent_edges_a1.end() ?
                                                    a1_et2_it->second :
                                                    0.0;
                            const auto a2_et2_val = a2_et2_it != agent_edges_a2.end() ?
                                                    a2_et2_it->second :
                                                    0.0;

                            // Calculate the LHS.
                            const auto lhs = a1_et1_val + a1_et2_val + a2_et1_val + a2_et2_val;

#ifdef USE_WAITCORRIDOR_CONFLICTS
                            // Lift the constraint to include two more edges of agent 1.
                            const EdgeTime a1_et3{a2_et1.n,
                                                  Direction::WAIT,
                                                  offset == 1 ? t : t - 1};
                            const auto a1_et3_it = agent_edges_with_waits_a1.find(a1_et3);
                            const auto a1_et3_val = a1_et3_it != agent_edges_with_waits_a1.end() ?
                                                    a1_et3_it->second :
                                                    0.0;
                            const EdgeTime a1_et4{a1_et1.n,
                                                  Direction::WAIT,
                                                  offset == 1 ? t + 1 : t};
                            const auto a1_et4_it = agent_edges_with_waits_a1.find(a1_et4);
                            const auto a1_et4_val = a1_et4_it != agent_edges_with_waits_a1.end() ?
                                                    a1_et4_it->second :
                                                    0.0;
                            const auto lhs_lift_a1 = lhs + a1_et3_val + a1_et4_val;

                            // Lift the constraint to include two more edges of agent 2.
                            const EdgeTime a2_et3{a1_et1.n,
                                                  Direction::WAIT,
                                                  offset == 1 ? t : t - 1};
                            const auto a2_et3_it = agent_edges_with_waits_a2.find(a2_et3);
                            const auto a2_et3_val = a2_et3_it != agent_edges_with_waits_a2.end() ?
                                                    a2_et3_it->second :
                                                    0.0;
                            const EdgeTime a2_et4{a2_et1.n,
                                                  Direction::WAIT,
                                                  offset == 1 ? t + 1 : t};
                            const auto a2_et4_it = agent_edges_with_waits_a2.find(a2_et4);
                            const auto a2_et4_val = a2_et4_it != agent_edges_with_waits_a2.end() ?
                                                    a2_et4_it->second :
                                                    0.0;
                            const auto lhs_lift_a2 = lhs + a2_et3_val + a2_et4_val;

                            if (SCIPisGT(scip, lhs_lift_a1, 1.0) || SCIPisGT(scip, lhs_lift_a2, 1.0))
                            {
                                if (lhs_lift_a1 >= lhs_lift_a2)
                                {
                                    // Print.
#ifdef PRINT_DEBUG
                                    {
                                        const auto [a1_et1_x1, a1_et1_y1] = map.get_xy(a1_et1.n);
                                        const auto [a1_et1_x2, a1_et1_y2] = map.get_xy(map.get_destination(a1_et1.et.e));

                                        const auto [a1_et2_x1, a1_et2_y1] = map.get_xy(a1_et2.n);
                                        const auto [a1_et2_x2, a1_et2_y2] = map.get_xy(map.get_destination(a1_et2.et.e));

                                        const auto [a1_et3_x1, a1_et3_y1] = map.get_xy(a1_et3.n);
                                        const auto [a1_et3_x2, a1_et3_y2] = map.get_xy(map.get_destination(a1_et3.et.e));

                                        const auto [a1_et4_x1, a1_et4_y1] = map.get_xy(a1_et4.n);
                                        const auto [a1_et4_x2, a1_et4_y2] = map.get_xy(map.get_destination(a1_et4.et.e));

                                        const auto [a2_et1_x1, a2_et1_y1] = map.get_xy(a2_et1.n);
                                        const auto [a2_et1_x2, a2_et1_y2] = map.get_xy(map.get_destination(a2_et1.et.e));

                                        const auto [a2_et2_x1, a2_et2_y1] = map.get_xy(a2_et2.n);
                                        const auto [a2_et2_x2, a2_et2_y2] = map.get_xy(map.get_destination(a2_et2.et.e));

                                        debugln("   Creating corridor conflict cut on edges "
                                                "(({},{}),{},({},{})), (({},{}),{},({},{})), (({},{}),{},({},{})) and "
                                                "(({},{}),{},({},{})) for agent {} "
                                                "and edges "
                                                "(({},{}),{},({},{})) and (({},{}),{},({},{})) for agent {} "
                                                "with value {} in branch-and-bound node {}",
                                                a1_et1_x1, a1_et1_y1, a1_et1.t, a1_et1_x2, a1_et1_y2,
                                                a1_et2_x1, a1_et2_y1, a1_et2.t, a1_et2_x2, a1_et2_y2,
                                                a1_et3_x1, a1_et3_y1, a1_et3.t, a1_et3_x2, a1_et3_y2,
                                                a1_et4_x1, a1_et4_y1, a1_et4.t, a1_et4_x2, a1_et4_y2,
                                                a1,
                                                a2_et1_x1, a2_et1_y1, a2_et1.t, a2_et1_x2, a2_et1_y2,
                                                a2_et2_x1, a2_et2_y1, a2_et2.t, a2_et2_x2, a2_et2_y2,
                                                a2,
                                                lhs_lift_a1,
                                                SCIPnodeGetNumber(SCIPgetCurrentNode(scip)));
                                    }
#endif

                                    // Create cut.
                                    SCIP_CALL(corridor_conflicts_create_cut(scip,
                                                                            probdata,
                                                                            sepa,
                                                                            a1,
                                                                            a2,
                                                                            a1_et1,
                                                                            a1_et2,
                                                                            a1_et3,
                                                                            a1_et4,
                                                                            a2_et1,
                                                                            a2_et2,
                                                                            result));
                                    goto NEXT_AGENT;
                                }
                                else
                                {
                                    // Print.
#ifdef PRINT_DEBUG
                                    {
                                        const auto [a1_et1_x1, a1_et1_y1] = map.get_xy(a1_et1.n);
                                        const auto [a1_et1_x2, a1_et1_y2] = map.get_xy(map.get_destination(a1_et1.et.e));

                                        const auto [a1_et2_x1, a1_et2_y1] = map.get_xy(a1_et2.n);
                                        const auto [a1_et2_x2, a1_et2_y2] = map.get_xy(map.get_destination(a1_et2.et.e));

                                        const auto [a2_et1_x1, a2_et1_y1] = map.get_xy(a2_et1.n);
                                        const auto [a2_et1_x2, a2_et1_y2] = map.get_xy(map.get_destination(a2_et1.et.e));

                                        const auto [a2_et2_x1, a2_et2_y1] = map.get_xy(a2_et2.n);
                                        const auto [a2_et2_x2, a2_et2_y2] = map.get_xy(map.get_destination(a2_et2.et.e));

                                        const auto [a2_et3_x1, a2_et3_y1] = map.get_xy(a2_et3.n);
                                        const auto [a2_et3_x2, a2_et3_y2] = map.get_xy(map.get_destination(a2_et3.et.e));

                                        const auto [a2_et4_x1, a2_et4_y1] = map.get_xy(a2_et4.n);
                                        const auto [a2_et4_x2, a2_et4_y2] = map.get_xy(map.get_destination(a2_et4.et.e));

                                        debugln("   Creating corridor conflict cut on edges "
                                                "(({},{}),{},({},{})) and (({},{}),{},({},{})) for agent {} "
                                                "and edges "
                                                "(({},{}),{},({},{})), (({},{}),{},({},{})), (({},{}),{},({},{})) and "
                                                "(({},{}),{},({},{})) for agent {} "
                                                "with value {} in branch-and-bound node {}",
                                                a1_et1_x1, a1_et1_y1, a1_et1.t, a1_et1_x2, a1_et1_y2,
                                                a1_et2_x1, a1_et2_y1, a1_et2.t, a1_et2_x2, a1_et2_y2,
                                                a1,
                                                a2_et1_x1, a2_et1_y1, a2_et1.t, a2_et1_x2, a2_et1_y2,
                                                a2_et2_x1, a2_et2_y1, a2_et2.t, a2_et2_x2, a2_et2_y2,
                                                a2_et3_x1, a2_et3_y1, a2_et3.t, a2_et3_x2, a2_et3_y2,
                                                a2_et4_x1, a2_et4_y1, a2_et4.t, a2_et4_x2, a2_et4_y2,
                                                a2,
                                                lhs_lift_a2,
                                                SCIPnodeGetNumber(SCIPgetCurrentNode(scip)));
                                    }
#endif

                                    // Create cut.
                                    SCIP_CALL(corridor_conflicts_create_cut(scip,
                                                                            probdata,
                                                                            sepa,
                                                                            a2,
                                                                            a1,
                                                                            a2_et1,
                                                                            a2_et2,
                                                                            a2_et3,
                                                                            a2_et4,
                                                                            a1_et1,
                                                                            a1_et2,
                                                                            result));
                                    goto NEXT_AGENT;
                                }
                            }
#else
                            if (SCIPisGT(scip, lhs, 1.0))
                            {
                                // Print.
#ifdef PRINT_DEBUG
                                {
                                    const auto [a1_et1_x1, a1_et1_y1] = map.get_xy(a1_et1.n);
                                    const auto [a1_et1_x2, a1_et1_y2] = map.get_xy(map.get_destination(a1_et1.et.e));

                                    const auto [a1_et2_x1, a1_et2_y1] = map.get_xy(a1_et2.n);
                                    const auto [a1_et2_x2, a1_et2_y2] = map.get_xy(map.get_destination(a1_et2.et.e));

                                    const auto [a2_et1_x1, a2_et1_y1] = map.get_xy(a2_et1.n);
                                    const auto [a2_et1_x2, a2_et1_y2] = map.get_xy(map.get_destination(a2_et1.et.e));

                                    const auto [a2_et2_x1, a2_et2_y1] = map.get_xy(a2_et2.n);
                                    const auto [a2_et2_x2, a2_et2_y2] = map.get_xy(map.get_destination(a2_et2.et.e));

                                    debugln("   Creating corridor conflict cut on edges "
                                            "(({},{}),{},({},{})) and (({},{}),{},({},{})) for agent {} "
                                            "and edges "
                                            "(({},{}),{},({},{})) and (({},{}),{},({},{})) for agent {} "
                                            "with value {} in branch-and-bound node {}",
                                            a1_et1_x1, a1_et1_y1, a1_et1.t, a1_et1_x2, a1_et1_y2,
                                            a1_et2_x1, a1_et2_y1, a1_et2.t, a1_et2_x2, a1_et2_y2,
                                            a1,
                                            a2_et1_x1, a2_et1_y1, a2_et1.t, a2_et1_x2, a2_et1_y2,
                                            a2_et2_x1, a2_et2_y1, a2_et2.t, a2_et2_x2, a2_et2_y2,
                                            a2,
                                            lhs,
                                            SCIPnodeGetNumber(SCIPgetCurrentNode(scip)));
                                }
#endif

                                // Create cut.
                                SCIP_CALL(corridor_conflicts_create_cut(scip,
                                                                        probdata,
                                                                        sepa,
                                                                        a1,
                                                                        a2,
                                                                        a1_et1,
                                                                        a1_et2,
                                                                        a2_et1,
                                                                        a2_et2,
                                                                        result));
                                goto NEXT_AGENT;
                            }
#endif
                        }
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
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_SEPACOPY(sepaCopyCorridorConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);

    // Include separator.
    SCIP_CALL(SCIPincludeSepaCorridorConflicts(scip));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Separation method for LP solutions
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_SEPAEXECLP(sepaExeclpCorridorConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);
    debug_assert(result);

    // Start.
    *result = SCIP_DIDNOTFIND;

    // Start separator.
    SCIP_CALL(corridor_conflicts_separate(scip, sepa, result));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Create separator for corridor conflicts constraints and include it in SCIP
SCIP_RETCODE SCIPincludeSepaCorridorConflicts(
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
                                   sepaExeclpCorridorConflicts,
                                   nullptr,
                                   nullptr));
    debug_assert(sepa);

    // Set callbacks.
    SCIP_CALL(SCIPsetSepaCopy(scip, sepa, sepaCopyCorridorConflicts));

    // Done.
    return SCIP_OKAY;
}

#endif
