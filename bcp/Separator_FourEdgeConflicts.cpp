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

#ifdef USE_FOUREDGE_CONFLICTS

//#define PRINT_DEBUG

#include "Separator_FourEdgeConflicts.h"
#include "ProblemData.h"
#include "VariableData.h"

#define SEPA_NAME                                   "four_edge"
#define SEPA_DESC           "Separator for four edge conflicts"
#define SEPA_PRIORITY                                      1000 // priority of the constraint handler for separation
#define SEPA_FREQ                                             1 // frequency for separating cuts; zero means to separate only in the root node
#define SEPA_MAXBOUNDDIST                                   1.0
#define SEPA_USESSUBSCIP                                  FALSE // does the separator use a secondary SCIP instance? */
#define SEPA_DELAY                                        FALSE // should separation method be delayed, if other separators found cuts? */

SCIP_RETCODE fouredge_conflicts_create_cut(
    SCIP* scip,                 // SCIP
    SCIP_ProbData* probdata,    // Problem data
    SCIP_SEPA* sepa,            // Separator
    const Agent a1,             // Agent 1
    const Agent a2,             // Agent 2
    const EdgeTime a1_et1,      // Edge 1 of agent 1
    const EdgeTime a1_et2,      // Edge 2 of agent 1
    const EdgeTime a2_et1,      // Edge 1 of agent 2
    const EdgeTime a2_et2,      // Edge 2 of agent 2
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

    auto name = fmt::format("fouredge_conflict("
                            "{},{},"
                            "(({},{}),({},{}),{}),"
                            "(({},{}),({},{}),{}),"
                            "(({},{}),({},{}),{}),"
                            "(({},{}),({},{}),{})"
                            ")",
                            a1, a2,
                            a1_et1_x1, a1_et1_y1, a1_et1_x2, a1_et1_y2, a1_et1.t,
                            a1_et2_x1, a1_et2_y1, a1_et2_x2, a1_et2_y2, a1_et2.t,
                            a2_et1_x1, a2_et1_y1, a2_et1_x2, a2_et1_y2, a2_et1.t,
                            a2_et2_x1, a2_et2_y1, a2_et2_x2, a2_et2_y2, a2_et2.t);
#endif

    // Create data for the cut.
    TwoAgentRobustCut cut(scip, a1, a2, 2, 2
#ifdef DEBUG
        , std::move(name)
#endif
    );
    cut.edge_times_a1(0) = a1_et1;
    cut.edge_times_a1(1) = a1_et2;
    cut.edge_times_a2(0) = a2_et1;
    cut.edge_times_a2(1) = a2_et2;

    // Store the cut.
    SCIP_CALL(SCIPprobdataAddTwoAgentRobustCut(scip, probdata, sepa, std::move(cut), 1, result));

    // Done.
    return SCIP_OKAY;
}

// Separator
static
SCIP_RETCODE fouredge_conflicts_separate(
    SCIP* scip,            // SCIP
    SCIP_SEPA* sepa,       // Separator
    SCIP_RESULT* result    // Pointer to store the result of the separation call
)
{
    // Print.
    debugln("Starting separator for four-edge conflicts on solution with obj {:.6f}:",
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

//
//    Found incompatible subset (
//        (46, ((218, 167), (219, 167)), 109)
//                    (46, ((218, 166), (219, 166)), 110)
//
//       (170, ((218, 167), (218, 166)), 109)
//                   (170, ((219, 167), (219, 166)), 110)
//    ) with LHS 1.3332 RHS 1


    // Find conflicts.
    for (Agent a1 = 0; a1 < N; ++a1)
    {
        // Get the edges of agent 1.
        const auto& agent_edges_a1 = agent_edges[a1];

        // Loop through the first edge of agent 1.
        for (const auto [a1_et1, a1_et1_val] : agent_edges_a1)
        {
            // Get the destination of the edge.
            const auto a1_et1_dest = map.get_destination(a1_et1.et.e);

            // Loop through the second agent.
            for (Agent a2 = a1 + 1; a2 < N; ++a2)
            {
                // Get the edges of agent 2.
                const auto& agent_edges_a2 = agent_edges[a2];

                // Loop through the first edge of agent 2.
                for (const auto [a2_et1, a2_et1_val] : agent_edges_a2)
                    if (a1_et1.t == a2_et1.t && a1_et1.n == a2_et1.n)
                    {
                        // Get the destination of the edge.
                        const auto a2_et1_dest = map.get_destination(a2_et1.et.e);

                        // Loop through the second edge of agent 1.
                        for (const auto [a1_et2, a1_et2_val] : agent_edges_a1)
                            if (a1_et2.t == a1_et1.t + 1 && a1_et2.n == a2_et1_dest && a1_et2.n != a1_et1_dest)
                            {
                                // Get the destination of the edge.
                                const auto a1_et2_dest = map.get_destination(a1_et2.et.e);

                                // Loop through the second edge of agent 2.
                                for (const auto [a2_et2, a2_et2_val] : agent_edges_a2)
                                    if (a2_et2.t == a2_et1.t + 1 && a2_et2.n == a1_et1_dest)
                                    {
                                        // Get the destination of the edge.
                                        const auto a2_et2_dest = map.get_destination(a2_et2.et.e);

                                        // Ensure the destinations are the same.
                                        if (a2_et2_dest == a1_et2_dest)
                                        {
                                            // Create the cut if violated.
                                            const auto lhs = a1_et1_val + a1_et2_val + a2_et1_val + a2_et2_val;
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

                                                    debugln("   Creating four-edge conflict cut on edges "
                                                            "(({},{}),({},{}),{}) and (({},{}),({},{}),{}) for agent {} and "
                                                            "(({},{}),({},{}),{}) and (({},{}),({},{}),{}) for agent {} "
                                                            "with value {} in "
                                                            "branch-and-bound node {}",
                                                            a1_et1_x1, a1_et1_y1, a1_et1_x2, a1_et1_y2, a1_et1.t,
                                                            a1_et2_x1, a1_et2_y1, a1_et2_x2, a1_et2_y2, a1_et2.t,
                                                            a1,
                                                            a2_et1_x1, a2_et1_y1, a2_et1_x2, a2_et1_y2, a2_et1.t,
                                                            a2_et2_x1, a2_et2_y1, a2_et2_x2, a2_et2_y2, a2_et2.t,
                                                            a2,
                                                            lhs,
                                                            SCIPnodeGetNumber(SCIPgetCurrentNode(scip)));
                                                }
#endif

                                                // Create cut.
                                                SCIP_CALL(fouredge_conflicts_create_cut(scip,
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
SCIP_DECL_SEPACOPY(sepaCopyFourEdgeConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);

    // Include separator.
    SCIP_CALL(SCIPincludeSepaFourEdgeConflicts(scip));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Separation method for LP solutions
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_SEPAEXECLP(sepaExeclpFourEdgeConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);
    debug_assert(result);

    // Start.
    *result = SCIP_DIDNOTFIND;

    // Start separator.
    SCIP_CALL(fouredge_conflicts_separate(scip, sepa, result));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Create separator for four-edge conflicts constraints and include it in SCIP
SCIP_RETCODE SCIPincludeSepaFourEdgeConflicts(
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
                                   sepaExeclpFourEdgeConflicts,
                                   nullptr,
                                   nullptr));
    debug_assert(sepa);

    // Set callbacks.
    SCIP_CALL(SCIPsetSepaCopy(scip, sepa, sepaCopyFourEdgeConflicts));

    // Done.
    return SCIP_OKAY;
}

#endif
