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

#ifdef USE_WAITDELAY_CONFLICTS

// #define PRINT_DEBUG

#include "Separator_WaitDelayConflicts.h"
#include "ProblemData.h"
#include "VariableData.h"

#define SEPA_NAME         "wait_delay"
#define SEPA_DESC         "Separator for wait delay conflicts"
#define SEPA_PRIORITY     107      // priority of the constraint handler for separation
#define SEPA_FREQ         1        // frequency for separating cuts; zero means to separate only in the root node
#define SEPA_MAXBOUNDDIST 1.0
#define SEPA_USESSUBSCIP  FALSE    // does the separator use a secondary SCIP instance? */
#define SEPA_DELAY        FALSE    // should separation method be delayed, if other separators found cuts? */

struct WaitDelayConflictData
{
    SCIP_Real lhs;
    Agent a1;
    Agent a2;
    Array<EdgeTime, 9> a1_ets;
    EdgeTime a2_et;
#ifdef DEBUG
    NodeTime nt;
#endif
};

#define MATRIX(i,j) (i * N + j)

SCIP_RETCODE waitdelay_conflicts_create_cut(
    SCIP* scip,                          // SCIP
    SCIP_ProbData* probdata,             // Problem data
    SCIP_SEPA* sepa,                     // Separator
    const Agent a1,                      // Agent 1
    const Agent a2,                      // Agent 2
#ifdef DEBUG
    const NodeTime nt,                   // Node-time of the conflict
#endif
    const Array<EdgeTime, 9>& a1_ets,    // Edge-times of agent 1
    const EdgeTime a2_et,                // Edge-time of the wait by agent 2
    SCIP_Result* result                  // Output result
)
{
    // Create constraint name.
#ifdef DEBUG
    const auto& map = SCIPprobdataGetMap(probdata);
    const auto [x, y] = map.get_xy(nt.n);
    auto name = fmt::format("waitdelay_conflict({},{},({},{}),{})", a1, a2, x, y, nt.t);
#endif

    // Create data for the cut.
    TwoAgentRobustCut cut{scip, a1, a2, 9, 1
#ifdef DEBUG
        , std::move(name)
#endif
    };
    std::copy(a1_ets.begin(), a1_ets.end(), &cut.a1_edge_time(0));
    cut.a2_edge_time(0) = a2_et;

    // Store the cut.
    SCIP_CALL(SCIPprobdataAddTwoAgentRobustCut(scip, probdata, sepa, std::move(cut), 1, result));

    // Done.
    return SCIP_OKAY;
}

// Separator
static
SCIP_RETCODE waitdelay_conflicts_separate(
    SCIP* scip,            // SCIP
    SCIP_SEPA* sepa,       // Separator
    SCIP_RESULT* result    // Pointer to store the result of the separation call
)
{
    // Print.
    debugln("Starting separator for wait-delay conflicts on solution with obj {:.6f}:",
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
    const auto& fractional_edges = SCIPprobdataGetFractionalEdges(probdata);

    // Find conflicts.
    Vector<WaitDelayConflictData> cuts;
    for (Agent a1 = 0; a1 < N; ++a1)
    {
        // Get the edges of agent 1.
        const auto& fractional_edges_a1 = fractional_edges[a1];

        // Loop through the second agent.
        for (Agent a2 = 0; a2 < N; ++a2)
            if (a2 != a1)
            {
                // Get the edges of agent 2.
                const auto& fractional_edges_a2 = fractional_edges[a2];

                // Loop through all waits of agent 2.
                for (const auto& [a2_et, a2_et_val] : fractional_edges_a2)
                    if (a2_et.d == Direction::WAIT && a2_et.t > 0)
                    {
                        // Store the edges for a1 being at n at time t.
                        const auto nt = a2_et.nt();
                        Array<EdgeTime, 9> a1_ets;
                        a1_ets[0] = EdgeTime(map.get_south(nt.n), Direction::NORTH, nt.t - 1);
                        a1_ets[1] = EdgeTime(map.get_north(nt.n), Direction::SOUTH, nt.t - 1);
                        a1_ets[2] = EdgeTime(map.get_west(nt.n), Direction::EAST, nt.t - 1);
                        a1_ets[3] = EdgeTime(map.get_east(nt.n), Direction::WEST, nt.t - 1);
                        a1_ets[4] = EdgeTime(map.get_wait(nt.n), Direction::WAIT, nt.t - 1);

                        // Store the edges for a1 being at n at time t+1.
                        a1_ets[5] = EdgeTime(map.get_south(nt.n), Direction::NORTH, nt.t);
                        a1_ets[6] = EdgeTime(map.get_north(nt.n), Direction::SOUTH, nt.t);
                        a1_ets[7] = EdgeTime(map.get_west(nt.n), Direction::EAST, nt.t);
                        a1_ets[8] = EdgeTime(map.get_east(nt.n), Direction::WEST, nt.t);

                        // Calculate the LHS.
                        debug_assert(a2_et_val > 0);
                        SCIP_Real lhs = a2_et_val;
                        for (const auto et : a1_ets)
                        {
                            auto it = fractional_edges_a1.find(et);
                            if (it != fractional_edges_a1.end())
                            {
                                lhs += it->second;
                            }
                        }

                        // Store a cut if violated.
                        if (SCIPisSumGT(scip, lhs, 1.0 + CUT_VIOLATION))
                        {
                            cuts.emplace_back(WaitDelayConflictData{lhs,
                                                                    a1,
                                                                    a2,
                                                                    a1_ets,
                                                                    a2_et
#ifdef DEBUG
                                                                  , nt
#endif
                            });
                        }
                    }
            }
    }

    // Create the most violated cuts.
    Vector<Int> agent_nb_cuts(N * N);
    std::sort(cuts.begin(),
              cuts.end(),
              [](const WaitDelayConflictData& a, const WaitDelayConflictData& b) { return a.lhs > b.lhs; });
    for (const auto& cut : cuts)
    {
        const auto& [lhs,
                     a1,
                     a2,
                     a1_ets,
                     a2_et
#ifdef DEBUG
                   , nt
#endif
        ] = cut;
        auto& nb_cuts = agent_nb_cuts[MATRIX(std::min(a1, a2), std::max(a1, a2))];
        if (nb_cuts < 1)
        {
            // Print.
            debugln("   Creating wait-delay conflict cut at ({},{}) at time {} for agents {} and {} with value {} in "
                    "branch-and-bound node {}",
                    map.get_x(nt.n), map.get_y(nt.n), nt.t,
                    a1, a2,
                    lhs,
                    SCIPnodeGetNumber(SCIPgetCurrentNode(scip)));

            // Create cut.
            SCIP_CALL(waitdelay_conflicts_create_cut(scip,
                                                     probdata,
                                                     sepa,
                                                     a1,
                                                     a2,
#ifdef DEBUG
                                                     nt,
#endif
                                                     a1_ets,
                                                     a2_et,
                                                     result));
            ++nb_cuts;
            found_cuts = true;
        }
    }

    // Done.
    return SCIP_OKAY;
}

// Copy method for separator
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_SEPACOPY(sepaCopyWaitDelayConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);

    // Include separator.
    SCIP_CALL(SCIPincludeSepaWaitDelayConflicts(scip));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Separation method for LP solutions
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_SEPAEXECLP(sepaExeclpWaitDelayConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);
    debug_assert(result);

    // Start.
    *result = SCIP_DIDNOTFIND;

    // Start separator.
    SCIP_CALL(waitdelay_conflicts_separate(scip, sepa, result));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Create separator for wait-delay conflicts constraints and include it in SCIP
SCIP_RETCODE SCIPincludeSepaWaitDelayConflicts(
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
                                   sepaExeclpWaitDelayConflicts,
                                   nullptr,
                                   nullptr));
    debug_assert(sepa);

    // Set callbacks.
    SCIP_CALL(SCIPsetSepaCopy(scip, sepa, sepaCopyWaitDelayConflicts));

    // Done.
    return SCIP_OKAY;
}

#endif
