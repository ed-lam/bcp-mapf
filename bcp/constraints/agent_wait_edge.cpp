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

// #define PRINT_DEBUG

#include "constraints/agent_wait_edge.h"
#include "problem/problem.h"
#include "problem/variable_data.h"

#define SEPA_NAME         "agent_wait_edge"
#define SEPA_DESC         "Separator for agent wait edge conflicts"
#define SEPA_PRIORITY     100      // priority of the constraint handler for separation
#define SEPA_FREQ         1        // frequency for separating cuts; zero means to separate only in the root node
#define SEPA_MAXBOUNDDIST 1.0
#define SEPA_USESSUBSCIP  FALSE    // does the separator use a secondary SCIP instance? */
#define SEPA_DELAY        FALSE    // should separation method be delayed, if other separators found cuts? */

struct AgentWaitEdgeConflictCandidate
{
    EdgeTime a1_et2;
    SCIP_Real* a1_et2_vals;
    Array<EdgeTime, 5> a2_et23456s;
    Array<SCIP_Real*, 5> a2_et23456_vals;
};

struct AgentWaitEdgeConflictData
{
    SCIP_Real lhs;
    Agent a1;
    Agent a2;
    EdgeTime a1_et1;
    EdgeTime a1_et2;
    EdgeTime a2_et1;
    Array<EdgeTime, 5> a2_et23456s;
};

#define MATRIX(i,j) (i * N + j)

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
    const auto [a1_et1_x2, a1_et1_y2] = map.get_destination_xy(a1_et1);

    const auto [a1_et2_x1, a1_et2_y1] = map.get_xy(a1_et2.n);
    const auto [a1_et2_x2, a1_et2_y2] = map.get_destination_xy(a1_et2);

    const auto [a2_et1_x1, a2_et1_y1] = map.get_xy(a2_et1.n);
    const auto [a2_et1_x2, a2_et1_y2] = map.get_destination_xy(a2_et1);

    const auto [a2_et2_x1, a2_et2_y1] = map.get_xy(a2_et2.n);
    const auto [a2_et2_x2, a2_et2_y2] = map.get_destination_xy(a2_et2);

    const auto [a2_et3_x1, a2_et3_y1] = map.get_xy(a2_et3.n);
    const auto [a2_et3_x2, a2_et3_y2] = map.get_destination_xy(a2_et3);

    const auto [a2_et4_x1, a2_et4_y1] = map.get_xy(a2_et4.n);
    const auto [a2_et4_x2, a2_et4_y2] = map.get_destination_xy(a2_et4);

    const auto [a2_et5_x1, a2_et5_y1] = map.get_xy(a2_et5.n);
    const auto [a2_et5_x2, a2_et5_y2] = map.get_destination_xy(a2_et5);

    const auto [a2_et6_x1, a2_et6_y1] = map.get_xy(a2_et6.n);
    const auto [a2_et6_x2, a2_et6_y2] = map.get_destination_xy(a2_et6);

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
    cut.a1_edge_time(0) = a1_et1;
    cut.a1_edge_time(1) = a1_et2;
    cut.a2_edge_time(0) = a2_et1;
    cut.a2_edge_time(1) = a2_et2;
    cut.a2_edge_time(2) = a2_et3;
    cut.a2_edge_time(3) = a2_et4;
    cut.a2_edge_time(4) = a2_et5;
    cut.a2_edge_time(5) = a2_et6;

    // Store the cut.
    SCIP_CALL(SCIPprobdataAddTwoAgentRobustCut(scip, probdata, sepa, std::move(cut), 1, result));

    // Done.
    return SCIP_OKAY;
}

Vector<AgentWaitEdgeConflictCandidate> get_candidates(
    const EdgeTime a1_et1,
    const EdgeTime a2_et1,
    const HashTable<EdgeTime, SCIP_Real*>& fractional_edges_vec,
    const Map& map
)
{
    Vector<AgentWaitEdgeConflictCandidate> candidates;
    {
        const auto a1_et2 = EdgeTime{a1_et1.n, Direction::WAIT, a1_et1.t};
        auto a1_et2_vals_it = fractional_edges_vec.find(a1_et2);
        if (a1_et2_vals_it != fractional_edges_vec.end())
        {
            const auto a1_et2_vals = a1_et2_vals_it->second;

            const auto n = a1_et2.n;
            const auto t = a1_et2.t;
            if (n != a2_et1.n)
            {
                // One timestep before.
                if (t > 0)
                {
                    auto& candidate = candidates.emplace_back();
                    candidate.a1_et2 = a1_et2;
                    candidate.a1_et2_vals = a1_et2_vals;

                    candidate.a2_et23456s = {EdgeTime{map.get_south(n), Direction::NORTH, t - 1},
                                             EdgeTime{map.get_north(n), Direction::SOUTH, t - 1},
                                             EdgeTime{map.get_west(n), Direction::EAST, t - 1},
                                             EdgeTime{map.get_east(n), Direction::WEST, t - 1},
                                             EdgeTime{map.get_wait(n), Direction::WAIT, t - 1}};
                    for (Int idx = 0; idx < 5; ++idx)
                    {
                        auto it = fractional_edges_vec.find(candidate.a2_et23456s[idx]);
                        candidate.a2_et23456_vals[idx] = (it != fractional_edges_vec.end() ? it->second : nullptr);
                    }
                }

                // Same timestep.
                {
                    auto& candidate = candidates.emplace_back();
                    candidate.a1_et2 = a1_et2;
                    candidate.a1_et2_vals = a1_et2_vals;

                    candidate.a2_et23456s = {EdgeTime{n, Direction::NORTH, t},
                                             EdgeTime{n, Direction::SOUTH, t},
                                             EdgeTime{n, Direction::EAST, t},
                                             EdgeTime{n, Direction::WEST, t},
                                             EdgeTime{n, Direction::WAIT, t}};
                    for (Int idx = 0; idx < 5; ++idx)
                    {
                        auto it = fractional_edges_vec.find(candidate.a2_et23456s[idx]);
                        candidate.a2_et23456_vals[idx] = (it != fractional_edges_vec.end() ? it->second : nullptr);
                    }
                }
            }

            if (n != map.get_destination(a2_et1))
            {
                // Same timestep.
                {
                    auto& candidate = candidates.emplace_back();
                    candidate.a1_et2 = a1_et2;
                    candidate.a1_et2_vals = a1_et2_vals;

                    candidate.a2_et23456s = {EdgeTime{map.get_south(n), Direction::NORTH, t},
                                             EdgeTime{map.get_north(n), Direction::SOUTH, t},
                                             EdgeTime{map.get_west(n), Direction::EAST, t},
                                             EdgeTime{map.get_east(n), Direction::WEST, t},
                                             EdgeTime{map.get_wait(n), Direction::WAIT, t}};
                    for (Int idx = 0; idx < 5; ++idx)
                    {
                        auto it = fractional_edges_vec.find(candidate.a2_et23456s[idx]);
                        candidate.a2_et23456_vals[idx] = (it != fractional_edges_vec.end() ? it->second : nullptr);
                    }
                }

                // One timestep after.
                {
                    auto& candidate = candidates.emplace_back();
                    candidate.a1_et2 = a1_et2;
                    candidate.a1_et2_vals = a1_et2_vals;

                    candidate.a2_et23456s = {EdgeTime{n, Direction::NORTH, t + 1},
                                             EdgeTime{n, Direction::SOUTH, t + 1},
                                             EdgeTime{n, Direction::EAST, t + 1},
                                             EdgeTime{n, Direction::WEST, t + 1},
                                             EdgeTime{n, Direction::WAIT, t + 1}};
                    for (Int idx = 0; idx < 5; ++idx)
                    {
                        auto it = fractional_edges_vec.find(candidate.a2_et23456s[idx]);
                        candidate.a2_et23456_vals[idx] = (it != fractional_edges_vec.end() ? it->second : nullptr);
                    }
                }
            }
        }
    }
    {
        const auto a1_et2 = EdgeTime{map.get_destination(a1_et1), Direction::WAIT, a1_et1.t};
        auto a1_et2_vals_it = fractional_edges_vec.find(a1_et2);
        if (a1_et2_vals_it != fractional_edges_vec.end())
        {
            const auto a1_et2_vals = a1_et2_vals_it->second;

            const auto n = a1_et2.n;
            const auto t = a1_et2.t;
            if (n != a2_et1.n)
            {
                // One timestep before.
                if (t > 0)
                {
                    auto& candidate = candidates.emplace_back();
                    candidate.a1_et2 = a1_et2;
                    candidate.a1_et2_vals = a1_et2_vals;

                    candidate.a2_et23456s = {EdgeTime{map.get_south(n), Direction::NORTH, t - 1},
                                             EdgeTime{map.get_north(n), Direction::SOUTH, t - 1},
                                             EdgeTime{map.get_west(n), Direction::EAST, t - 1},
                                             EdgeTime{map.get_east(n), Direction::WEST, t - 1},
                                             EdgeTime{map.get_wait(n), Direction::WAIT, t - 1}};
                    for (Int idx = 0; idx < 5; ++idx)
                    {
                        auto it = fractional_edges_vec.find(candidate.a2_et23456s[idx]);
                        candidate.a2_et23456_vals[idx] = (it != fractional_edges_vec.end() ? it->second : nullptr);
                    }
                }

                // Same timestep.
                {
                    auto& candidate = candidates.emplace_back();
                    candidate.a1_et2 = a1_et2;
                    candidate.a1_et2_vals = a1_et2_vals;

                    candidate.a2_et23456s = {EdgeTime{n, Direction::NORTH, t},
                                             EdgeTime{n, Direction::SOUTH, t},
                                             EdgeTime{n, Direction::EAST, t},
                                             EdgeTime{n, Direction::WEST, t},
                                             EdgeTime{n, Direction::WAIT, t}};
                    for (Int idx = 0; idx < 5; ++idx)
                    {
                        auto it = fractional_edges_vec.find(candidate.a2_et23456s[idx]);
                        candidate.a2_et23456_vals[idx] = (it != fractional_edges_vec.end() ? it->second : nullptr);
                    }
                }
            }

            if (n != map.get_destination(a2_et1))
            {
                // Same timestep.
                {
                    auto& candidate = candidates.emplace_back();
                    candidate.a1_et2 = a1_et2;
                    candidate.a1_et2_vals = a1_et2_vals;

                    candidate.a2_et23456s = {EdgeTime{map.get_south(n), Direction::NORTH, t},
                                             EdgeTime{map.get_north(n), Direction::SOUTH, t},
                                             EdgeTime{map.get_west(n), Direction::EAST, t},
                                             EdgeTime{map.get_east(n), Direction::WEST, t},
                                             EdgeTime{map.get_wait(n), Direction::WAIT, t}};
                    for (Int idx = 0; idx < 5; ++idx)
                    {
                        auto it = fractional_edges_vec.find(candidate.a2_et23456s[idx]);
                        candidate.a2_et23456_vals[idx] = (it != fractional_edges_vec.end() ? it->second : nullptr);
                    }
                }

                // One timestep after.
                {
                    auto& candidate = candidates.emplace_back();
                    candidate.a1_et2 = a1_et2;
                    candidate.a1_et2_vals = a1_et2_vals;

                    candidate.a2_et23456s = {EdgeTime{n, Direction::NORTH, t + 1},
                                             EdgeTime{n, Direction::SOUTH, t + 1},
                                             EdgeTime{n, Direction::EAST, t + 1},
                                             EdgeTime{n, Direction::WEST, t + 1},
                                             EdgeTime{n, Direction::WAIT, t + 1}};
                    for (Int idx = 0; idx < 5; ++idx)
                    {
                        auto it = fractional_edges_vec.find(candidate.a2_et23456s[idx]);
                        candidate.a2_et23456_vals[idx] = (it != fractional_edges_vec.end() ? it->second : nullptr);
                    }
                }
            }
        }
    }
    return candidates;
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

    // Skip this separator if an earlier separator found cuts.
    auto& found_cuts = SCIPprobdataGetFoundCutsIndicator(probdata);
    if (found_cuts)
    {
        return SCIP_OKAY;
    }

    // Get the edges fractionally used by each agent.
    const auto& fractional_edges_vec = SCIPprobdataGetFractionalEdgesVec(probdata);

    // Find conflicts.
    Vector<AgentWaitEdgeConflictData> cuts;
    for (const auto& [a1_et1, a1_et1_vals] : fractional_edges_vec)
        if (a1_et1.d != Direction::WAIT)
        {
            // Get the first edge of agent 2.
            const EdgeTime a2_et1{map.get_opposite_edge(a1_et1.et.e), a1_et1.t};
            auto a2_et1_it = fractional_edges_vec.find(a2_et1);
            if (a2_et1_it == fractional_edges_vec.end())
            {
                continue;
            }
            const auto& a2_et1_vals = a2_et1_it->second;

            // Loop through the cut candidates.
            const auto candidates = get_candidates(a1_et1, a2_et1, fractional_edges_vec, map);
            for (const auto& [a1_et2, a1_et2_vals, a2_et23456s, a2_et23456_vals] : candidates)
                for (Agent a1 = 0; a1 < N; ++a1)
                {
                    const auto a1_et1_val = a1_et1_vals[a1];
                    const auto a1_et2_val = a1_et2_vals[a1];
                    if (a1_et1_val > 0 && a1_et2_val > 0)
                    {
                        for (Agent a2 = 0; a2 < N; ++a2)
                            if (a2 != a1)
                            {
                                const auto a2_et1_val = a2_et1_vals[a2];
                                if (a2_et1_val > 0)
                                {
                                    // Get the values of the remaining edges of agent 2.
                                    const auto a2_et2_val = a2_et23456_vals[0] ? a2_et23456_vals[0][a2] : 0.0;
                                    const auto a2_et3_val = a2_et23456_vals[1] ? a2_et23456_vals[1][a2] : 0.0;
                                    const auto a2_et4_val = a2_et23456_vals[2] ? a2_et23456_vals[2][a2] : 0.0;
                                    const auto a2_et5_val = a2_et23456_vals[3] ? a2_et23456_vals[3][a2] : 0.0;
                                    const auto a2_et6_val = a2_et23456_vals[4] ? a2_et23456_vals[4][a2] : 0.0;

                                    // Store a cut if violated.
                                    const auto lhs = a1_et1_val + a1_et2_val +
                                                     a2_et1_val + a2_et2_val + a2_et3_val + a2_et4_val + a2_et5_val +
                                                     a2_et6_val;
                                    if (SCIPisSumGT(scip, lhs, 1.0 + CUT_VIOLATION))
                                    {
                                        auto& cut = cuts.emplace_back();
                                        cut.lhs = lhs;
                                        cut.a1 = a1;
                                        cut.a2 = a2;
                                        cut.a1_et1 = a1_et1;
                                        cut.a1_et2 = a1_et2;
                                        cut.a2_et1 = a2_et1;
                                        cut.a2_et23456s = a2_et23456s;
                                    }
                                }
                            }
                    }
                }
        }

    // Create the most violated cuts.
    Vector<Int> agent_nb_cuts(N * N);
    std::sort(cuts.begin(),
              cuts.end(),
              [](const AgentWaitEdgeConflictData& a, const AgentWaitEdgeConflictData& b) { return a.lhs > b.lhs; });
    for (const auto& cut: cuts)
    {
        const auto& [lhs, a1, a2, a1_et1, a1_et2, a2_et1, a2_et23456s] = cut;
        auto& nb_cuts = agent_nb_cuts[MATRIX(std::min(a1, a2), std::max(a1, a2))];
        if (nb_cuts < 1)
        {
            // Get the remaining edges of agent 2.
            const auto& a2_et2 = a2_et23456s[0];
            const auto& a2_et3 = a2_et23456s[1];
            const auto& a2_et4 = a2_et23456s[2];
            const auto& a2_et5 = a2_et23456s[3];
            const auto& a2_et6 = a2_et23456s[4];

            // Print.
#ifdef PRINT_DEBUG
            {
                const auto [a1_et1_x1, a1_et1_y1] = map.get_xy(a1_et1.n);
                const auto [a1_et1_x2, a1_et1_y2] = map.get_destination_xy(a1_et1);

                const auto [a1_et2_x1, a1_et2_y1] = map.get_xy(a1_et2.n);
                const auto [a1_et2_x2, a1_et2_y2] = map.get_destination_xy(a1_et2);

                const auto [a2_et1_x1, a2_et1_y1] = map.get_xy(a2_et1.n);
                const auto [a2_et1_x2, a2_et1_y2] = map.get_destination_xy(a2_et1);

                const auto [a2_et2_x1, a2_et2_y1] = map.get_xy(a2_et2.n);
                const auto [a2_et2_x2, a2_et2_y2] = map.get_destination_xy(a2_et2);

                const auto [a2_et3_x1, a2_et3_y1] = map.get_xy(a2_et3.n);
                const auto [a2_et3_x2, a2_et3_y2] = map.get_destination_xy(a2_et3);

                const auto [a2_et4_x1, a2_et4_y1] = map.get_xy(a2_et4.n);
                const auto [a2_et4_x2, a2_et4_y2] = map.get_destination_xy(a2_et4);

                const auto [a2_et5_x1, a2_et5_y1] = map.get_xy(a2_et5.n);
                const auto [a2_et5_x2, a2_et5_y2] = map.get_destination_xy(a2_et5);

                const auto [a2_et6_x1, a2_et6_y1] = map.get_xy(a2_et6.n);
                const auto [a2_et6_x2, a2_et6_y2] = map.get_destination_xy(a2_et6);

                debugln("   Creating agent wait-edge conflict cut on edges "
                        "(({},{}),({},{}),{}) and (({},{}),({},{}),{}) for agent {} and "
                        "(({},{}),({},{}),{}), (({},{}),({},{}),{}), (({},{}),({},{}),{}), (({},{}),({},{}),{}), "
                        "(({},{}),({},{}),{}), (({},{}),({},{}),{}) for agent {} with value {} in "
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
        }
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
