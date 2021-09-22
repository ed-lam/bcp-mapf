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

#ifdef USE_TWOVERTEX_CONFLICTS

//#define PRINT_DEBUG

#include "Separator_TwoVertexConflicts.h"
#include "ProblemData.h"
#include "VariableData.h"

#define SEPA_NAME                                  "two_vertex"
#define SEPA_DESC          "Separator for two vertex conflicts"
#define SEPA_PRIORITY                                      1000 // priority of the constraint handler for separation
#define SEPA_FREQ                                             1 // frequency for separating cuts; zero means to separate only in the root node
#define SEPA_MAXBOUNDDIST                                   1.0
#define SEPA_USESSUBSCIP                                  FALSE // does the separator use a secondary SCIP instance? */
#define SEPA_DELAY                                        FALSE // should separation method be delayed, if other separators found cuts? */

struct TwoVertexConflictData
{
    SCIP_Real lhs;
    Agent a1;
    Agent a2;
    EdgeTime a1_et;
    EdgeTime a2_et1;
    EdgeTime a2_et2;
};

#define MATRIX(i,j) (i * N + j)

SCIP_RETCODE twovertex_conflicts_create_cut(
    SCIP* scip,                 // SCIP
    SCIP_ProbData* probdata,    // Problem data
    SCIP_SEPA* sepa,            // Separator
    const Agent a1,             // Agent 1
    const Agent a2,             // Agent 2
    const EdgeTime a1_et,       // Edge of agent 1
    const EdgeTime a2_et1,      // Edge 1 of agent 2
    const EdgeTime a2_et2,      // Edge 2 of agent 2
    SCIP_Result* result         // Output result
)
{
    // Create constraint name.
#ifdef DEBUG
    const auto& map = SCIPprobdataGetMap(probdata);

    const auto [a1_et_x1, a1_et_y1] = map.get_xy(a1_et.n);
    const auto [a1_et_x2, a1_et_y2] = map.get_destination_xy(a1_et);

    const auto [a2_et1_x1, a2_et1_y1] = map.get_xy(a2_et1.n);
    const auto [a2_et1_x2, a2_et1_y2] = map.get_destination_xy(a2_et1);

    const auto [a2_et2_x1, a2_et2_y1] = map.get_xy(a2_et2.n);
    const auto [a2_et2_x2, a2_et2_y2] = map.get_destination_xy(a2_et2);

    auto name = fmt::format("twovertex_conflict("
                            "{},{},"
                            "(({},{}),({},{}),{}),"
                            "(({},{}),({},{}),{}),"
                            "(({},{}),({},{}),{}),"
                            ")",
                            a1, a2,
                            a1_et_x1, a1_et_y1, a1_et_x2, a1_et_y2, a1_et.t,
                            a2_et1_x1, a2_et1_y1, a2_et1_x2, a2_et1_y2, a2_et1.t,
                            a2_et2_x1, a2_et2_y1, a2_et2_x2, a2_et2_y2, a2_et2.t);
#endif

    // Create data for the cut.
    TwoAgentRobustCut cut(scip, a1, a2, 1, 2
#ifdef DEBUG
        , std::move(name)
#endif
    );
    cut.a1_edge_time(0) = a1_et;
    cut.a2_edge_time(0) = a2_et1;
    cut.a2_edge_time(1) = a2_et2;

    // Store the cut.
    SCIP_CALL(SCIPprobdataAddTwoAgentRobustCut(scip, probdata, sepa, std::move(cut), 1, result));

    // Done.
    return SCIP_OKAY;
}

// Separator
static
SCIP_RETCODE twovertex_conflicts_separate(
    SCIP* scip,            // SCIP
    SCIP_SEPA* sepa,       // Separator
    SCIP_RESULT* result    // Pointer to store the result of the separation call
)
{
    // Print.
    debugln("Starting separator for two-vertex conflicts on solution with obj {:.6f}:",
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
    const auto& agent_edges_no_waits = SCIPprobdataGetAgentFractionalEdgesNoWaits(probdata);
    const auto& agent_edges_vec = SCIPprobdataGetAgentFractionalEdgesVec(probdata);

    // Find conflicts.
    Vector<TwoVertexConflictData> cuts;
    for (Agent a1 = 0; a1 < N; ++a1)
    {
        // Get the edges of agent 1.
        const auto& agent_edges_a1 = agent_edges_no_waits[a1];

        // Loop through the edge of agent 1.
        for (const auto [a1_et, a1_et_val] : agent_edges_a1)
        {
            // Get all potential first edge of agent 2.
            Array<EdgeTime, 5> a2_et1s;
            Array<const Vector<SCIP_Real>*, 5> a2_et1_vals;
            Int a2_et1_size = 0;
            {
                const EdgeTime et{a1_et.n, Direction::NORTH, a1_et.t};
                if (const auto it = agent_edges_vec.find(et); it != agent_edges_vec.end())
                {
                    a2_et1s[a2_et1_size] = et;
                    a2_et1_vals[a2_et1_size] = &it->second;
                    ++a2_et1_size;
                }
            }
            {
                const EdgeTime et{a1_et.n, Direction::SOUTH, a1_et.t};
                if (const auto it = agent_edges_vec.find(et); it != agent_edges_vec.end())
                {
                    a2_et1s[a2_et1_size] = et;
                    a2_et1_vals[a2_et1_size] = &it->second;
                    ++a2_et1_size;
                }
            }
            {
                const EdgeTime et{a1_et.n, Direction::EAST, a1_et.t};
                if (const auto it = agent_edges_vec.find(et); it != agent_edges_vec.end())
                {
                    a2_et1s[a2_et1_size] = et;
                    a2_et1_vals[a2_et1_size] = &it->second;
                    ++a2_et1_size;
                }
            }
            {
                const EdgeTime et{a1_et.n, Direction::WEST, a1_et.t};
                if (const auto it = agent_edges_vec.find(et); it != agent_edges_vec.end())
                {
                    a2_et1s[a2_et1_size] = et;
                    a2_et1_vals[a2_et1_size] = &it->second;
                    ++a2_et1_size;
                }
            }
            {
                const EdgeTime et{a1_et.n, Direction::WAIT, a1_et.t};
                if (const auto it = agent_edges_vec.find(et); it != agent_edges_vec.end())
                {
                    a2_et1s[a2_et1_size] = et;
                    a2_et1_vals[a2_et1_size] = &it->second;
                    ++a2_et1_size;
                }
            }

            // Get all potential second edge of agent 2.
            Array<EdgeTime, 5> a2_et2s;
            Array<const Vector<SCIP_Real>*, 5> a2_et2_vals;
            Int a2_et2_size = 0;
            {
                const auto a2_et2_dest = map.get_destination(a1_et);
                const auto a2_et1s_end = a2_et1s.begin() + a2_et1_size;
                {
                    const EdgeTime et{map.get_south(a2_et2_dest), Direction::NORTH, a1_et.t};
                    if (std::find(a2_et1s.begin(), a2_et1s_end, et) == a2_et1s_end)
                        if (const auto it = agent_edges_vec.find(et); it != agent_edges_vec.end())
                        {
                            a2_et2s[a2_et2_size] = et;
                            a2_et2_vals[a2_et2_size] = &it->second;
                            ++a2_et2_size;
                        }
                }
                {
                    const EdgeTime et{map.get_north(a2_et2_dest), Direction::SOUTH, a1_et.t};
                    if (std::find(a2_et1s.begin(), a2_et1s_end, et) == a2_et1s_end)
                        if (const auto it = agent_edges_vec.find(et); it != agent_edges_vec.end())
                        {
                            a2_et2s[a2_et2_size] = et;
                            a2_et2_vals[a2_et2_size] = &it->second;
                            ++a2_et2_size;
                        }
                }
                {
                    const EdgeTime et{map.get_west(a2_et2_dest), Direction::EAST, a1_et.t};
                    if (std::find(a2_et1s.begin(), a2_et1s_end, et) == a2_et1s_end)
                        if (const auto it = agent_edges_vec.find(et); it != agent_edges_vec.end())
                        {
                            a2_et2s[a2_et2_size] = et;
                            a2_et2_vals[a2_et2_size] = &it->second;
                            ++a2_et2_size;
                        }
                }
                {
                    const EdgeTime et{map.get_east(a2_et2_dest), Direction::WEST, a1_et.t};
                    if (std::find(a2_et1s.begin(), a2_et1s_end, et) == a2_et1s_end)
                        if (const auto it = agent_edges_vec.find(et); it != agent_edges_vec.end())
                        {
                            a2_et2s[a2_et2_size] = et;
                            a2_et2_vals[a2_et2_size] = &it->second;
                            ++a2_et2_size;
                        }
                }
                {
                    const EdgeTime et{map.get_wait(a2_et2_dest), Direction::WAIT, a1_et.t};
                    if (std::find(a2_et1s.begin(), a2_et1s_end, et) == a2_et1s_end)
                        if (const auto it = agent_edges_vec.find(et); it != agent_edges_vec.end())
                        {
                            a2_et2s[a2_et2_size] = et;
                            a2_et2_vals[a2_et2_size] = &it->second;
                            ++a2_et2_size;
                        }
                }
            }

            // Check.
            for (Int a2_et1_idx = 0; a2_et1_idx < a2_et1_size; ++a2_et1_idx)
                for (Int a2_et2_idx = 0; a2_et2_idx < a2_et2_size; ++a2_et2_idx)
                    debug_assert(a2_et1s[a2_et1_idx] != a2_et2s[a2_et2_idx]);

            // Loop through the second agent.
            for (Agent a2 = 0; a2 < N; ++a2)
                if (a2 != a1)
                {
                    // Loop through the two edges of agent 2.
                    for (Int a2_et1_idx = 0; a2_et1_idx < a2_et1_size; ++a2_et1_idx)
                        for (Int a2_et2_idx = 0; a2_et2_idx < a2_et2_size; ++a2_et2_idx)
                        {
                            // Get the edges of agent 2.
                            const auto a2_et1 = a2_et1s[a2_et1_idx];
                            const auto a2_et1_val = (*(a2_et1_vals[a2_et1_idx]))[a2];
                            const auto a2_et2 = a2_et2s[a2_et2_idx];
                            const auto a2_et2_val = (*(a2_et2_vals[a2_et2_idx]))[a2];

                            // Check.
                            debug_assert(a1_et.n == a2_et1.n);
                            debug_assert(map.get_destination(a1_et) == map.get_destination(a2_et2));
                            debug_assert(a2_et1 != a2_et2);

                            // Compute the LHS.
                            const auto lhs = a1_et_val + a2_et1_val + a2_et2_val;

                            // Store a cut if violated.
                            if (SCIPisSumGT(scip, lhs, 1.0 + CUT_VIOLATION))
                            {
                                cuts.emplace_back(TwoVertexConflictData{lhs, a1, a2, a1_et, a2_et1, a2_et2});
                            }
                        }
                }
        }
    }

    // Create the most violated cuts.
    Vector<Int> agent_nb_cuts(N * N);
    std::sort(cuts.begin(),
              cuts.end(),
              [](const TwoVertexConflictData& a, const TwoVertexConflictData& b) { return a.lhs > b.lhs; });
    for (const auto& cut : cuts)
    {
        const auto& [lhs, a1, a2, a1_et, a2_et1, a2_et2] = cut;
        auto& nb_cuts = agent_nb_cuts[MATRIX(std::min(a1, a2), std::max(a1, a2))];
        if (nb_cuts < 1)
        {
            // Print.
#ifdef PRINT_DEBUG
            {
                const auto [a1_et_x1, a1_et_y1] = map.get_xy(a1_et.n);
                const auto [a1_et_x2, a1_et_y2] = map.get_destination_xy(a1_et);

                const auto [a2_et1_x1, a2_et1_y1] = map.get_xy(a2_et1.n);
                const auto [a2_et1_x2, a2_et1_y2] = map.get_destination_xy(a2_et1);

                const auto [a2_et2_x1, a2_et2_y1] = map.get_xy(a2_et2.n);
                const auto [a2_et2_x2, a2_et2_y2] = map.get_destination_xy(a2_et2);

                debugln("   Creating two-vertex conflict cut on edges "
                        "(({},{}),({},{}),{}) for agent {} and "
                        "(({},{}),({},{}),{}) and (({},{}),({},{}),{}) for agent {} "
                        "with value {} in "
                        "branch-and-bound node {}",
                        a1_et_x1, a1_et_y1, a1_et_x2, a1_et_y2, a1_et.t,
                        a1,
                        a2_et1_x1, a2_et1_y1, a2_et1_x2, a2_et1_y2, a2_et1.t,
                        a2_et2_x1, a2_et2_y1, a2_et2_x2, a2_et2_y2, a2_et2.t,
                        a2,
                        lhs,
                        SCIPnodeGetNumber(SCIPgetCurrentNode(scip)));
            }
#endif

            // Create the cut.
            SCIP_CALL(twovertex_conflicts_create_cut(scip, probdata, sepa, a1, a2, a1_et, a2_et1, a2_et2, result));
            ++nb_cuts;
        }
    }

    // Done.
    return SCIP_OKAY;
}

// Copy method for separator
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_SEPACOPY(sepaCopyTwoVertexConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);

    // Include separator.
    SCIP_CALL(SCIPincludeSepaTwoVertexConflicts(scip));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Separation method for LP solutions
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_SEPAEXECLP(sepaExeclpTwoVertexConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);
    debug_assert(result);

    // Start.
    *result = SCIP_DIDNOTFIND;

    // Start separator.
    SCIP_CALL(twovertex_conflicts_separate(scip, sepa, result));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Create separator for two-vertex conflicts constraints and include it in SCIP
SCIP_RETCODE SCIPincludeSepaTwoVertexConflicts(
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
                                   sepaExeclpTwoVertexConflicts,
                                   nullptr,
                                   nullptr));
    debug_assert(sepa);

    // Set callbacks.
    SCIP_CALL(SCIPsetSepaCopy(scip, sepa, sepaCopyTwoVertexConflicts));

    // Done.
    return SCIP_OKAY;
}

#endif
