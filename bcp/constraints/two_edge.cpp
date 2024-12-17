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

#if defined(USE_TWOEDGE_CONFLICTS) || defined(USE_WAITTWOEDGE_CONFLICTS)

// #define PRINT_DEBUG

#include "constraints/two_edge.h"
#include "problem/problem.h"
#include "problem/variable_data.h"

#ifdef USE_WAITTWOEDGE_CONFLICTS
#define SEPA_NAME         "wait_two_edge"
#else
#define SEPA_NAME         "two_edge"
#endif
#define SEPA_DESC         "Separator for two edge conflicts"
#define SEPA_PRIORITY     109      // priority of the constraint handler for separation
#define SEPA_FREQ         1        // frequency for separating cuts; zero means to separate only in the root node
#define SEPA_MAXBOUNDDIST 1.0
#define SEPA_USESSUBSCIP  FALSE    // does the separator use a secondary SCIP instance? */
#define SEPA_DELAY        FALSE    // should separation method be delayed, if other separators found cuts? */

struct TwoEdgeConflictData
{
    SCIP_Real lhs;
    Agent a1;
    Agent a2;
    Edge a1_e1;
    Edge a1_e2;
#ifdef USE_WAITTWOEDGE_CONFLICTS
    Edge a1_e3;
#endif
    Edge a2_e1;
    Edge a2_e2;
#ifdef USE_WAITTWOEDGE_CONFLICTS
    Edge a2_e3;
#endif
    Time t;
};

#define MATRIX(i,j) (i * N + j)

SCIP_RETCODE twoedge_conflicts_create_cut(
    SCIP* scip,                 // SCIP
    SCIP_ProbData* probdata,    // Problem data
    SCIP_SEPA* sepa,            // Separator
    const Agent a1,             // Agent 1
    const Agent a2,             // Agent 2
    const Edge a1_e1,           // Edge 1 of agent 1
    const Edge a1_e2,           // Edge 2 of agent 1
#ifdef USE_WAITTWOEDGE_CONFLICTS
    const Edge a1_e3,           // Edge 3 of agent 1
#endif
    const Edge a2_e1,           // Edge 1 of agent 2
    const Edge a2_e2,           // Edge 2 of agent 2
#ifdef USE_WAITTWOEDGE_CONFLICTS
    const Edge a2_e3,           // Edge 3 of agent 2
#endif
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
                            "{},{},"
                            "(({},{}),({},{})),"
                            "(({},{}),({},{})),"
                            "{})",
                            a1, a2,
                            x1, y1, x2, y2,
                            x3, y3, x4, y4,
                            t);
#endif

    // Create data for the cut.
    TwoAgentRobustCut cut(scip, a1, a2,
#ifdef USE_WAITTWOEDGE_CONFLICTS
                          3, 3
#else
                          2, 2
#endif
#ifdef DEBUG
                          , std::move(name)
#endif
    );
    cut.a1_edge_time(0) = EdgeTime{a1_e1, t};
    cut.a1_edge_time(1) = EdgeTime{a1_e2, t};
#ifdef USE_WAITTWOEDGE_CONFLICTS
    cut.a1_edge_time(2) = EdgeTime{a1_e3, t};
#endif
    cut.a2_edge_time(0) = EdgeTime{a2_e1, t};
    cut.a2_edge_time(1) = EdgeTime{a2_e2, t};
#ifdef USE_WAITTWOEDGE_CONFLICTS
    cut.a2_edge_time(2) = EdgeTime{a2_e3, t};
#endif

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

    // Skip this separator if an earlier separator found cuts.
    auto& found_cuts = SCIPprobdataGetFoundCutsIndicator(probdata);
    if (found_cuts)
    {
        return SCIP_OKAY;
    }

    // Get the edges fractionally used by each agent.
    const auto& fractional_move_edges = SCIPprobdataGetFractionalMoveEdges(probdata);
    const auto& fractional_edges_vec = SCIPprobdataGetFractionalEdgesVec(probdata);

    // Find conflicts.
    Vector<TwoEdgeConflictData> cuts;
    auto zeros = std::make_unique<SCIP_Real[]>(N);
    for (Agent a1 = 0; a1 < N - 1; ++a1)
    {
        // Get the edges of agent 1.
        const auto& fractional_move_edges_a1 = fractional_move_edges[a1];

        // Loop through the first edge of agent 1.
        for (const auto& [a1_et1, a1_et1_val] : fractional_move_edges_a1)
        {
            const auto t = a1_et1.t;

            // Get the first edge of agent 2.
            const EdgeTime a2_et1{map.get_opposite_edge(a1_et1.et.e), t};
            const auto a2_et1_it = fractional_edges_vec.find(a2_et1);
            const auto a2_et1_vals = a2_et1_it != fractional_edges_vec.end() ? a2_et1_it->second : zeros.get();

            // Get the second edge of agent 1.
            const auto a1_e2_orig = map.get_destination(a1_et1);
            Array<Edge, 4> a1_e2s;
            Int a1_e2_size = 0;
            if (const Edge e{a1_e2_orig, Direction::NORTH}; map.get_destination(e) != a1_et1.n)
            {
                a1_e2s[a1_e2_size] = e;
                ++a1_e2_size;
            }
            if (const Edge e{a1_e2_orig, Direction::SOUTH}; map.get_destination(e) != a1_et1.n)
            {
                a1_e2s[a1_e2_size] = e;
                ++a1_e2_size;
            }
            if (const Edge e{a1_e2_orig, Direction::EAST}; map.get_destination(e) != a1_et1.n)
            {
                a1_e2s[a1_e2_size] = e;
                ++a1_e2_size;
            }
            if (const Edge e{a1_e2_orig, Direction::WEST}; map.get_destination(e) != a1_et1.n)
            {
                a1_e2s[a1_e2_size] = e;
                ++a1_e2_size;
            }

            // Get the wait edge of both agents.
#ifdef USE_WAITTWOEDGE_CONFLICTS
            const EdgeTime a12_et3{a1_e2_orig, Direction::WAIT, t};
            const auto a12_et3_it = fractional_edges_vec.find(a12_et3);
            const auto a12_et3_vals = a12_et3_it != fractional_edges_vec.end() ? a12_et3_it->second : zeros.get();
#endif

            // Loop through the second edge of agent 1.
            for (Int idx = 0; idx < a1_e2_size; ++idx)
            {
                // Get the second edge of agent 1.
                const auto a1_et2 = EdgeTime{a1_e2s[idx], t};
                const auto a1_et2_it = fractional_move_edges_a1.find(a1_et2);
                const auto a1_et2_val = a1_et2_it != fractional_move_edges_a1.end() ? a1_et2_it->second : 0;

                // Get the second edge of agent 2.
                const EdgeTime a2_et2{map.get_opposite_edge(a1_et2.et.e), t};
                const auto a2_et2_it = fractional_edges_vec.find(a2_et2);
                const auto a2_et2_vals = a2_et2_it != fractional_edges_vec.end() ? a2_et2_it->second : zeros.get();

                // Loop through the second agent.
                for (Agent a2 = a1 + 1; a2 < N; ++a2)
                {
                    // Store a cut if violated.
                    const auto lhs = a1_et1_val + a1_et2_val + a2_et1_vals[a2] + a2_et2_vals[a2]
#ifdef USE_WAITTWOEDGE_CONFLICTS
                                     + a12_et3_vals[a1] + a12_et3_vals[a2]
#endif
                                     ;
                    if (SCIPisSumGT(scip, lhs, 1.0 + CUT_VIOLATION))
                    {
                        cuts.emplace_back(TwoEdgeConflictData{lhs,
                                                              a1,
                                                              a2,
                                                              a1_et1.et.e,
                                                              a1_et2.et.e,
#ifdef USE_WAITTWOEDGE_CONFLICTS
                                                              a12_et3.et.e,
#endif
                                                              a2_et1.et.e,
                                                              a2_et2.et.e,
#ifdef USE_WAITTWOEDGE_CONFLICTS
                                                              a12_et3.et.e,
#endif
                                                              t});
                    }
                }
            }
        }
    }

    // Create the most violated cuts.
    Vector<Int> agent_nb_cuts(N * N);
    std::sort(cuts.begin(),
              cuts.end(),
              [](const TwoEdgeConflictData& a, const TwoEdgeConflictData& b) { return a.lhs > b.lhs; });
    for (const auto& cut: cuts)
    {
        const auto& [lhs,
                     a1,
                     a2,
                     a1_e1,
                     a1_e2,
#ifdef USE_WAITTWOEDGE_CONFLICTS
                     a1_e3,
#endif
                     a2_e1,
                     a2_e2,
#ifdef USE_WAITTWOEDGE_CONFLICTS
                     a2_e3,
#endif
                     t] = cut;
        auto& nb_cuts = agent_nb_cuts[MATRIX(std::min(a1, a2), std::max(a1, a2))];
        if (nb_cuts < 1)
        {
            // Print.
#ifdef PRINT_DEBUG
            {
                const auto [a1_e1_x1, a1_e1_y1] = map.get_xy(a1_e1.n);
                const auto [a1_e1_x2, a1_e1_y2] = map.get_destination_xy(a1_e1);

                const auto [a1_e2_x1, a1_e2_y1] = map.get_xy(a1_e2.n);
                const auto [a1_e2_x2, a1_e2_y2] = map.get_destination_xy(a1_e2);

#ifdef USE_WAITTWOEDGE_CONFLICTS
                const auto [a1_e3_x1, a1_e3_y1] = map.get_xy(a1_e3.n);
                const auto [a1_e3_x2, a1_e3_y2] = map.get_destination_xy(a1_e3);
#endif

                const auto [a2_e1_x1, a2_e1_y1] = map.get_xy(a2_e1.n);
                const auto [a2_e1_x2, a2_e1_y2] = map.get_destination_xy(a2_e1);

                const auto [a2_e2_x1, a2_e2_y1] = map.get_xy(a2_e2.n);
                const auto [a2_e2_x2, a2_e2_y2] = map.get_destination_xy(a2_e2);

#ifdef USE_WAITTWOEDGE_CONFLICTS
                const auto [a2_e3_x1, a2_e3_y1] = map.get_xy(a2_e3.n);
                const auto [a2_e3_x2, a2_e3_y2] = map.get_destination_xy(a2_e3);
#endif

                debugln("   Creating two-edge conflict cut on "
                        "(({},{}),({},{}))"
#ifdef USE_WAITTWOEDGE_CONFLICTS
                        ", (({},{}),({},{}))"
#endif
                        " and (({},{}),({},{})) for agent {} "
                            "and "
                        "(({},{}),({},{}))"
#ifdef USE_WAITTWOEDGE_CONFLICTS
                        ", (({},{}),({},{}))"
#endif
                        " and (({},{}),({},{})) for agent {} "
                        "at time {} "
                        "with value {} in "
                        "branch-and-bound node {}",
                        a1_e1_x1, a1_e1_y1, a1_e1_x2, a1_e1_y2,
                        a1_e2_x1, a1_e2_y1, a1_e2_x2, a1_e2_y2,
#ifdef USE_WAITTWOEDGE_CONFLICTS
                        a1_e3_x1, a1_e3_y1, a1_e3_x2, a1_e3_y2,
#endif
                            a1,
                        a2_e1_x1, a2_e1_y1, a2_e1_x2, a2_e1_y2,
                        a2_e2_x1, a2_e2_y1, a2_e2_x2, a2_e2_y2,
#ifdef USE_WAITTWOEDGE_CONFLICTS
                        a2_e3_x1, a2_e3_y1, a2_e3_x2, a2_e3_y2,
#endif
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
#ifdef USE_WAITTWOEDGE_CONFLICTS
                                                   a1_e3,
#endif
                                                   a2_e1,
                                                   a2_e2,
#ifdef USE_WAITTWOEDGE_CONFLICTS
                                                   a2_e3,
#endif
                                                   t,
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
SCIP_DECL_SEPACOPY(sepaCopyTwoEdgeConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);

    // Include separator.
    SCIP_CALL(SCIPincludeSepaTwoEdgeConflicts(scip));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Separation method for LP solutions
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
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
#pragma GCC diagnostic pop

// Create separator for two-edge conflicts constraints and include it in SCIP
SCIP_RETCODE SCIPincludeSepaTwoEdgeConflicts(
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
                                   sepaExeclpTwoEdgeConflicts,
                                   nullptr,
                                   nullptr));
    debug_assert(sepa);

    // Set callbacks.
    SCIP_CALL(SCIPsetSepaCopy(scip, sepa, sepaCopyTwoEdgeConflicts));

    // Done.
    return SCIP_OKAY;
}

#endif
