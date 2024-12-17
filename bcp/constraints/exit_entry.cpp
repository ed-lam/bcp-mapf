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

#ifdef USE_EXITENTRY_CONFLICTS

// #define PRINT_DEBUG

#include "constraints/exit_entry.h"
#include "problem/problem.h"
#include "problem/variable_data.h"

#define SEPA_NAME         "exit_entry"
#define SEPA_DESC         "Separator for exit entry conflicts"
#define SEPA_PRIORITY     111      // priority of the constraint handler for separation
#define SEPA_FREQ         1        // frequency for separating cuts; zero means to separate only in the root node
#define SEPA_MAXBOUNDDIST 1.0
#define SEPA_USESSUBSCIP  FALSE    // does the separator use a secondary SCIP instance? */
#define SEPA_DELAY        FALSE    // should separation method be delayed, if other separators found cuts? */

struct ExitEntryConflictData
{
    SCIP_Real lhs;
    Agent a1;
    Agent a2;
    Edge a1_e;
    Int a2_es_size;
    Array<Edge, 11> a2_es;
    Time t;
};

#define MATRIX(i,j) (i * N + j)

SCIP_RETCODE exitentry_conflicts_create_cut(
    SCIP* scip,                      // SCIP
    SCIP_ProbData* probdata,         // Problem data
    SCIP_SEPA* sepa,                 // Separator
    const Agent a1,                  // Agent 1
    const Agent a2,                  // Agent 2
    const Edge a1_e,                 // Edge of agent 1
    const Int a2_es_size,            // Number of edges for agent 2
    const Array<Edge, 11>& a2_es,    // Edges of agent 2
    const Time t,                    // Time
    SCIP_Result* result              // Output result
)
{
    // Create constraint name.
#ifdef DEBUG
    auto name = fmt::format("exitentry_conflict({},{},{})", t, a1, a2);
#endif

    // Create data for the cut.
    TwoAgentRobustCut cut{scip, a1, a2, 1, a2_es_size
#ifdef DEBUG
        , std::move(name)
#endif
    };
    cut.a1_edge_time(0) = EdgeTime{a1_e, t};
    for (Int idx = 0; idx < a2_es_size; ++idx)
    {
        cut.a2_edge_time(idx) = EdgeTime{a2_es[idx], t};
    }
    // std::copy(a2_es.begin(), a2_es.begin() + a2_es_size, cut.edges_a2().first);

    // Store the cut.
    SCIP_CALL(SCIPprobdataAddTwoAgentRobustCut(scip, probdata, sepa, std::move(cut), 1, result));

    // Done.
    return SCIP_OKAY;
}

// Separator
static
SCIP_RETCODE exitentry_conflicts_separate(
    SCIP* scip,            // SCIP
    SCIP_SEPA* sepa,       // Separator
    SCIP_RESULT* result    // Pointer to store the result of the separation call
)
{
    // Print.
    debugln("Starting separator for exit-entry conflicts on solution with obj {:.6f}:",
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
    const auto& fractonal_move_edges = SCIPprobdataGetFractionalMoveEdges(probdata);
    const auto& fractional_edges_vec = SCIPprobdataGetFractionalEdgesVec(probdata);

    // Find conflicts.
    Vector<ExitEntryConflictData> cuts;
    for (Agent a1 = 0; a1 < N; ++a1)
    {
        // Get the edges of agent 1.
        const auto& fractional_move_edges_a1 = fractonal_move_edges[a1];

        // Loop through all edges of agent 1.
        for (const auto& [a1_et, a1_et_val] : fractional_move_edges_a1)
        {
            // Get the vertices of the edge.
            const auto t = a1_et.t;
            const auto a1_e = a1_et.et.e;
            debug_assert(a1_e.d != Direction::WAIT);
            const auto n1 = a1_e.n;
            const auto n2 = map.get_destination(a1_e);

            // Make the incompatible edges for agent 2.
            Array<Edge, 11> a2_es{map.get_opposite_edge(a1_e),
                                  Edge{n1, Direction::NORTH},
                                  Edge{n1, Direction::SOUTH},
                                  Edge{n1, Direction::EAST},
                                  Edge{n1, Direction::WEST},
                                  Edge{n1, Direction::WAIT}};
            Int a2_es_size = 6;
            if (const auto orig = map.get_south(n2); orig != n1)
            {
                a2_es[a2_es_size] = Edge{orig, Direction::NORTH};
                ++a2_es_size;
            }
            if (const auto orig = map.get_north(n2); orig != n1)
            {
                a2_es[a2_es_size] = Edge{orig, Direction::SOUTH};
                ++a2_es_size;
            }
            if (const auto orig = map.get_west(n2); orig != n1)
            {
                a2_es[a2_es_size] = Edge{orig, Direction::EAST};
                ++a2_es_size;
            }
            if (const auto orig = map.get_east(n2); orig != n1)
            {
                a2_es[a2_es_size] = Edge{orig, Direction::WEST};
                ++a2_es_size;
            }
            if (const auto orig = map.get_wait(n2); orig != n1)
            {
                a2_es[a2_es_size] = Edge{orig, Direction::WAIT};
                ++a2_es_size;
            }

            // Get the values of those edges.
            Array<SCIP_Real*, 11> a2_es_vals{nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
                                             nullptr, nullptr, nullptr, nullptr, nullptr};
            Int a2_es_vals_size = 0;
            for (Int idx = 0; idx < a2_es_size; ++idx)
            {
                const auto e = a2_es[idx];
                if (auto it = fractional_edges_vec.find(EdgeTime{e, t}); it != fractional_edges_vec.end())
                {
                    a2_es_vals[a2_es_vals_size] = it->second;
                    ++a2_es_vals_size;
                }
            }

            // Loop through the second agent.
            for (Agent a2 = 0; a2 < N; ++a2)
                if (a2 != a1)
                {
                    // Compute the LHS.
                    SCIP_Real lhs = a1_et_val;
                    for (Int idx = 0; idx < a2_es_vals_size; ++idx)
                    {
                        lhs += a2_es_vals[idx][a2];
                    }

                    // Store a cut if violated.
                    if (SCIPisSumGT(scip, lhs, 1.0 + CUT_VIOLATION))
                    {
                        cuts.emplace_back(ExitEntryConflictData{lhs, a1, a2, a1_e, a2_es_size, a2_es, t});
                    }
                }
        }
    }

    // Create the most violated cuts.
    Vector<Int> agent_nb_cuts(N * N);
    std::sort(cuts.begin(),
              cuts.end(),
              [](const ExitEntryConflictData& a, const ExitEntryConflictData& b) { return a.lhs > b.lhs; });
    for (const auto& cut : cuts)
    {
        const auto& [lhs, a1, a2, a1_e, a2_es_size, a2_es, t] = cut;
        auto& nb_cuts = agent_nb_cuts[MATRIX(std::min(a1, a2), std::max(a1, a2))];
        if (nb_cuts < 1)
        {
            // Print.
#ifdef PRINT_DEBUG
            {
                const auto [a1_et_x1, a1_et_y1] = map.get_xy(a1_e.n);
                const auto [a1_et_x2, a1_et_y2] = map.get_destination_xy(a1_e);

                String a2_str;
                for (Int idx = 0; idx < a2_es_size; ++idx)
                {
                    if (!a2_str.empty())
                    {
                        a2_str += ", ";
                    }
                    const auto e = a2_es[idx];
                    const auto [x1, y1] = map.get_xy(e.n);
                    const auto [x2, y2] = map.get_destination_xy(e);
                    a2_str += fmt::format("(({},{}),({},{}),{})", x1, y1, x2, y2, t);
                }

                debugln("   Creating exit-entry conflict cut on (({},{}),({},{}),{}) for agent {} and "
                        "{} for agent {} with value {} in branch-and-bound node {}",
                        a1_et_x1, a1_et_y1, a1_et_x2, a1_et_y2, t,
                        a1,
                        a2_str,
                        a2,
                        lhs,
                        SCIPnodeGetNumber(SCIPgetCurrentNode(scip)));
            }
#endif

            // Create the cut.
            SCIP_CALL(exitentry_conflicts_create_cut(scip, probdata, sepa, a1, a2, a1_e, a2_es_size, a2_es, t, result));
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
SCIP_DECL_SEPACOPY(sepaCopyExitEntryConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);

    // Include separator.
    SCIP_CALL(SCIPincludeSepaExitEntryConflicts(scip));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Separation method for LP solutions
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_SEPAEXECLP(sepaExeclpExitEntryConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);
    debug_assert(result);

    // Start.
    *result = SCIP_DIDNOTFIND;

    // Start separator.
    SCIP_CALL(exitentry_conflicts_separate(scip, sepa, result));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Create separator for exit-entry conflicts constraints and include it in SCIP
SCIP_RETCODE SCIPincludeSepaExitEntryConflicts(
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
                                   sepaExeclpExitEntryConflicts,
                                   nullptr,
                                   nullptr));
    debug_assert(sepa);

    // Set callbacks.
    SCIP_CALL(SCIPsetSepaCopy(scip, sepa, sepaCopyExitEntryConflicts));

    // Done.
    return SCIP_OKAY;
}

#endif
