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

#ifdef USE_STEPASIDE_CONFLICTS

//#define PRINT_DEBUG

#include "Separator_StepAsideConflicts.h"
#include "ProblemData.h"
#include "VariableData.h"

#define SEPA_NAME                                  "step_aside"
#define SEPA_DESC          "Separator for step aside conflicts"
#define SEPA_PRIORITY                                      1000 // priority of the constraint handler for separation
#define SEPA_FREQ                                             1 // frequency for separating cuts; zero means to separate only in the root node
#define SEPA_MAXBOUNDDIST                                   1.0
#define SEPA_USESSUBSCIP                                  FALSE // does the separator use a secondary SCIP instance? */
#define SEPA_DELAY                                        FALSE // should separation method be delayed, if other separators found cuts? */

struct StepAsideConflictData
{
    SCIP_Real lhs;
    Agent a1;
    Agent a2;
    Array<EdgeTime, 4> a1_ets;
    Array<EdgeTime, 4> a2_ets;
};

#define MATRIX(i,j) (i * N + j)

SCIP_RETCODE stepaside_conflicts_create_cut(
    SCIP* scip,                          // SCIP
    SCIP_ProbData* probdata,             // Problem data
    SCIP_SEPA* sepa,                     // Separator
    const Agent a1,                      // Agent 1
    const Agent a2,                      // Agent 2
    const Array<EdgeTime, 4>& a1_ets,    // Edge-times of agent 1
    const Array<EdgeTime, 4>& a2_ets,    // Edge-times of agent 2
    SCIP_Result* result                  // Output result
)
{
    // Create constraint name.
#ifdef DEBUG
    const auto& map = SCIPprobdataGetMap(probdata);

    const auto a1_et1 = a1_ets[0];
    const auto [a1_x1, a1_y1] = map.get_xy(a1_et1.n);
    const auto [a1_x2, a1_y2] = map.get_destination_xy(a1_et1.et.e);

    const auto a2_et1 = a2_ets[2];
    const auto [a2_x1, a2_y1] = map.get_xy(a2_et1.n);
    const auto [a2_x2, a2_y2] = map.get_destination_xy(a2_et1.et.e);

    auto name = fmt::format("stepaside_conflict("
                            "{},{},"
                            "(({},{}),({},{}),{}),"
                            "(({},{}),({},{}),{}))",
                            a1, a2,
                            a1_x1, a1_y1, a1_x2, a1_y2, a1_et1.t,
                            a2_x1, a2_y1, a2_x2, a2_y2, a2_et1.t);
#endif

    // Create data for the cut.
    TwoAgentRobustCut cut(scip, a1, a2, 4, 4
#ifdef DEBUG
        , std::move(name)
#endif
    );
    std::copy(a1_ets.begin(), a1_ets.end(), cut.edge_times_a1().first);
    std::copy(a2_ets.begin(), a2_ets.end(), cut.edge_times_a2().first);

    // Store the cut.
    SCIP_CALL(SCIPprobdataAddTwoAgentRobustCut(scip, probdata, sepa, std::move(cut), 3, result));

    // Done.
    return SCIP_OKAY;
}

// Separator
static
SCIP_RETCODE stepaside_conflicts_separate(
    SCIP* scip,            // SCIP
    SCIP_SEPA* sepa,       // Separator
    SCIP_RESULT* result    // Pointer to store the result of the separation call
)
{
    // Print.
    debugln("Starting separator for step-aside conflicts on solution with obj {:.6f}:",
            SCIPgetSolOrigObj(scip, nullptr));

    // Check.
    static_assert(Direction::NORTH == 0);
    static_assert(Direction::SOUTH == 1);
    static_assert(Direction::EAST == 2);
    static_assert(Direction::WEST == 3);
    static_assert(Direction::WAIT == 4);

    // Print paths.
#ifdef PRINT_DEBUG
    print_used_paths(scip);
#endif

    // Get problem data.
    auto probdata = SCIPgetProbData(scip);
    const auto N = SCIPprobdataGetN(probdata);
    const auto& map = SCIPprobdataGetMap(probdata);

    // Get variables.
    const auto& agent_vars = SCIPprobdataGetAgentVars(probdata);

    // Get edges of each agent.
    Array<Vector<HashTable<EdgeTime, SCIP_Real>>, 4> agent_dir_edges;
    agent_dir_edges[Direction::NORTH].resize(N);
    agent_dir_edges[Direction::SOUTH].resize(N);
    agent_dir_edges[Direction::EAST].resize(N);
    agent_dir_edges[Direction::WEST].resize(N);
    for (Agent a = 0; a < N; ++a)
    {
        // Calculate the number of times an edge is used by summing the columns.
        for (auto var : agent_vars[a])
        {
            // Get the path.
            debug_assert(var);
            auto vardata = SCIPvarGetData(var);
            const auto path_length = SCIPvardataGetPathLength(vardata);
            const auto path = SCIPvardataGetPath(vardata);

            // Get the variable value.
            const auto var_val = SCIPgetSolVal(scip, nullptr, var);

            // Store the edge.
            if (!SCIPisIntegral(scip, var_val))
            {
                for (Time t = 0; t < path_length - 1; ++t)
                    if (path[t].d != Direction::WAIT)
                    {
                        const EdgeTime et{path[t], t};
                        agent_dir_edges[path[t].d][a][et] += var_val;
                    }
            }
        }

        // Delete edges with integer values.
        for (Int d = 0; d < 4; ++d)
        {
            auto& edges = agent_dir_edges[d][a];
            for (auto it = edges.begin(); it != edges.end();)
            {
                const auto& [et, val] = *it;
                if (SCIPisIntegral(scip, val))
                {
                    it = edges.erase(it);
                }
                else
                {
                    ++it;
                }
            }
        }

        // Print.
#ifdef PRINT_DEBUG
        {
            bool has_edges = false;
            for (Int d = 0; d < 4; ++d)
            {
                has_edges |= !agent_dir_edges[d][a].empty();
            }

            if (has_edges)
            {
                debugln("   Fractional edges for agent {}:", a);
                for (Int d = 0; d < 4; ++d)
                    for (const auto [et, val] : agent_dir_edges[d][a])
                    {
                        const auto [x1, y1] = map.get_xy(et.n);
                        const auto [x2, y2] = map.get_destination_xy(et.et.e);
                        debugln("      (({},{}),({},{}),{}) val {:.4f}",
                                x1, y1, x2, y2, et.t, val);
                    }
            }
        }
#endif
    }

    // Find conflicts.
    Vector<StepAsideConflictData> cuts;
    Array<Pair<Direction, Direction>, 4> directions{
        Pair<Direction, Direction>{Direction::EAST, Direction::WEST},
        Pair<Direction, Direction>{Direction::WEST, Direction::EAST},
        Pair<Direction, Direction>{Direction::NORTH, Direction::SOUTH},
        Pair<Direction, Direction>{Direction::SOUTH, Direction::NORTH},
    };
    for (const auto& [d1, d2] : directions)
    {
        // Loop through the first agent.
        for (Agent a1 = 0; a1 < N; ++a1)
        {
            // Get the edges of the first agent.
            const auto& a1_dir_edges = agent_dir_edges[d1][a1];

            // Loop through the first edge of agent 1.
            for (const auto [a1_et1, a1_et1_val] : a1_dir_edges)
            {
                // Get the second edge of agent 1.
                const EdgeTime a1_et2{a1_et1.et.e, a1_et1.t + 1};
                const auto a1_et2_it = a1_dir_edges.find(a1_et2);
                const auto a1_et2_val = a1_et2_it != a1_dir_edges.end() ? a1_et2_it->second : 0.0;

                // Loop through the second agent.
                const auto a2_et3_orig = map.get_destination(a1_et1.et.e);
                for (Agent a2 = a1 + 1; a2 < N; ++a2)
                {
                    // Get the edges of the second agent.
                    const auto& a2_dir_edges = agent_dir_edges[d2][a2];

                    // Loop through the third edge of agent 2.
                    for (const auto [a2_et3, a2_et3_val] : a2_dir_edges)
                        if (a2_et3.n == a2_et3_orig && a2_et3.t > a1_et1.t)
                        {
                            // Get the fourth edge of agent 2.
                            const EdgeTime a2_et4{a2_et3.et.e, a2_et3.t + 1};
                            const auto a2_et4_it = a2_dir_edges.find(a2_et4);
                            const auto a2_et4_val = a2_et4_it != a2_dir_edges.end() ? a2_et4_it->second : 0.0;

                            // Compute the end of the corridor.
                            const auto h = a2_et3.t - a1_et1.t;
                            debug_assert(h > 0);
                            auto [x, y] = map.get_xy(a1_et1.n);
                            switch (d1)
                            {
                                case Direction::NORTH: y -= h; break;
                                case Direction::SOUTH: y += h; break;
                                case Direction::EAST: x += h; break;
                                case Direction::WEST: x -= h; break;
                                default: unreachable();
                            }
                            if (!(0 <= x && x < map.width() && 0 <= y && y < map.height()))
                            {
                                continue;
                            }

                            // Get the third edge of agent 1.
                            const auto a1_et3_orig = map.get_id(x, y);
                            const EdgeTime a1_et3{a1_et3_orig, d1, a1_et1.t + h};
                            const auto a1_et3_it = a1_dir_edges.find(a1_et3);
                            const auto a1_et3_val = a1_et3_it != a1_dir_edges.end() ? a1_et3_it->second : 0.0;

                            // Get the fourth edge of agent 1.
                            const EdgeTime a1_et4{a1_et3.et.e, a1_et3.t + 1};
                            const auto a1_et4_it = a1_dir_edges.find(a1_et4);
                            const auto a1_et4_val = a1_et4_it != a1_dir_edges.end() ? a1_et4_it->second : 0.0;

                            // Get the first edge of agent 2.
                            const EdgeTime a2_et1{map.get_destination(a1_et3.et.e), d2, a1_et1.t};
                            const auto a2_et1_it = a2_dir_edges.find(a2_et1);
                            const auto a2_et1_val = a2_et1_it != a2_dir_edges.end() ? a2_et1_it->second : 0.0;

                            // Get the second edge of agent 2.
                            const EdgeTime a2_et2{a2_et1.et.e, a2_et1.t + 1};
                            const auto a2_et2_it = a2_dir_edges.find(a2_et2);
                            const auto a2_et2_val = a2_et2_it != a2_dir_edges.end() ? a2_et2_it->second : 0.0;

                            // Compute the LHS.
                            const auto lhs = a1_et1_val + a1_et2_val + a1_et3_val + a1_et4_val +
                                             a2_et1_val + a2_et2_val + a2_et3_val + a2_et4_val;

                            // Store a cut if violated.
                            if (SCIPisSumGT(scip, lhs, 3.0 + CUT_VIOLATION))
                            {
                                cuts.emplace_back(StepAsideConflictData{lhs,
                                                                        a1,
                                                                        a2,
                                                                        {a1_et1, a1_et2, a1_et3, a1_et4},
                                                                        {a2_et1, a2_et2, a2_et3, a2_et4}});
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
              [](const StepAsideConflictData& a, const StepAsideConflictData& b) { return a.lhs > b.lhs; });
    for (const auto& cut : cuts)
    {
        const auto& [lhs, a1, a2, a1_ets, a2_ets] = cut;
        auto& nb_cuts = agent_nb_cuts[MATRIX(std::min(a1, a2), std::max(a1, a2))];
        if (nb_cuts < 1)
        {
            // Print.
#ifdef PRINT_DEBUG
            {
                const auto a1_et1 = a1_ets[0];
                const auto [a1_et1_x1, a1_et1_y1] = map.get_xy(a1_et1.n);
                const auto [a1_et1_x2, a1_et1_y2] = map.get_destination_xy(a1_et1.et.e);

                const auto a1_et2 = a1_ets[1];
                const auto [a1_et2_x1, a1_et2_y1] = map.get_xy(a1_et2.n);
                const auto [a1_et2_x2, a1_et2_y2] = map.get_destination_xy(a1_et2.et.e);

                const auto a1_et3 = a1_ets[2];
                const auto [a1_et3_x1, a1_et3_y1] = map.get_xy(a1_et3.n);
                const auto [a1_et3_x2, a1_et3_y2] = map.get_destination_xy(a1_et3.et.e);

                const auto a1_et4 = a1_ets[3];
                const auto [a1_et4_x1, a1_et4_y1] = map.get_xy(a1_et4.n);
                const auto [a1_et4_x2, a1_et4_y2] = map.get_destination_xy(a1_et4.et.e);

                const auto a2_et1 = a2_ets[0];
                const auto [a2_et1_x1, a2_et1_y1] = map.get_xy(a2_et1.n);
                const auto [a2_et1_x2, a2_et1_y2] = map.get_destination_xy(a2_et1.et.e);

                const auto a2_et2 = a2_ets[1];
                const auto [a2_et2_x1, a2_et2_y1] = map.get_xy(a2_et2.n);
                const auto [a2_et2_x2, a2_et2_y2] = map.get_destination_xy(a2_et2.et.e);

                const auto a2_et3 = a2_ets[2];
                const auto [a2_et3_x1, a2_et3_y1] = map.get_xy(a2_et3.n);
                const auto [a2_et3_x2, a2_et3_y2] = map.get_destination_xy(a2_et3.et.e);

                const auto a2_et4 = a2_ets[3];
                const auto [a2_et4_x1, a2_et4_y1] = map.get_xy(a2_et4.n);
                const auto [a2_et4_x2, a2_et4_y2] = map.get_destination_xy(a2_et4.et.e);

                debugln("   Creating step-aside conflict cut on "
                        "(({},{}),({},{}),{}), (({},{}),({},{}),{}), (({},{}),({},{}),{}) and (({},{}),({},{}),{}) for agent {} and "
                        "(({},{}),({},{}),{}), (({},{}),({},{}),{}), (({},{}),({},{}),{}) and (({},{}),({},{}),{}) for agent {} "
                        "with value {} in "
                        "branch-and-bound node {}",
                        a1_et1_x1, a1_et1_y1, a1_et1_x2, a1_et1_y2, a1_et1.t,
                        a1_et2_x1, a1_et2_y1, a1_et2_x2, a1_et2_y2, a1_et2.t,
                        a1_et3_x1, a1_et3_y1, a1_et3_x2, a1_et3_y2, a1_et3.t,
                        a1_et4_x1, a1_et4_y1, a1_et4_x2, a1_et4_y2, a1_et4.t,
                        a1,
                        a2_et1_x1, a2_et1_y1, a2_et1_x2, a2_et1_y2, a2_et1.t,
                        a2_et2_x1, a2_et2_y1, a2_et2_x2, a2_et2_y2, a2_et2.t,
                        a2_et3_x1, a2_et3_y1, a2_et3_x2, a2_et3_y2, a2_et3.t,
                        a2_et4_x1, a2_et4_y1, a2_et4_x2, a2_et4_y2, a2_et4.t,
                        a2,
                        lhs,
                        SCIPnodeGetNumber(SCIPgetCurrentNode(scip)));
            }
#endif

            // Create the cut.
            SCIP_CALL(stepaside_conflicts_create_cut(scip, probdata, sepa, a1, a2, a1_ets, a2_ets, result));
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
SCIP_DECL_SEPACOPY(sepaCopyStepAsideConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);

    // Include separator.
    SCIP_CALL(SCIPincludeSepaStepAsideConflicts(scip));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Separation method for LP solutions
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_SEPAEXECLP(sepaExeclpStepAsideConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);
    debug_assert(result);

    // Start.
    *result = SCIP_DIDNOTFIND;

    // Start separator.
    SCIP_CALL(stepaside_conflicts_separate(scip, sepa, result));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Create separator for step-aside conflicts constraints and include it in SCIP
SCIP_RETCODE SCIPincludeSepaStepAsideConflicts(
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
                                   sepaExeclpStepAsideConflicts,
                                   nullptr,
                                   nullptr));
    debug_assert(sepa);

    // Set callbacks.
    SCIP_CALL(SCIPsetSepaCopy(scip, sepa, sepaCopyStepAsideConflicts));

    // Done.
    return SCIP_OKAY;
}

#endif
