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
#define SEPA_PRIORITY                                   +350000 // priority of the constraint handler for separation
#define SEPA_FREQ                                             1  // frequency for separating cuts; zero means to separate only in the root node
#define SEPA_MAXBOUNDDIST                                   1.0
#define SEPA_USESSUBSCIP                                  FALSE // does the separator use a secondary SCIP instance? */
#define SEPA_DELAY                                        FALSE // should separation method be delayed, if other separators found cuts? */

SCIP_RETCODE stepaside_conflicts_create_cut(
    SCIP* scip,                       // SCIP
    SCIP_ProbData* probdata,          // Problem data
    SCIP_SEPA* sepa,                  // Separator
    const Agent a1,                   // Agent 1
    const Agent a2,                   // Agent 2
    const Array<EdgeTime, 8>& ets,    // Edge-times of the cut
    SCIP_Result* result               // Output result
)
{
    // Create constraint name.
#ifdef DEBUG
    const auto& map = SCIPprobdataGetMap(probdata);

    const auto a1_et1a = ets[0];
    const auto [x1, y1] = map.get_xy(a1_et1a.n);
    const auto [x2, y2] = map.get_destination_xy(a1_et1a.et.e);

    const auto a1_et2a = ets[2];
    const auto [x3, y3] = map.get_xy(a1_et2a.n);
    const auto [x4, y4] = map.get_destination_xy(a1_et2a.et.e);

    auto name = fmt::format("stepaside_conflict("
                            "{},{},"
                            "(({},{}),({},{}),{}),"
                            "(({},{}),({},{}),{}))",
                            a1, a2,
                            x1, y1, x2, y2, a1_et1a.t,
                            x3, y3, x4, y4, a1_et2a.t);
#endif

    // Print.
#ifdef PRINT_DEBUG
    debugln("----------------------");
    for (Int idx = 0; idx < 4; ++idx)
    {
        const auto et = ets[idx];

        const auto [x1, y1] = map.get_xy(et.n);
        const auto [x2, y2] = map.get_destination_xy(a1_et1a.et.e);

        debugln("Agent {} ({},{},{},{}) at {}",
                a1,
                x1, y1, x2, y2, x3, y3, x4, y4,
                et.t);
    }
    debugln("----------------------");
#endif

    // Create data for the cut.
    TwoAgentRobustCut cut(scip, a1, a2, 4, 4
#ifdef DEBUG
        , std::move(name)
#endif
    );
    std::copy(ets.begin(), ets.end(), cut.edge_times_a1().first);

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

    // Check.
    static_assert(Direction::NORTH == 0);
    static_assert(Direction::SOUTH == 1);
    static_assert(Direction::EAST == 2);
    static_assert(Direction::WEST == 3);
    static_assert(Direction::WAIT == 4);

    // Get edges of each agent.
    Array<Vector<HashTable<EdgeTime, SCIP_Real>>, 4> agent_dir_edges;
    agent_dir_edges[0].resize(N);
    agent_dir_edges[1].resize(N);
    agent_dir_edges[2].resize(N);
    agent_dir_edges[3].resize(N);
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

            // Append the path.
            if (SCIPisPositive(scip, var_val))
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
    Array<Pair<Direction, Direction>, 4> directions{
        Pair<Direction, Direction>{Direction::EAST, Direction::WEST},
        Pair<Direction, Direction>{Direction::WEST, Direction::EAST},
        Pair<Direction, Direction>{Direction::NORTH, Direction::SOUTH},
        Pair<Direction, Direction>{Direction::SOUTH, Direction::NORTH},
    };
    Array<EdgeTime, 8> ets;
    for (const auto [d1, d2] : directions)
    {
        for (Agent a1 = 0; a1 < N; ++a1)
        {
            const auto& a1_dir_edges = agent_dir_edges[static_cast<Int>(d1)][a1];

            for (const auto [a1_et1a, a1_et1a_val] : a1_dir_edges)
            {
                ets[0] = a1_et1a;

                const EdgeTime a1_et1b{a1_et1a.et.e, a1_et1a.t + 1};
                ets[1] = a1_et1b;

                for (Agent a2 = 0; a2 < N; ++a2)
                    if (a1 != a2)
                    {
                        const auto& a2_dir_edges = agent_dir_edges[static_cast<Int>(d2)][a2];

                        for (const auto [a2_et2a, a2_et2a_val] : a2_dir_edges)
                            if (a2_et2a.n == a1_et1a.n && a2_et2a.t > a1_et1a.t)
                            {
                                ets[6] = a2_et2a;

                                const EdgeTime a2_et2b{a2_et2a.et.e, a2_et2a.t + 1};
                                ets[7] = a2_et2b;

                                const auto steps = a2_et2a.t - a1_et1a.t + 1;
                                auto [x, y] = map.get_xy(a1_et1a.n);
                                if (d1 == Direction::EAST)
                                {
                                    x += steps;
                                }
                                else if (d1 == Direction::WEST)
                                {
                                    x -= steps;
                                }
                                else if (d1 == Direction::NORTH)
                                {
                                    y -= steps;
                                }
                                else if (d1 == Direction::SOUTH)
                                {
                                    y += steps;
                                }
                                else
                                {
                                    unreachable();
                                }
                                if (x < 0 || y < 0)
                                    continue;

                                if (x < map.width() && y < map.height())
                                {
                                    const auto id = map.get_id(x, y);

                                    const EdgeTime a1_et2a{id, d1, a1_et1a.t + steps};
                                    const EdgeTime a1_et2b{a1_et2a.et.e, a1_et2a.t + 1};
                                    ets[2] = a1_et2a;
                                    ets[3] = a1_et2b;

                                    const EdgeTime a2_et1a = a2_et2a.t - steps >= 0 ?
                                                             EdgeTime{id, d2, a2_et2a.t - steps} :
                                                             EdgeTime{0, Direction::INVALID, 0};
                                    const EdgeTime a2_et1b = a2_et2a.t - steps + 1 >= 0 ?
                                                             EdgeTime{id, d2, a2_et2a.t - steps + 1} :
                                                             EdgeTime{0, Direction::INVALID, 0};
                                    ets[4] = a2_et1a;
                                    ets[5] = a2_et1b;
                                    if (a2_et1a.d == Direction::INVALID && a2_et1b.d == Direction::INVALID)
                                        continue;

                                    debugln("");
                                    SCIP_Real lhs = 0.0;
                                    for (Int idx = 0; idx < 4; ++idx)
                                    {
                                        const auto et = ets[idx];
                                        auto it = a1_dir_edges.find(et);
                                        if (it != a1_dir_edges.end())
                                        {
                                            lhs += it->second;
                                        }

                                        const auto [x1, y1] = map.get_xy(et.n);
                                        auto x2 = x1, y2 = y1;
                                        if (et.d == Direction::NORTH)
                                            y2--;
                                        else if (et.d == Direction::SOUTH)
                                            y2++;
                                        else if (et.d == Direction::EAST)
                                            x2++;
                                        else if (et.d == Direction::WEST)
                                            x2--;
                                        debugln("      (({},{}),({},{}),{})",
                                                x1, y1, x2, y2, et.t);
                                    }
                                    for (Int idx = 4; idx < 8; ++idx)
                                    {
                                        const auto et = ets[idx];
                                        auto it = a2_dir_edges.find(ets[idx]);
                                        if (it != a2_dir_edges.end())
                                        {
                                            lhs += it->second;
                                        }

                                        const auto [x1, y1] = map.get_xy(et.n);
                                        auto x2 = x1, y2 = y1;
                                        if (et.d == Direction::NORTH)
                                            y2--;
                                        else if (et.d == Direction::SOUTH)
                                            y2++;
                                        else if (et.d == Direction::EAST)
                                            x2++;
                                        else if (et.d == Direction::WEST)
                                            x2--;
                                        debugln("      (({},{}),({},{}),{})",
                                                x1, y1, x2, y2, et.t);
                                    }

                                    if (SCIPisSumGT(scip, lhs, 3.0 + CUT_VIOLATION))
                                    {
                                        stepaside_conflicts_create_cut(scip,
                                                                       probdata,
                                                                       sepa,
                                                                       a1,
                                                                       a2,
                                                                       ets,
                                                                       result);
                                        goto NEXT_AGENT;
                                    }
                                }
                            }
                        NEXT_AGENT:;
                    }
            }
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
