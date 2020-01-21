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

#ifdef USE_CORRIDOR_CONFLICTS

//#define PRINT_DEBUG

#include "Separator_CorridorConflicts.h"
#include "ProblemData.h"
#include "VariableData.h"
#include "ConstraintHandler_EdgeConflicts.h"

#define SEPA_NAME                          "corridor_conflicts"
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

    // Get variables.
    const auto& agent_vars = SCIPprobdataGetAgentVars(probdata);

    // Force cuts for debugging.
//    {
//        Vector<Agent> a1s{};
//        Vector<Position> a1_x1s{};
//        Vector<Position> a1_y1s{};
//        Vector<Direction> a1_d1s{};
//        Vector<Time> a1_t1s{};
//        Vector<Position> a1_x2s{};
//        Vector<Position> a1_y2s{};
//        Vector<Direction> a1_d2s{};
//        Vector<Time> a1_t2s{};
//
//        Vector<Agent> a2s{};
//        Vector<Position> a2_x1s{};
//        Vector<Position> a2_y1s{};
//        Vector<Direction> a2_d1s{};
//        Vector<Time> a2_t1s{};
//        Vector<Position> a2_x2s{};
//        Vector<Position> a2_y2s{};
//        Vector<Direction> a2_d2s{};
//        Vector<Time> a2_t2s{};
//
//        for (size_t idx = 0; idx < a1s.size(); ++idx)
//        {
//            const auto a1 = a1s[idx];
//            const auto a2 = a2s[idx];
//
//            EdgeTime a1_e1(NodeTime(map.get_id(a1_x1s[idx], a1_y1s[idx]), a1_t1s[idx]), a1_d1s[idx]);
//            EdgeTime a1_e2(NodeTime(map.get_id(a1_x2s[idx], a1_y2s[idx]), a1_t2s[idx]), a1_d2s[idx]);
//            EdgeTime a2_e1(NodeTime(map.get_id(a2_x1s[idx], a2_y1s[idx]), a2_t1s[idx]), a2_d1s[idx]);
//            EdgeTime a2_e2(NodeTime(map.get_id(a2_x2s[idx], a2_y2s[idx]), a2_t2s[idx]), a2_d2s[idx]);
//
//            SCIP_CALL(corridor_conflicts_create_cut(scip,
//                                                    map,
//                                                    sepa,
//                                                    sepadata,
//                                                    a1,
//                                                    a2,
//                                                    a1_e1,
//                                                    a1_e2,
//                                                    a2_e1,
//                                                    a2_e2,
//                                                    agent_vars[a1],
//                                                    agent_vars[a2],
//                                                    result));
//        }
//    }

    // Get edges of each agent.
    Vector<HashTable<EdgeTime, SCIP_Real>> agent_edges(N);
    for (Agent a = 0; a < N; ++a)
    {
        // Calculate the number of times an edge is used by summing the columns.
        auto& agent_edges_a = agent_edges[a];
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
                        agent_edges_a[et] += var_val;
                    }
            }
        }

        // Delete edges with integer values.
        for (auto it = agent_edges_a.begin(); it != agent_edges_a.end();)
        {
            const auto& [et, val] = *it;
            if (SCIPisIntegral(scip, val))
            {
                it = agent_edges_a.erase(it);
            }
            else
            {
                ++it;
            }
        }

        // Print.
#ifdef PRINT_DEBUG
        if (!agent_edges_a.empty())
        {
            debugln("   Fractional edges for agent {}:", a);
            for (const auto [et, val] : agent_edges_a)
            {
                const auto [x1, y1] = map.get_xy(et.e.n);
                auto x2 = x1, y2 = y1;
                if (et.e.d == Direction::NORTH)
                    y2--;
                else if (et.e.d == Direction::SOUTH)
                    y2++;
                else if (et.e.d == Direction::EAST)
                    x2++;
                else if (et.e.d == Direction::WEST)
                    x2--;
                debugln("      (({},{}),({},{}),{}) val {:.4f}",
                        x1, y1, x2, y2, et.t, val);
            }
        }
#endif
    }

    // Find conflicts.
    for (Agent a1 = 0; a1 < N - 1; ++a1)
    {
        // Get the edges of agent 1.
        const auto& agent_edges_a1 = agent_edges[a1];

        // Loop through the second agent.
        for (Agent a2 = a1 + 1; a2 < N; ++a2)
        {
            // Get the edges of agent 2.
            const auto& agent_edges_a2 = agent_edges[a2];

            // Loop through all edges of agent 1.
            for (const auto [a1_e1, a1_e1_val] : agent_edges_a1)
            {
                // See if agent 2 is using the reverse edge.
                const auto t = a1_e1.t;
                const EdgeTime a2_e1{get_opposite_edge(a1_e1.et.e, map), t};
                const auto it_a2_e1 = agent_edges_a2.find(a2_e1);
                if (it_a2_e1 != agent_edges_a2.end())
                {
                    // Store the value.
                    const auto a2_e1_val = it_a2_e1->second;

                    // See if the same edge is used at either one timestep before or one
                    // timestep after.
                    const Array<Time, 2> offsets{-1, 1};
                    for (const auto offset : offsets)
                    {
                        const EdgeTime a1_e2{a1_e1.et.e, t + offset};
                        const EdgeTime a2_e2{a2_e1.et.e, t + offset};
                        auto it_a1_e2 = agent_edges_a1.find(a1_e2);
                        auto it_a2_e2 = agent_edges_a2.find(a2_e2);
                        if (it_a1_e2 != agent_edges_a1.end() ||
                            it_a2_e2 != agent_edges_a2.end())
                        {
                            // Store the value.
                            const auto a1_e2_val = it_a1_e2 != agent_edges_a1.end() ?
                                                   it_a1_e2->second :
                                                   0.0;
                            const auto a2_e2_val = it_a2_e2 != agent_edges_a2.end() ?
                                                   it_a2_e2->second :
                                                   0.0;

                            // Determine if there is a conflict.
                            const auto lhs = a1_e1_val + a1_e2_val +
                                             a2_e1_val + a2_e2_val;
                            if (SCIPisGT(scip, lhs, 1.0))
                            {
                                // Print.
                                debugln("   Creating corridor conflict cut on edges "
                                        "({},{},{},{}) and ({},{},{},{}) for agent {} "
                                        "and edges ({},{},{},{}) and ({},{},{},{}) for "
                                        "agent {} with value {} in branch-and-bound node "
                                        "{}",
                                        map.get_x(a1_e1.e.n),
                                        map.get_y(a1_e1.e.n),
                                        a1_e1.e.d == Direction::NORTH ? "north" :
                                        a1_e1.e.d == Direction::SOUTH ? "south" :
                                        a1_e1.e.d == Direction::EAST ? "east" :
                                        a1_e1.e.d == Direction::WEST ? "west" :
                                        a1_e1.e.d == Direction::WAIT ? "wait" : "other",
                                        a1_e1.t,
                                        map.get_x(a1_e2.e.n),
                                        map.get_y(a1_e2.e.n),
                                        a1_e2.e.d == Direction::NORTH ? "north" :
                                        a1_e2.e.d == Direction::SOUTH ? "south" :
                                        a1_e2.e.d == Direction::EAST ? "east" :
                                        a1_e2.e.d == Direction::WEST ? "west" :
                                        a1_e2.e.d == Direction::WAIT ? "wait" : "other",
                                        a1_e2.t,
                                        a1,
                                        map.get_x(a2_e1.e.n),
                                        map.get_y(a2_e1.e.n),
                                        a2_e1.e.d == Direction::NORTH ? "north" :
                                        a2_e1.e.d == Direction::SOUTH ? "south" :
                                        a2_e1.e.d == Direction::EAST ? "east" :
                                        a2_e1.e.d == Direction::WEST ? "west" :
                                        a2_e1.e.d == Direction::WAIT ? "wait" : "other",
                                        a2_e1.t,
                                        map.get_x(a2_e2.e.n),
                                        map.get_y(a2_e2.e.n),
                                        a2_e2.e.d == Direction::NORTH ? "north" :
                                        a2_e2.e.d == Direction::SOUTH ? "south" :
                                        a2_e2.e.d == Direction::EAST ? "east" :
                                        a2_e2.e.d == Direction::WEST ? "west" :
                                        a2_e2.e.d == Direction::WAIT ? "wait" : "other",
                                        a2_e2.t,
                                        a2,
                                        lhs,
                                        SCIPnodeGetNumber(SCIPgetCurrentNode(scip)));

                                // Create cut.
                                SCIP_CALL(corridor_conflicts_create_cut(scip,
                                                                        probdata,
                                                                        sepa,
                                                                        a1,
                                                                        a2,
                                                                        a1_e1,
                                                                        a1_e2,
                                                                        a2_e1,
                                                                        a2_e2,
                                                                        result));
                                goto NEXT_AGENT;
                            }
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
static
SCIP_DECL_SEPACOPY(sepaCopyCorridorConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);

    // Include separator.
    SCIP_SEPA* sepa_copy;
    SCIP_CALL(SCIPincludeSepaCorridorConflicts(scip, &sepa_copy));

    // Done.
    return SCIP_OKAY;
}

// Separation method for LP solutions
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

// Create separator for corridor conflicts constraints and include it in SCIP
SCIP_RETCODE SCIPincludeSepaCorridorConflicts(
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
                                   sepaExeclpCorridorConflicts,
                                   nullptr,
                                   nullptr));
    debug_assert(*sepa);

    // Set callbacks.
    SCIP_CALL(SCIPsetSepaCopy(scip, *sepa, sepaCopyCorridorConflicts));

    // Done.
    return SCIP_OKAY;
}

#endif
