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

//#define PRINT_DEBUG

#include "Separator_ExitEntryConflicts.h"
#include "Coordinates.h"
#include "ProblemData.h"
#include "VariableData.h"
#include "ConstraintHandler_EdgeConflicts.h"

#define SEPA_NAME                         "exitentry_conflicts"
#define SEPA_DESC          "Separator for exit-entry conflicts"
#define SEPA_PRIORITY                                   +550000 // priority of the constraint handler for separation
#define SEPA_FREQ                                             1 // frequency for separating cuts; zero means to separate only in the root node
#define SEPA_MAXBOUNDDIST                                   1.0
#define SEPA_USESSUBSCIP                                  FALSE // does the separator use a secondary SCIP instance? */
#define SEPA_DELAY                                        FALSE // should separation method be delayed, if other separators found cuts? */

SCIP_RETCODE exitentry_conflicts_create_cut(
    SCIP* scip,                  // SCIP
    SCIP_ProbData* probdata,     // Problem data
    SCIP_SEPA* sepa,             // Separator
    const Agent a1,              // Agent 1
    const Agent a2,              // Agent 2
    const Edge a1_e,             // Edge-time of agent 1
    const Vector<Edge> a2_es,    // Edge-times of agent 2
    const Time t,                // Time
    SCIP_Result* result          // Output result
)
{
    // Create constraint name.
#ifdef DEBUG
    auto name = fmt::format("exitentry_conflict({},{},{})", t, a1, a2);
#endif

    // Create data for the cut.
    TwoAgentRobustCut cut{scip, a1, a2, t, 1, static_cast<Int>(a2_es.size())
#ifdef DEBUG
        , std::move(name)
#endif
    };
    cut.edges_a1(0) = a1_e;
    std::copy(a2_es.begin(), a2_es.end(), cut.edges_a2().first);

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

    // Get the edges fractionally used by each agent.
    const auto& agent_edges = get_agent_fractional_edges_no_waits(scip);

    // Find conflicts.
    for (Agent a1 = 0; a1 < N; ++a1)
    {
        // Get the edges of agent 1.
        const auto& agent_edges_a1 = agent_edges[a1];

        // Loop through the second agent.
        for (Agent a2 = 0; a2 < N; ++a2)
            if (a2 != a1)
            {
                // Get the edges of agent 2.
                const auto& agent_edges_a2 = agent_edges[a2];

                // Loop through all edges of agent 1.
                for (const auto [a1_et, a1_et_val] : agent_edges_a1)
                {
                    // Get the vertices of the edge.
                    const auto t = a1_et.t;
                    const auto a1_e = a1_et.et.e;
                    const auto n1 = a1_e.n;
                    const auto n2 = a1_e.d == Direction::NORTH ? map.get_north(n1) :
                                    a1_e.d == Direction::SOUTH ? map.get_south(n1) :
                                    a1_e.d == Direction::EAST  ? map.get_east(n1)  :
                                    a1_e.d == Direction::WEST  ? map.get_west(n1)  :
                                                                 map.get_wait(n1);

                    // Make the incompatible edges for agent 2.
                    Vector<Edge> a2_es{Edge(n1, Direction::NORTH),
                                       Edge(n1, Direction::SOUTH),
                                       Edge(n1, Direction::EAST),
                                       Edge(n1, Direction::WEST),
                                       Edge(n1, Direction::WAIT),

                                       Edge(map.get_south(n2), Direction::NORTH),
                                       Edge(map.get_north(n2), Direction::SOUTH),
                                       Edge(map.get_west(n2), Direction::EAST),
                                       Edge(map.get_east(n2), Direction::WEST),
                                       Edge(map.get_wait(n2), Direction::WAIT),

                                       get_opposite_edge_allow_wait(a1_e, map)};

                    // Remove duplicates.
                    for (auto it = a2_es.begin(); it != a2_es.end() - 1; ++it)
                        for (auto it2 = it + 1; it2 != a2_es.end();)
                        {
                            if (*it == *it2)
                            {
                                it2 = a2_es.erase(it2);
                            }
                            else
                            {
                                ++it2;
                            }
                        }

                    // Compute the LHS.
                    SCIP_Real lhs = a1_et_val;
                    for (const auto e : a2_es)
                    {
                        auto it = agent_edges_a2.find(EdgeTime(e, t));
                        if (it != agent_edges_a2.end())
                        {
                            const auto a2_et_val = it->second;
                            lhs += a2_et_val;
                        }
                    }

                    // Create a cut if violated.
                    if (SCIPisGT(scip, lhs, 1.0))
                    {
                        // Print.
                        debugln("   Creating exit-entry conflict cut on edge "
                                "(({},{}),({},{})) for agents {} and {} at time {} "
                                "with value {} in branch-and-bound node {}",
                                map.get_x(n1), map.get_y(n1),
                                map.get_x(n2), map.get_y(n2),
                                a1,
                                a2,
                                t,
                                lhs,
                                SCIPnodeGetNumber(SCIPgetCurrentNode(scip)));

                        // Create cut.
                        SCIP_CALL(exitentry_conflicts_create_cut(scip,
                                                                 probdata,
                                                                 sepa,
                                                                 a1,
                                                                 a2,
                                                                 a1_e,
                                                                 a2_es,
                                                                 t,
                                                                 result));
                        goto NEXT_AGENT;
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
SCIP_DECL_SEPACOPY(sepaCopyExitEntryConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);

    // Include separator.
    SCIP_SEPA* sepa_copy;
    SCIP_CALL(SCIPincludeSepaExitEntryConflicts(scip, &sepa_copy));

    // Done.
    return SCIP_OKAY;
}

// Separation method for LP solutions
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

// Create separator for exit-entry conflicts constraints and include it in SCIP
SCIP_RETCODE SCIPincludeSepaExitEntryConflicts(
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
                                   sepaExeclpExitEntryConflicts,
                                   nullptr,
                                   nullptr));
    debug_assert(*sepa);

    // Set callbacks.
    SCIP_CALL(SCIPsetSepaCopy(scip, *sepa, sepaCopyExitEntryConflicts));

    // Done.
    return SCIP_OKAY;
}

#endif
