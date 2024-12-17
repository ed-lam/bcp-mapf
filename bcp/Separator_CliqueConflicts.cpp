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

#ifdef USE_CLIQUE_CONFLICTS

// #define PRINT_DEBUG

#include "Separator_CliqueConflicts.h"
#include "problem/problem.h"
#include "problem/variable_data.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-compare"
extern "C" {
#include "cliquer-1.21/cliquer.h"
}
#pragma GCC diagnostic pop

#define SEPA_NAME         "vertex_edge_clique"
#define SEPA_DESC         "Separator for clique conflicts"
#define SEPA_PRIORITY     2        // priority of the constraint handler for separation
#define SEPA_FREQ         0        // frequency for separating cuts; zero means to separate only in the root node
#define SEPA_MAXBOUNDDIST 1.0
#define SEPA_USESSUBSCIP  FALSE    // does the separator use a secondary SCIP instance? */
#define SEPA_DELAY        FALSE    // should separation method be delayed, if other separators found cuts? */

//#define RUN_ONLY_ON_MULTI_AGENT_RESOURCES
#define MIN_CLIQUE_SIZE                                                  3
#define MAX_CLIQUE_SIZE                                                  6
#define MAX_CLIQUES                                                  20000

struct CliqueItem
{
    Agent a;
    Time t;
    union
    {
        struct
        {
            Position x1;
            Position y1;
        };
        int64_t l1;
    };
    union
    {
        struct
        {
            Position x2;
            Position y2;
        };
        int64_t l2;
    };
    NodeTime nt;
    EdgeTime et;
    SCIP_Real val;

    static_assert(2 * sizeof(Position) == sizeof(int64_t));

    inline bool is_vertex() const
    {
        return l2 == -1;
    }
    inline bool is_edge() const
    {
        return !is_vertex();
    }
};

inline bool vertex_collision(const int64_t l1, const Time t1, const int64_t l2, const Time t2)
{
    debug_assert(l1 != -1);
    debug_assert(l2 != -1);
    return l1 == l2 && t1 == t2;
}

inline bool edge_collision(const int64_t o1, const int64_t d1, const Time t1,
                           const int64_t o2, const int64_t d2, const Time t2)
{
    debug_assert(o1 != -1);
    debug_assert(d1 != -1);
    debug_assert(o2 != -1);
    debug_assert(d2 != -1);
    return o1 == d2 && d1 == o2 && t1 == t2;
}

inline bool time_incompatible(const Position x1, const Position y1, const Time t1,
                              const Position x2, const Position y2, const Time t2)
{
    const auto time_diff = abs(t2 - t1);
    const auto dist_diff = abs(x2 - x1) + abs(y2 - y1);
    return dist_diff > time_diff;
}

struct CliquerUserData
{
    SCIP* scip;
    SCIP_SEPA* sepa;
    SCIP_RESULT* result;

    const Vector<CliqueItem>& items;
    size_t nb_cliques;
};

bool incompatible(const CliqueItem item1, const CliqueItem item2)
{
    if (item1.a == item2.a)
    {
        if (item1.is_vertex() && item2.is_vertex())
        {
            return time_incompatible(item1.x1, item1.y1, item1.t, item2.x1, item2.y1, item2.t);
        }
        else if (item1.is_vertex() && item2.is_edge())
        {
            return time_incompatible(item1.x1, item1.y1, item1.t, item2.x1, item2.y1, item2.t    ) &&
                   time_incompatible(item1.x1, item1.y1, item1.t, item2.x2, item2.y2, item2.t + 1);
        }
        else if (item1.is_edge() && item2.is_vertex())
        {
            return time_incompatible(item1.x1, item1.y1, item1.t,     item2.x1, item2.y1, item2.t) &&
                   time_incompatible(item1.x2, item1.y2, item1.t + 1, item2.x1, item2.y1, item2.t);
        }
        else if (item1.is_edge() && item2.is_edge())
        {
            if (item1.t < item2.t)
            {
                return time_incompatible(item1.x2, item1.y2, item1.t + 1, item2.x1, item2.y1, item2.t);
            }
            else if (item1.t > item2.t)
            {
                return time_incompatible(item2.x2, item2.y2, item2.t + 1, item1.x1, item1.y1, item1.t);
            }
            else
            {
                return item1.l1 != item2.l1 || item1.l2 != item2.l2;
            }
        }
        else
        {
            unreachable();
        }
    }
    else
    {
        if (item1.is_vertex() && item2.is_vertex())
        {
            return vertex_collision(item1.l1, item1.t, item2.l1, item2.t);
        }
        else if (item1.is_vertex() && item2.is_edge())
        {
            return vertex_collision(item1.l1, item1.t, item2.l1, item2.t    ) ||
                   vertex_collision(item1.l1, item1.t, item2.l2, item2.t + 1);
        }
        else if (item1.is_edge() && item2.is_vertex())
        {
            return vertex_collision(item1.l1, item1.t,     item2.l1, item2.t) ||
                   vertex_collision(item1.l2, item1.t + 1, item2.l1, item2.t);
        }
        else if (item1.is_edge() && item2.is_edge())
        {
            return vertex_collision(item1.l1, item1.t,     item2.l1, item2.t    ) ||
                   vertex_collision(item1.l2, item1.t + 1, item2.l1, item2.t    ) ||
                   vertex_collision(item1.l1, item1.t,     item2.l2, item2.t + 1) ||
                   vertex_collision(item1.l2, item1.t + 1, item2.l2, item2.t + 1) ||
                   edge_collision(item1.l1, item1.l2, item1.t, item2.l1, item2.l2, item2.t);
        }
        else
        {
            unreachable();
        }
    }
}

SCIP_RETCODE clique_conflicts_create_cut(
    SCIP* scip,                                // SCIP
    SCIP_ProbData* probdata,                   // Problem data
    SCIP_SEPA* sepa,                           // Separator
    const Agent a1,                            // Agent 1
    const Agent a2,                            // Agent 2
    const Vector<CliqueItem>& clique_items,    // Items in the clique
    SCIP_Result* result                        // Output result
)
{
    // Get the map.
    const auto& map = SCIPprobdataGetMap(probdata);

    // Get the edges representing the items in the clique.
    Vector<EdgeTime> ets1;
    Vector<EdgeTime> ets2;
    for (const auto& item : clique_items)
    {
        auto& ets = item.a == a1 ? ets1 : ets2;
        if (item.is_vertex())
        {
            const auto nt = item.nt;
            const auto prev_time = nt.t - 1;
            ets.emplace_back(map.get_south(nt.n), Direction::NORTH, prev_time);
            ets.emplace_back(map.get_north(nt.n), Direction::SOUTH, prev_time);
            ets.emplace_back(map.get_west(nt.n), Direction::EAST, prev_time);
            ets.emplace_back(map.get_east(nt.n), Direction::WEST, prev_time);
            ets.emplace_back(map.get_wait(nt.n), Direction::WAIT, prev_time);
        }
        else
        {
            ets.emplace_back(item.et);
        }
    }

    // Create constraint name.
#ifdef DEBUG
    String str1;
    String str2;
    for (auto it = clique_items.begin(); it != clique_items.end(); ++it)
    {
        const auto& item = *it;
        auto& str = item.a == a1 ? str1 : str2;
        if (item.is_vertex())
        {
            if (!str.empty())
            {
                str += ", ";
            }
            str += fmt::format("(({},{}),{})", item.x1, item.y1, item.t);
        }
        else
        {
            if (!str.empty())
            {
                str += ", ";
            }
            str += fmt::format("(({},{}),({},{}),{})", item.x1, item.y1, item.x2, item.y2, item.t);
        }
    }
    auto name = fmt::format("clique_conflict({},{},({}),({}))", a1, a2, str1, str2);
#endif

    // Create data for the cut.
    TwoAgentRobustCut cut(scip,
                          a1,
                          a2,
                          ets1.size(),
                          ets2.size()
#ifdef DEBUG
                        , std::move(name)
#endif
    );
    for (Int idx = 0; idx < static_cast<Int>(ets1.size()); ++idx)
    {
        cut.a1_edge_time(idx) = ets1[idx];
    }
    for (Int idx = 0; idx < static_cast<Int>(ets2.size()); ++idx)
    {
        cut.a2_edge_time(idx) = ets2[idx];
    }

    // Store the cut.
    SCIP_CALL(SCIPprobdataAddTwoAgentRobustCut(scip, probdata, sepa, std::move(cut), 1, result));

    // Done.
    return SCIP_OKAY;
}

boolean cliquer_callback(set_t clique, graph_t*, clique_options* opts)
{
    // Get SCIP data.
    const auto& user_data = static_cast<CliquerUserData*>(opts->user_data);
    auto scip = user_data->scip;
    auto sepa = user_data->sepa;
    auto result = user_data->result;

    // Get conflict graph data.
    const auto& items = user_data->items;
    auto& nb_cliques = user_data->nb_cliques;

    // Get problem data.
    auto probdata = SCIPgetProbData(scip);
    const auto N = SCIPprobdataGetN(probdata);

    // Get the clique and the compute the LHS.
    debug_assert(clique);
    SCIP_Real lhs = 0;
    Vector<CliqueItem> clique_items;
    {
        int i = -1;
        while ((i = set_return_next(clique, i)) >= 0)
        {
            const auto& item = items[i];
            clique_items.push_back(item);
            lhs += item.val;
        }
    }

    // Add a cut if violated.
    if (SCIPisSumGT(scip, lhs, 1.0 + CUT_VIOLATION))
    {
        // Print.
#ifdef PRINT_DEBUG
        debugln("Clique LHS = {}", lhs);
        for (const auto& item : clique_items)
        {
            if (item.is_vertex())
            {
                debugln("    Agent {}: (({},{}),{}) {}",
                        item.a, item.x1, item.y1, item.t, item.val);
            }
            else
            {
                debugln("    Agent {}: (({},{}),({},{}),{}) {}",
                        item.a, item.x1, item.y1, item.x2, item.y2, item.t, item.val);
            }
        }
#endif

        // Count the number of agents in clique.
        Vector<Agent> agents;
        {
            Vector<bool> agent_used(N);
            for (const auto& item : clique_items)
            {
                agent_used[item.a] = true;
            }
            for (Agent a = 0; a < static_cast<Agent>(agent_used.size()); ++a)
                if (agent_used[a])
                {
                    agents.push_back(a);
                }
        }

        // Create a cut.
        if (agents.size() == 2)
        {
#ifdef PRINT_DEBUG
            {
                String str1;
                String str2;
                for (auto it = clique_items.begin(); it != clique_items.end(); ++it)
                {
                    const auto& item = *it;
                    auto& str = item.a == agents[0] ? str1 : str2;
                    if (item.is_vertex())
                    {
                        if (!str.empty())
                        {
                            str += ", ";
                        }
                        str += fmt::format("(({},{}),{})", item.x1, item.y1, item.t);
                    }
                    else
                    {
                        if (!str.empty())
                        {
                            str += ", ";
                        }
                        str += fmt::format("(({},{}),({},{}),{})", item.x1, item.y1, item.x2, item.y2, item.t);
                    }
                }
                debugln("   Creating clique conflict cut on {} for agent {} and {} for agent {} with value {} in "
                        "branch-and-bound node {}",
                        str1,
                        agents[0],
                        str2,
                        agents[1],
                        lhs,
                        SCIPnodeGetNumber(SCIPgetCurrentNode(scip)));
            }
#endif

            // Create cut.
            SCIP_CALL(clique_conflicts_create_cut(scip, probdata, sepa, agents[0], agents[1], clique_items, result));
        }
//#ifdef DEBUG
        else if (agents.size() > 2)
        {
            println("   Multi-agent clique cuts not yet implemented");
        }
//#endif
        nb_cliques++;
        return nb_cliques >= MAX_CLIQUES;
    }
    else
    {
        return true;
    }
};

// Separator
static
SCIP_RETCODE clique_conflicts_separate(
    SCIP* scip,            // SCIP
    SCIP_SEPA* sepa,       // Separator
    SCIP_RESULT* result    // Pointer to store the result of the separation call
)
{
    // Print.
    debugln("Starting separator for clique conflicts on solution with obj {:.6f}:",
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

    // Get variables.
    const auto& vars = SCIPprobdataGetVars(probdata);

    // Calculate the number of times a vertex or edge is used by summing the columns.
    HashTable<AgentNodeTime, SCIP_Real> vertex_val;
    HashTable<AgentEdgeTime, SCIP_Real> edge_val;
    HashTable<NodeTime, Vector<bool>> vertex_agents;
    HashTable<EdgeTime, Vector<bool>> edge_agents;
    for (const auto& [var, var_val] : vars)
    {
        // Get the path.
        debug_assert(var);
        auto vardata = SCIPvarGetData(var);
        const auto a = SCIPvardataGetAgent(vardata);
        const auto path_length = SCIPvardataGetPathLength(vardata);
        const auto path = SCIPvardataGetPath(vardata);

        // Sum variable values.
        debug_assert(var_val == SCIPgetSolVal(scip, nullptr, var));
        if (!SCIPisIntegral(scip, var_val))
        {
            for (Time t = 1; t < path_length; ++t)
            {
                const AgentNodeTime ant{a, path[t].n, t};
                vertex_val[ant] += var_val;

                const NodeTime nt{path[t].n, t};
                auto [it, _] = vertex_agents.emplace(std::piecewise_construct,
                                                     std::forward_as_tuple(nt),
                                                     std::forward_as_tuple(N));
                it->second[a] = true;
            }
            for (Time t = 0; t < path_length - 1; ++t)
            {
                const AgentEdgeTime aet{a, path[t], t};
                edge_val[aet] += var_val;

                const EdgeTime et{path[t], t};
                auto [it, _] = edge_agents.emplace(std::piecewise_construct,
                                                   std::forward_as_tuple(et),
                                                   std::forward_as_tuple(N));
                it->second[a] = true;
            }
        }
    }

    // Remove integral vertices and edges.
    for (auto it = vertex_val.begin(); it != vertex_val.end();)
    {
        const auto [ant, val] = *it;
        if (SCIPisIntegral(scip, val))
        {
            debug_assert(vertex_agents.find(NodeTime{ant.n, ant.t}) != vertex_agents.end());
            vertex_agents.erase(vertex_agents.find(NodeTime{ant.n, ant.t}));
            it = vertex_val.erase(it);
        }
        else
        {
            ++it;
        }
    }
    for (auto it = edge_val.begin(); it != edge_val.end();)
    {
        const auto [aet, val] = *it;
        if (SCIPisIntegral(scip, val))
        {
            debug_assert(edge_agents.find(EdgeTime{aet.e, aet.t}) != edge_agents.end());
            edge_agents.erase(edge_agents.find(EdgeTime{aet.e, aet.t}));
            it = edge_val.erase(it);
        }
        else
        {
            ++it;
        }
    }

    // Create the data for each node in the conflict graph.
    Vector<CliqueItem> items;
    for (const auto [nt, agents] : vertex_agents)
    {
#ifdef RUN_ONLY_ON_MULTI_AGENT_RESOURCES
        Agent nb_agents = 0;
        for (const auto used : agents)
        {
            nb_agents += used;
        }
        if (nb_agents > 1)
#endif
        {
            for (Agent a = 0; a < N; ++a)
                if (agents[a])
                {
                    auto& item = items.emplace_back();
                    item.a = a;
                    item.t = nt.t;
                    std::tie(item.x1, item.y1) = map.get_xy(nt.n);
                    item.l2 = -1;
                    item.nt = nt;
                    debug_assert(vertex_val.find({a, nt.n, nt.t}) != vertex_val.end());
                    item.val = vertex_val.find({a, nt.n, nt.t})->second;
                }
        }
    }
    for (const auto [et, agents] : edge_agents)
    {
#ifdef RUN_ONLY_ON_MULTI_AGENT_RESOURCES
        Agent nb_agents = 0;
        for (const auto used : agents)
        {
            nb_agents += used;
        }
        if (nb_agents > 1)
#endif
        {
            for (Agent a = 0; a < N; ++a)
                if (agents[a])
                {
                    auto& item = items.emplace_back();
                    item.a = a;
                    item.t = et.t;
                    std::tie(item.x1, item.y1) = map.get_xy(et.n);
                    std::tie(item.x2, item.y2) = map.get_destination_xy(et);
                    item.et = et;
                    debug_assert(edge_val.find({a, et.et.e, et.t}) != edge_val.end());
                    item.val = edge_val.find({a, et.et.e, et.t})->second;
                }
        }
    }

    // Find cliques.
    if (!items.empty())
    {
        // Create the conflict graph.
        auto graph = graph_new(items.size());
        for (size_t i = 0; i < items.size(); ++i)
        {
            graph->weights[i] = items[i].val * 100;

            for (size_t j = i + 1; j < items.size(); ++j)
                if (incompatible(items[i], items[j]))
                {
                    GRAPH_ADD_EDGE(graph, i, j);
                }
        }

        // Run Cliquer.
        CliquerUserData user_data{scip, sepa, result, items, 0};
        clique_default_options->user_data = static_cast<void*>(&user_data);
        clique_default_options->user_function = cliquer_callback;
        clique_default_options->time_function = nullptr;
//        clique_unweighted_find_all(graph, MIN_CLIQUE_SIZE, MAX_CLIQUE_SIZE, false, nullptr);
        clique_find_all(graph, 1.1 * 100.0, 0, false, nullptr);
    }

    // Done.
    return SCIP_OKAY;
}

// Copy method for separator
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_SEPACOPY(sepaCopyCliqueConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);

    // Include separator.
    SCIP_CALL(SCIPincludeSepaCliqueConflicts(scip));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Separation method for LP solutions
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_SEPAEXECLP(sepaExeclpCliqueConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);
    debug_assert(result);

    // Start.
    *result = SCIP_DIDNOTFIND;

    // Start separator.
    SCIP_CALL(clique_conflicts_separate(scip, sepa, result));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Create separator for clique conflicts constraints and include it in SCIP
SCIP_RETCODE SCIPincludeSepaCliqueConflicts(
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
                                   sepaExeclpCliqueConflicts,
                                   nullptr,
                                   nullptr));
    debug_assert(sepa);

    // Set callbacks.
    SCIP_CALL(SCIPsetSepaCopy(scip, sepa, sepaCopyCliqueConflicts));

    // Done.
    return SCIP_OKAY;
}

#endif
