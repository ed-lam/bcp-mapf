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

//#define PRINT_DEBUG

#include "BranchingRule.h"
#include "Includes.h"
#include "ProblemData.h"
#include "VariableData.h"
#ifdef USE_RECTANGLE_KNAPSACK_CONFLICTS
#include "Separator_RectangleKnapsackConflicts.h"
#endif
#ifdef USE_RECTANGLE_CLIQUE_CONFLICTS
#include "Separator_RectangleCliqueConflicts.h"
#endif
#include <numeric>

struct Score
{
    Agent a;
    SCIP_Real val;
    Time shortest_path_length;
};

struct SuccessorDirection
{
    bool has_move{false};
    bool has_wait{false};
};

struct GoalTimeBound
{
    Node n;
    Time first;
    Time last;
};

Tuple<HashTable<NodeTime, Pair<Vector<Score>, Vector<Score>>>,
      HashTable<AgentTime, SuccessorDirection>,
      Vector<Int>,
      Vector<GoalTimeBound>>
get_lp_branch_candidates(
    SCIP* scip,                 // SCIP
    SCIP_PROBDATA* probdata,    // Problem data
    const Agent N               // Number of agents
)
{
    // Create output.
    Tuple<HashTable<NodeTime, Pair<Vector<Score>, Vector<Score>>>,
          HashTable<AgentTime, SuccessorDirection>,
          Vector<Int>,
          Vector<GoalTimeBound>> output;
    auto& [candidates, succ_dirs, nb_paths, goal_time_bounds] = output;

    // Get candidate variables.
    SCIP_VAR** candidate_vars;
    SCIP_Real* candidate_var_vals;
    int nb_candidate_vars;
    scip_assert(SCIPgetLPBranchCands(scip,
                                     &candidate_vars,
                                     nullptr,
                                     &candidate_var_vals,
                                     &nb_candidate_vars,
                                     nullptr,
                                     nullptr));
    debug_assert(nb_candidate_vars > 0);

    // Find the makespan.
    Time makespan = 0;
    for (Int v = 0; v < nb_candidate_vars; ++v)
    {
        // Get the variable.
        auto var = candidate_vars[v];
        debug_assert(var);

        // Proceed if not artificial variable.
        auto vardata = SCIPvarGetData(var);
        if (vardata)
        {
            // Get the path length.
            const auto path_length = SCIPvardataGetPathLength(vardata);

            // Store the length of the longest path.
            if (path_length > makespan)
                makespan = path_length;
        }
    }

    // Calculate branching candidates.
    nb_paths.resize(N);
    for (Int v = 0; v < nb_candidate_vars; ++v)
    {
        // Get the variable.
        auto var = candidate_vars[v];
        debug_assert(var);

        // Proceed if not artificial variable.
        auto vardata = SCIPvarGetData(var);
        if (vardata)
        {
            // Get the path.
            const auto a = SCIPvardataGetAgent(vardata);
            const auto path_length = SCIPvardataGetPathLength(vardata);
            const auto path = SCIPvardataGetPath(vardata);

            // Get the variable value.
            const auto var_val = candidate_var_vals[v];

            // Update candidates data.
            nb_paths[a]++;
            for (Time t = 1; t < path_length; ++t)
            {
                // Store the candidate vertex.
                const NodeTime nt{path[t].n, t};
                auto& scores = candidates[nt].first;
                auto it = std::find_if(scores.begin(),
                                       scores.end(),
                                       [a](const Score& score){ return score.a == a; });
                if (it == scores.end())
                {
                    scores.push_back({a, var_val, path_length});
                }
                else
                {
                    auto& score = *it;
                    score.val += var_val;
                    if (path_length < score.shortest_path_length)
                        score.shortest_path_length = path_length;
                }

                // Store whether the vertex is reached by waiting.
                auto& prev = succ_dirs[AgentTime{{a, t - 1}}];
                prev.has_move |= (path[t - 1].d != Direction::WAIT);
                prev.has_wait |= (path[t - 1].d == Direction::WAIT);
            }
        }
    }
    release_assert(!candidates.empty(), "No candidates for branching in fractional LP");

    // Separate integral and fractional vertices.
    for (auto it = candidates.begin(); it != candidates.end();)
    {
        // Sum the vertex weight.
        auto& scores = it->second;
        const auto val = std::accumulate(scores.first.begin(),
                                         scores.first.end(),
                                         0.0,
                                         [](const SCIP_Real val, const Score& score)
                                         {
                                             return val + score.val;
                                         });

        // Move to list of fractional vertices.
        debug_assert(scores.first.size() >= 1);
        if (!SCIPisIntegral(scip, val))
        {
            // Fractional.
            scores.second = move(scores.first);
            ++it;
        }
        else if (scores.first.size() == 1)
        {
            // Integral and used by one agent.
            it = candidates.erase(it);
        }
        else
        {
            // Integral and used by more than one agent.
            ++it;
        }
    }

    // Print.
//#ifdef PRINT_DEBUG
//    {
//        auto& gm = SCIPprobdataGetGridMap(SCIPgetProbData(scip));
//
//        debugln("   Branching candidates:");
//        for (const auto& [nt, scores] : candidates)
//        {
//            const auto [x, y] = map.get_xy(nt.n);
//            for (const auto& [a, val, shortest_path_length] : scores.first)
//            {
//                debugln("      ({},({},{}),{}) {} {:.4f}",
//                        a, x, y, nt.t, shortest_path_length, val);
//            }
//            for (const auto& [a, val, shortest_path_length] : scores.second)
//            {
//                debugln("   ---({},({},{}),{}) {} {:.4f}",
//                        a, x, y, nt.t, shortest_path_length, val);
//            }
//        }
//    }
//#endif

    // Get variables.
    const auto& vars = SCIPprobdataGetVars(probdata);

    // Find the earliest and latest time an agent reaches its goal.
    goal_time_bounds.resize(N, {0, std::numeric_limits<Time>::max(), 0});
    for (auto var : vars)
    {
        // Get the path.
        debug_assert(var);
        auto vardata = SCIPvarGetData(var);
        const auto a = SCIPvardataGetAgent(vardata);
        const auto path_length = SCIPvardataGetPathLength(vardata);
        const auto path = SCIPvardataGetPath(vardata);

        // Get the variable value.
        const auto var_val = SCIPgetSolVal(scip, nullptr, var);

        // Store goal.
        goal_time_bounds[a].n = path[path_length - 1].n;

        // Sum.
        const auto t = path_length - 1;
        if (SCIPisPositive(scip, var_val))
        {
            if (t < goal_time_bounds[a].first)
            {
                goal_time_bounds[a].first = t;
            }
            if (t > goal_time_bounds[a].last)
            {
                goal_time_bounds[a].last = t;
            }
        }
    }

    // Return.
    return output;
}

Pair<AgentNodeTime, bool> find_decision_early_goal(
    SCIP* scip,                                      // SCIP
    SCIP_PROBDATA* probdata,                         // Problem data
    const Agent N,                                   // Number of agents
    const Vector<GoalTimeBound>& goal_time_bounds    // Earliest and latest time an agent reaches its goal
)
{
    // Create output.
    AgentNodeTime best_ant{-1, 0, std::numeric_limits<Time>::max()};
    bool prefer_branch_0 = false;
    Time best_diff = 1;

    // Get agent variables.
#ifdef DEBUG
    const auto& agent_vars = SCIPprobdataGetAgentVars(probdata);
#endif

    // Select an agent involved in a rectangle clique conflict.
#ifdef USE_RECTANGLE_CLIQUE_CONFLICTS
    {
        const auto& rectangle_clique_conflicts_conss =
            rectangle_clique_conflicts_get_constraints(probdata);
        SCIP_Real best_lhs = 1.0 - 0.99;
        for (const auto& conflict : rectangle_clique_conflicts_conss)
        {
            // Get variables in this row.
            auto nvars = SCIProwGetNNonz(conflict.row);
            auto cols = SCIProwGetCols(conflict.row);
            auto coeffs = SCIProwGetVals(conflict.row);

            // Check.
#ifdef DEBUG
            SCIP_Real tmp = 0.0;
            for (auto var :  agent_vars[conflict.a1])
            {
                debug_assert(var);
                auto vardata = SCIPvarGetData(var);
                const auto a = SCIPvardataGetAgent(vardata);
                const auto path_length = SCIPvardataGetPathLength(vardata);
                const auto path = SCIPvardataGetPath(vardata);

                const auto var_val = SCIPgetSolVal(scip, nullptr, var);

                SCIP_Real count = 0;
                for (auto [it, end] = conflict.agent_edges(a); it != end; ++it)
                {
                    const auto e = it->et.e;
                    const auto t = it->t;
                    if (t < path_length - 1 && path[t] == e)
                    {
                        count++;
                    }
                }
                debug_assert(count <= 2);
                if (count == 2)
                {
                    tmp += var_val;
                }
            }
            for (auto var : agent_vars[conflict.a2])
            {
                debug_assert(var);
                auto var = agent_vars[conflict.a2][v];
                auto vardata = SCIPvarGetData(var);
                const auto a = SCIPvardataGetAgent(vardata);
                const auto path_length = SCIPvardataGetPathLength(vardata);
                const auto path = SCIPvardataGetPath(vardata);

                const auto var_val = SCIPgetSolVal(scip, nullptr, var);

                SCIP_Real count = 0.0;
                for (auto [it, end] = conflict.agent_edges(a); it != end; ++it)
                {
                    const auto e = it->et.e;
                    const auto t = it->t;
                    if (t < path_length - 1 && path[t] == e)
                    {
                        count++;
                    }
                }
                debug_assert(count <= 2);
                if (count == 2)
                {
                    tmp += var_val;
                }
            }
#endif

            // Compute LHS.
            SCIP_Real lhs = 0.0;
            for (auto v = 0; v < nvars; ++v)
            {
                // Check.
                debug_assert(coeffs[v] == 1 || coeffs[v] == 2);

                // Sum.
                auto var = SCIPcolGetVar(cols[v]);
                const auto var_val = SCIPgetSolVal(scip, nullptr, var);
                lhs += var_val * coeffs[v];
            }
            debug_assert(SCIPisEQ(scip, lhs, tmp));

            // Skip if the constraint is not binding.
            if (SCIPisSumLT(scip, lhs, best_lhs))
                continue;

            // Loop through all columns of the row.
            for (auto v = 0; v < nvars; ++v)
            {
                // Get the variable value.
                auto var = SCIPcolGetVar(cols[v]);
                const auto var_val = SCIPgetSolVal(scip, nullptr, var);

                // Find a wait.
                if (SCIPisPositive(scip, var_val))
                {
                    // Get the agent.
                    auto vardata = SCIPvarGetData(var);
                    const auto a = SCIPvardataGetAgent(vardata);

                    // Prefer agents with the earliest finish time.
                    const auto earliest_finish = goal_time_bounds[a].first;
                    if (earliest_finish > best_ant.t)
                        continue;

                    // Prefer agents with large difference in path lengths.
                    const auto diff = goal_time_bounds[a].last - goal_time_bounds[a].first;
                    if (diff < best_diff)
                        continue;

                    // Accept the decision.
                    best_lhs = lhs;
                    best_ant = {a, goal_time_bounds[a].n, earliest_finish};
                    best_diff = diff;
                }
            }
        }
        if (best_ant.a >= 0)
        {
            debugln("   Selected decision as splitting agent in rectangle clique "
                    "conflict");
            return {best_ant, prefer_branch_0};
        }
    }
#endif

    // Select an agent involved in a rectangle knapsack conflict.
#ifdef USE_RECTANGLE_KNAPSACK_CONFLICTS
    const auto& rectangle_knapsack_cuts = rectangle_knapsack_get_cuts(probdata);
    const auto& two_agent_robust_cuts = SCIPprobdataGetTwoAgentRobustCuts(probdata);
    {
        SCIP_Real best_lhs = 3.0 - 0.99;
        for (const auto [idx, out1_begin, in2_begin, out2_begin] : rectangle_knapsack_cuts)
        {
            // Get the cut.
            debug_assert(idx < static_cast<Int>(two_agent_robust_cuts.size()));
            const auto& cut = two_agent_robust_cuts[idx];

            // Get variables in this row.
            auto nvars = SCIProwGetNNonz(cut.row());
            auto cols = SCIProwGetCols(cut.row());
            auto coeffs = SCIProwGetVals(cut.row());

            // Check.
#ifdef DEBUG
            SCIP_Real tmp = 0.0;
            for (const auto a : Array<Agent, 2>{cut.a1(), cut.a2()})
                for (auto var : agent_vars[a])
                {
                    debug_assert(var);
                    auto vardata = SCIPvarGetData(var);
                    const auto path_length = SCIPvardataGetPathLength(vardata);
                    const auto path = SCIPvardataGetPath(vardata);

                    const auto var_val = SCIPgetSolVal(scip, nullptr, var);

                    SCIP_Real coeff = 0;
                    for (auto [it, end] = cut.edge_times(a); it != end; ++it)
                    {
                        const auto e = it->et.e;
                        const auto t = it->t;
                        coeff += (t < path_length - 1 && path[t] == e);
                    }
                    debug_assert(coeff <= 2);
                    tmp += coeff * var_val;
                }
#endif

            // Compute LHS.
            SCIP_Real lhs = 0.0;
            for (auto v = 0; v < nvars; ++v)
            {
                // Check.
                debug_assert(coeffs[v] == 1 || coeffs[v] == 2);

                // Sum.
                auto var = SCIPcolGetVar(cols[v]);
                debug_assert(var);
                const auto var_val = SCIPgetSolVal(scip, nullptr, var);
                lhs += var_val * coeffs[v];
            }
            debug_assert(SCIPisEQ(scip, lhs, tmp));

            // Skip if the constraint is not binding.
            if (SCIPisSumLT(scip, lhs, best_lhs))
                continue;

            // Loop through all columns of the row.
            for (auto v = 0; v < nvars; ++v)
            {
                // Get the variable value.
                auto var = SCIPcolGetVar(cols[v]);
                debug_assert(var);
                const auto var_val = SCIPgetSolVal(scip, nullptr, var);

                // Find a wait.
                if (SCIPisPositive(scip, var_val))
                {
                    // Get the agent.
                    auto vardata = SCIPvarGetData(var);
                    const auto a = SCIPvardataGetAgent(vardata);

                    // Prefer agents with the earliest finish time.
                    const auto earliest_finish = goal_time_bounds[a].first;
                    if (earliest_finish > best_ant.t)
                        continue;

                    // Prefer agents with large difference in path lengths.
                    const auto diff = goal_time_bounds[a].last - goal_time_bounds[a].first;
                    if (diff < best_diff)
                        continue;

                    // Accept the decision.
                    best_lhs = lhs;
                    best_ant = {a, goal_time_bounds[a].n, earliest_finish};
                    best_diff = diff;
                }
            }
        }
        if (best_ant.a >= 0)
        {
            debugln("   Selected decision as splitting agent in rectangle knapsack "
                    "conflict");
            return {best_ant, prefer_branch_0};
        }
    }
#endif

    // Select any other agent.
    best_ant.t = std::numeric_limits<Time>::max();
    best_diff = 0;
    for (Agent a = 0; a < N; ++a)
    {
        const auto earliest_finish = goal_time_bounds[a].first;
        const auto diff = goal_time_bounds[a].last - goal_time_bounds[a].first;
        if (diff >= 1 &&
            ((earliest_finish <  best_ant.t) ||
             (earliest_finish == best_ant.t && diff > best_diff)))
        {
            best_ant = {a, goal_time_bounds[a].n, earliest_finish};
            best_diff = diff;
        }
    }
    if (best_ant.a >= 0)
    {
        prefer_branch_0 = true;
        debugln("   Selected decision as splitting agent not in rectangle conflict");
    }

    // Return.
    return {best_ant, prefer_branch_0};
}

//Pair<AgentTime, bool> find_decision_wait(
//    SCIP* scip,                                             // SCIP
//    SCIP_PROBDATA* probdata,                                // Problem data
//    const HashTable<AgentTime, SuccessorDirection>& succ_dirs,    // Vertices with a wait
//    const Vector<GoalTimeBound>& goal_time_bounds           // Earliest and latest time an agent reaches its goal
//)
//{
//    // Create output.
//    AgentTime best_at{-1, std::numeric_limits<Time>::max()};
//    bool prefer_branch_0 = false;
//
//    // Get agent variables.
//    auto agent_nvars = SCIPprobdataGetAgentNVars(probdata);
//    auto agent_vars = SCIPprobdataGetAgentVars(probdata);
//
//    // Get rectangle constraints.
//    const auto& rectangle_conflicts_conss = rectangle_conflicts_get_constraints(probdata);
//
//    // Find a wait decision.
//    SCIP_Real best_lhs = 2.01;
//    Time best_path_length = std::numeric_limits<Time>::max();
//    for (const auto& conflict : rectangle_conflicts_conss)
//    {
//        // Get variables in this row.
//        auto nvars = SCIProwGetNNonz(conflict.row);
//        auto cols = SCIProwGetCols(conflict.row);
//        auto coeffs = SCIProwGetVals(conflict.row);
//
//        // Check.
//#ifdef DEBUG
//        SCIP_Real tmp = 0.0;
//        for (Count v = 0; v < agent_nvars[conflict.a1]; ++v)
//        {
//            auto var = agent_vars[conflict.a1][v];
//            auto vardata = SCIPvarGetData(var);
//            const auto a = SCIPvardataGetAgent(vardata);
//            const auto path_length = SCIPvardataGetPathLength(vardata);
//            const auto path = SCIPvardataGetPath(vardata);
//
//            const auto var_val = SCIPgetSolVal(scip, nullptr, var);
//
//            SCIP_Real coeff = 0.0;
//            for (auto [it, end] = conflict.agent_edges(a); it != end; ++it)
//            {
//                const auto& [t, e] = *it;
//                if (t < path_length - 1 && path[t] == e)
//                {
//                    coeff++;
//                }
//            }
//            tmp += coeff * var_val;
//        }
//        for (Count v = 0; v < agent_nvars[conflict.a2]; ++v)
//        {
//            auto var = agent_vars[conflict.a2][v];
//            auto vardata = SCIPvarGetData(var);
//            const auto a = SCIPvardataGetAgent(vardata);
//            const auto path_length = SCIPvardataGetPathLength(vardata);
//            const auto path = SCIPvardataGetPath(vardata);
//
//            const auto var_val = SCIPgetSolVal(scip, nullptr, var);
//
//            SCIP_Real coeff = 0.0;
//            for (auto [it, end] = conflict.agent_edges(a); it != end; ++it)
//            {
//                const auto& [t, e] = *it;
//                if (t < path_length - 1 && path[t] == e)
//                {
//                    coeff++;
//                }
//            }
//            tmp += coeff * var_val;
//        }
//#endif
//
//        // Compute LHS.
//        SCIP_Real lhs = 0.0;
//        for (auto v = 0; v < nvars; ++v)
//        {
//            // Check.
//            debug_assert(coeffs[v] == 1 || coeffs[v] == 2);
//
//            // Sum.
//            auto var = SCIPcolGetVar(cols[v]);
//            const auto var_val = SCIPgetSolVal(scip, nullptr, var);
//            lhs += var_val * coeffs[v];
//        }
//        debug_assert(SCIPisEQ(scip, lhs, tmp));
//
//        // Skip if the constraint is not binding.
//        if (SCIPisSumLT(scip, lhs, best_lhs))
//            continue;
//
//        // Loop through all columns of the row whose path crosses two boundaries.
//        for (auto v = 0; v < nvars; ++v)
//            if (coeffs[v] == 2)
//            {
//                // Get the variable value.
//                auto var = SCIPcolGetVar(cols[v]);
//                const auto var_val = SCIPgetSolVal(scip, nullptr, var);
//
//                // Find a wait.
//                if (SCIPisPositive(scip, var_val))
//                {
//                    // Get the path.
//                    auto vardata = SCIPvarGetData(var);
//                    const auto a = SCIPvardataGetAgent(vardata);
//                    const auto path_length = SCIPvardataGetPathLength(vardata);
//                    const auto path = SCIPvardataGetPath(vardata);
//
//                    // Don't consider longer paths.
//                    if (path_length > best_path_length)
//                        continue;
//
//                    // Find the time of entry and exit from the rectangle.
//                    Time entry = -1, exit = -1;
//                    auto [it, out_begin, out_end] = conflict.agent_in_out_edges(a);
//                    for (; it != out_begin; ++it)
//                    {
//                        const auto& [t, e] = *it;
//                        if (t < path_length - 1 && path[t] == e)
//                        {
//                            entry = t;
//                            break;
//                        }
//                    }
//                    for (it = out_begin; it != out_end; ++it)
//                    {
//                        const auto& [t, e] = *it;
//                        if (t < path_length - 1 && path[t] == e)
//                        {
//                            exit = t;
//                            break;
//                        }
//                    }
//                    release_assert(entry >= 0 && exit > entry);
//
//                    // Find a wait between the entry and exit times.
//                    for (Time t = entry; t < std::min(exit, best_at.t); ++t)
//                    {
//                        const AgentTime at{a, t};
//                        auto it = succ_dirs.find(at);
//                        if (it != succ_dirs.end() &&
//                            it->second.has_move && it->second.has_wait)
//                        {
//                            best_lhs = lhs;
//                            best_path_length = path_length;
//                            best_at = at;
//                        }
//                    }
//                }
//            }
//    }
//#ifdef PRINT_DEBUG
//    if (best_at.a >= 0)
//    {
//        debugln("   Selected decision as wait");
//    }
//#endif
//
//    // Return.
//    return {best_at, prefer_branch_0};
//}

Pair<AgentNodeTime, bool> find_decision_vertex(
    SCIP* scip,                                                                   // SCIP
    SCIP_PROBDATA* probdata,                                                      // Problem data
    const HashTable<NodeTime, Pair<Vector<Score>, Vector<Score>>>& candidates,    // Candidate agent-time-nodes
    const Vector<GoalTimeBound>& goal_time_bounds,                                // Earliest and latest time an agent reaches its goal
    const Vector<Int>& nb_paths                                                   // Number of paths used by an agent
)
{
    // Create output.
    AgentNodeTime best_ant{-1, 0, std::numeric_limits<Time>::max()};
    bool prefer_branch_0 = false;

    // Get agent variables.
#ifdef DEBUG
    const auto& agent_vars = SCIPprobdataGetAgentVars(probdata);
#endif

    // Prefer vertices inside a rectangle clique conflict.
#ifdef USE_RECTANGLE_CLIQUE_CONFLICTS
    {
        const auto& rectangle_clique_conflicts_conss =
            rectangle_clique_conflicts_get_constraints(probdata);
        SCIP_Real best_lhs = 1.0 - 0.99;
        Time best_path_length = std::numeric_limits<Time>::max();
        for (const auto& conflict : rectangle_clique_conflicts_conss)
        {
            // Get variables in this row.
            auto nvars = SCIProwGetNNonz(conflict.row);
            auto cols = SCIProwGetCols(conflict.row);
            auto coeffs = SCIProwGetVals(conflict.row);

            // Check.
#ifdef DEBUG
            SCIP_Real tmp = 0.0;
            for (auto var : agent_vars[conflict.a1])
            {
                debug_assert(var);
                auto vardata = SCIPvarGetData(var);
                const auto a = SCIPvardataGetAgent(vardata);
                const auto path_length = SCIPvardataGetPathLength(vardata);
                const auto path = SCIPvardataGetPath(vardata);

                const auto var_val = SCIPgetSolVal(scip, nullptr, var);

                SCIP_Real count = 0.0;
                for (auto [it, end] = conflict.agent_edges(a); it != end; ++it)
                {
                    const auto e = it->et.e;
                    const auto t = it->t;
                    if (t < path_length - 1 && path[t] == e)
                    {
                        count++;
                    }
                }
                debug_assert(count <= 2);
                if (count == 2)
                {
                    tmp += var_val;
                }
            }
            for (auto var : agent_vars[conflict.a2])
            {
                debug_assert(var);
                auto vardata = SCIPvarGetData(var);
                const auto a = SCIPvardataGetAgent(vardata);
                const auto path_length = SCIPvardataGetPathLength(vardata);
                const auto path = SCIPvardataGetPath(vardata);

                const auto var_val = SCIPgetSolVal(scip, nullptr, var);

                SCIP_Real count = 0.0;
                for (auto [it, end] = conflict.agent_edges(a); it != end; ++it)
                {
                    const auto e = it->et.e;
                    const auto t = it->t;
                    if (t < path_length - 1 && path[t] == e)
                    {
                        count++;
                    }
                }
                debug_assert(count <= 2);
                if (count == 2)
                {
                    tmp += var_val;
                }
            }
#endif

            // Compute LHS.
            SCIP_Real lhs = 0.0;
            for (auto v = 0; v < nvars; ++v)
            {
                // Check.
                debug_assert(coeffs[v] == 1 || coeffs[v] == 2);

                // Sum.
                auto var = SCIPcolGetVar(cols[v]);
                const auto var_val = SCIPgetSolVal(scip, nullptr, var);
                lhs += var_val * coeffs[v];
            }
            debug_assert(SCIPisEQ(scip, lhs, tmp));

            // Skip if the constraint is not binding.
            if (SCIPisSumLT(scip, lhs, best_lhs))
                continue;

            // Loop through all columns of the row whose path crosses two boundaries.
            for (Int v = 0; v < nvars; ++v)
            {
                // Get the variable value.
                auto var = SCIPcolGetVar(cols[v]);
                const auto var_val = SCIPgetSolVal(scip, nullptr, var);

                // Find a wait.
                if (SCIPisPositive(scip, var_val))
                {
                    // Get the path.
                    auto vardata = SCIPvarGetData(var);
                    const auto a = SCIPvardataGetAgent(vardata);
                    const auto path_length = SCIPvardataGetPathLength(vardata);
                    const auto path = SCIPvardataGetPath(vardata);

                    // Don't consider longer paths.
                    if (path_length > best_path_length)
                        continue;

                    // Find the time of entry and exit from the rectangle.
                    Time entry = 0, exit = path_length;
                    auto [it, out_begin, out_end] = conflict.agent_in_out_edges(a);
                    for (; it != out_begin; ++it)
                    {
                        const auto e = it->et.e;
                        const auto t = it->t;
                        if (t < path_length - 1 && path[t] == e)
                        {
                            entry = t;
                            break;
                        }
                    }
                    for (it = out_begin; it != out_end; ++it)
                    {
                        const auto e = it->et.e;
                        const auto t = it->t;
                        if (t < path_length - 1 && path[t] == e)
                        {
                            exit = t + 1;
                            break;
                        }
                    }
                    release_assert(entry >= 0 && exit > entry && exit <= path_length);

                    // Find a candidate between the entry and exit times.
                    for (Time t = entry; t < std::min(exit, best_ant.t); ++t)
                    {
                        const NodeTime nt{path[t].n, t};
                        auto it = candidates.find(nt);
                        if (it != candidates.end())
                        {
                            const auto& scores = it->second.first;
                            auto it2 = std::find_if(scores.begin(),
                                                    scores.end(),
                                                    [a](const Score& score)
                                                    {
                                                        return score.a == a;
                                                    });
                            if (it2 != scores.end())
                            {
                                best_lhs = lhs;
                                best_path_length = path_length;
                                best_ant = {a, path[t].n, t};
                            }
                        }
                    }
                }
            }
        }
        if (best_ant.a >= 0)
        {
            debugln("   Selected decision as vertex within a rectangle clique conflict");
            return {best_ant, prefer_branch_0};
        }
    }
#endif

    // Prefer vertices inside a rectangle knapsack conflict.
#ifdef USE_RECTANGLE_KNAPSACK_CONFLICTS
    const auto& rectangle_knapsack_cuts = rectangle_knapsack_get_cuts(probdata);
    const auto& two_agent_robust_cuts = SCIPprobdataGetTwoAgentRobustCuts(probdata);
    {
        SCIP_Real best_lhs = 3.0 - 0.99;
        Time best_path_length = std::numeric_limits<Time>::max();
        for (const auto [idx, out1_begin, in2_begin, out2_begin] : rectangle_knapsack_cuts)
        {
            // Get the cut.
            debug_assert(idx < static_cast<Int>(two_agent_robust_cuts.size()));
            const auto& cut = two_agent_robust_cuts[idx];

            // Get variables in this row.
            auto nvars = SCIProwGetNNonz(cut.row());
            auto cols = SCIProwGetCols(cut.row());
            auto coeffs = SCIProwGetVals(cut.row());

            // Check.
#ifdef DEBUG
            SCIP_Real tmp = 0.0;
            for (const auto a : Array<Agent, 2>{cut.a1(), cut.a2()})
                for (auto var : agent_vars[a])
                {
                    debug_assert(var);
                    auto vardata = SCIPvarGetData(var);
                    const auto path_length = SCIPvardataGetPathLength(vardata);
                    const auto path = SCIPvardataGetPath(vardata);

                    const auto var_val = SCIPgetSolVal(scip, nullptr, var);

                    SCIP_Real coeff = 0;
                    for (auto [it, end] = cut.edge_times(a); it != end; ++it)
                    {
                        const auto e = it->et.e;
                        const auto t = it->t;
                        coeff += (t < path_length - 1 && path[t] == e);
                    }
                    debug_assert(coeff <= 2);
                    tmp += coeff * var_val;
                }
#endif

            // Compute LHS.
            SCIP_Real lhs = 0.0;
            for (auto v = 0; v < nvars; ++v)
            {
                // Check.
                debug_assert(coeffs[v] == 1 || coeffs[v] == 2);

                // Sum.
                auto var = SCIPcolGetVar(cols[v]);
                debug_assert(var);
                const auto var_val = SCIPgetSolVal(scip, nullptr, var);
                lhs += var_val * coeffs[v];
            }
            debug_assert(SCIPisEQ(scip, lhs, tmp));

            // Skip if the constraint is not binding.
            if (SCIPisSumLT(scip, lhs, best_lhs))
                continue;

            // Loop through all columns of the row whose path crosses two boundaries.
            for (Int v = 0; v < nvars; ++v)
            {
                // Get the variable value.
                auto var = SCIPcolGetVar(cols[v]);
                debug_assert(var);
                const auto var_val = SCIPgetSolVal(scip, nullptr, var);

                // Find a wait.
                if (SCIPisPositive(scip, var_val))
                {
                    // Get the path.
                    auto vardata = SCIPvarGetData(var);
                    const auto a = SCIPvardataGetAgent(vardata);
                    const auto path_length = SCIPvardataGetPathLength(vardata);
                    const auto path = SCIPvardataGetPath(vardata);

                    // Don't consider longer paths.
                    if (path_length > best_path_length)
                        continue;

                    // Find the time of entry and exit from the rectangle.
                    Time entry = 0, exit = path_length;
                    auto [it, out_end] = cut.edge_times(a);
                    auto out_begin = cut.edge_times_a1().first +
                        (a == cut.a1() ? out1_begin : out2_begin);
                    for (; it != out_begin; ++it)
                    {
                        const auto e = it->et.e;
                        const auto t = it->t;
                        if (t < path_length - 1 && path[t] == e)
                        {
                            entry = t;
                            break;
                        }
                    }
                    for (; it != out_end; ++it)
                    {
                        const auto e = it->et.e;
                        const auto t = it->t;
                        if (t < path_length - 1 && path[t] == e)
                        {
                            exit = t + 1;
                            break;
                        }
                    }
                    release_assert(entry >= 0 && exit > entry && exit <= path_length);

                    // Find a candidate between the entry and exit times.
                    for (Time t = entry; t < std::min(exit, best_ant.t); ++t)
                    {
                        const NodeTime nt{path[t].n, t};
                        auto it = candidates.find(nt);
                        if (it != candidates.end())
                        {
                            const auto& scores = it->second.first;
                            auto it2 = std::find_if(scores.begin(),
                                                    scores.end(),
                                                    [a](const Score& score)
                                                    {
                                                        return score.a == a;
                                                    });
                            if (it2 != scores.end())
                            {
                                best_lhs = lhs;
                                best_path_length = path_length;
                                best_ant = {a, path[t].n, t};
                            }
                        }
                    }
                }
            }
        }
        if (best_ant.a >= 0)
        {
            debugln("   Selected decision as vertex within a rectangle knapsack "
                    "conflict");
            return {best_ant, prefer_branch_0};
        }
    }
#endif

    // Prefer a vertex allowing an agent to reach its goal the earliest.
    Time best_diff = 0;
    Int best_nb_paths = 0;
    Time best_shortest_path_length = std::numeric_limits<Time>::max();
    SCIP_Real best_val = 0.0;
    for (const auto& [nt, scores] : candidates)
        for (const auto& [a, val, shortest_path_length] : scores.first)
            if (const auto diff = goal_time_bounds[a].last - goal_time_bounds[a].first;
                (diff >  best_diff) ||
                (diff == best_diff && nb_paths[a] >  best_nb_paths) ||
                (diff == best_diff && nb_paths[a] == best_nb_paths && shortest_path_length <  best_shortest_path_length) ||
                (diff == best_diff && nb_paths[a] == best_nb_paths && shortest_path_length == best_shortest_path_length && nt.t <  best_ant.t) ||
                (diff == best_diff && nb_paths[a] == best_nb_paths && shortest_path_length == best_shortest_path_length && nt.t == best_ant.t && val > best_val))
            {
                best_diff = diff;
                best_nb_paths = nb_paths[a];
                best_shortest_path_length = shortest_path_length;
                best_val = val;
                best_ant = {a, nt.n, nt.t};
            }
    if (best_ant.a >= 0)
    {
        debugln("   Selected decision as integer vertex");
        return {best_ant, prefer_branch_0};
    }

    // No integer vertex was found so find a fractional vertex, arising from edge
    // conflicts.
    best_diff = 0;
    best_nb_paths = 0;
    best_shortest_path_length = std::numeric_limits<Time>::max();
    best_val = 0.0;
    for (const auto& [nt, scores] : candidates)
        for (const auto& [a, val, shortest_path_length] : scores.second)
            if (const auto diff = goal_time_bounds[a].last - goal_time_bounds[a].first;
                (diff >  best_diff) ||
                (diff == best_diff && nb_paths[a] >  best_nb_paths) ||
                (diff == best_diff && nb_paths[a] == best_nb_paths && shortest_path_length <  best_shortest_path_length) ||
                (diff == best_diff && nb_paths[a] == best_nb_paths && shortest_path_length == best_shortest_path_length && nt.t <  best_ant.t) ||
                (diff == best_diff && nb_paths[a] == best_nb_paths && shortest_path_length == best_shortest_path_length && nt.t == best_ant.t && val > best_val))
            {
                best_diff = diff;
                best_nb_paths = nb_paths[a];
                best_shortest_path_length = shortest_path_length;
                best_val = val;
                best_ant = {a, nt.n, nt.t};
            }
    if (best_ant.a >= 0)
    {
        debugln("   Selected decision as fractional vertex");
        return {best_ant, prefer_branch_0};
    }

    // Failed to select a decision.
    unreachable();
}

SCIP_RETCODE branch_lp(
    SCIP* scip,            // SCIP
    SCIP_RESULT* result    // Output status
)
{
    // Get problem data.
    auto probdata = SCIPgetProbData(scip);
    const auto N = SCIPprobdataGetN(probdata);

    // Print.
    debugln("Branching on fractional LP at node {}, depth {}, lb {}, ub {}:",
            SCIPnodeGetNumber(SCIPgetCurrentNode(scip)),
            SCIPgetDepth(scip),
            SCIPnodeGetLowerbound(SCIPgetCurrentNode(scip)),
            SCIPgetUpperbound(scip));

    // Print paths.
#ifdef PRINT_DEBUG
    print_used_paths(scip);
#endif

    // Get branching candidates.
    const auto [candidates, succ_dirs, nb_paths, goal_time_bounds] = get_lp_branch_candidates(scip, probdata, N);

    // If the LP is solved to optimality, the B&B node is infeasible if any dummy variable
    // is used.
    {
        const auto& dummy_vars = SCIPprobdataGetDummyVars(probdata);
        for (Agent a = 0; a < N; ++a)
        {
            // Get the variable.
            auto var = dummy_vars[a];
            debug_assert(var);
            debug_assert(!SCIPvarGetData(var));

            // Get the variable value.
            const auto var_val = SCIPgetSolVal(scip, nullptr, var);

            // Check.
            if (SCIPisPositive(scip, var_val))
            {
                // Print.
                debugln("   Node is infeasible by using artificial variable for agent {}",
                        a);

                // Done.
                *result = SCIP_CUTOFF;
                return SCIP_OKAY;
            }
        }
    }

    // Attempt to branch on finishing early.
    {
        const auto [ant, prefer_branch_0] = find_decision_early_goal(scip, probdata, N, goal_time_bounds);
        if (ant.a >= 0)
        {
            debug_assert(ant.t >= 0);
            branch_on_length(scip, ant, prefer_branch_0);
            goto DONE;
        }
    }

    // Attempt to branch on waits.
//    {
//        const auto [at, prefer_branch_0] = find_decision_wait(scip, probdata, succ_dirs, goal_time_bounds);
//        if (at.a >= 0)
//        {
//            debug_assert(at.t >= 0);
//            branch_on_wait(scip, at, prefer_branch_0);
//            goto DONE;
//        }
//    }

    // Attempt to branch on vertices.
    {
        const auto [ant, prefer_branch_0] = find_decision_vertex(scip, probdata, candidates, goal_time_bounds, nb_paths);
        if (ant.a >= 0)
        {
            debug_assert(ant.t > 0);
            branch_on_vertex(scip, ant, prefer_branch_0);
            goto DONE;
        }
    }

    // Failed to find a branching decision.
    unreachable();

    // Done.
    DONE:
    *result = SCIP_BRANCHED;
    return SCIP_OKAY;
}
