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

#ifdef USE_GOAL_CONFLICTS

//#define PRINT_DEBUG

#include "Separator_GoalConflicts.h"
#include "ProblemData.h"
#include "VariableData.h"

#define SEPA_NAME         "goal"
#define SEPA_DESC         "Separator for goal conflicts"
#define SEPA_PRIORITY     1001     // priority of the constraint handler for separation
#define SEPA_FREQ         1        // frequency for separating cuts; zero means to separate only in the root node
#define SEPA_MAXBOUNDDIST 1.0
#define SEPA_USESSUBSCIP  FALSE    // does the separator use a secondary SCIP instance? */
#define SEPA_DELAY        FALSE    // should separation method be delayed, if other separators found cuts? */

struct GoalConflictData
{
    SCIP_Real lhs;
    Agent a1;
    Agent a2;
    NodeTime nt;
};

#define MATRIX(i,j) (i * N + j)

SCIP_RETCODE goal_conflicts_create_cut(
    SCIP* scip,                              // SCIP
    SCIP_SEPA* sepa,                         // Separator
    Vector<GoalConflict>& goal_conflicts,    // Goal conflicts
    const Agent a1,                          // Agent of the goal
    const Agent a2,                          // Agent trying to use the goal vertex
    const NodeTime nt,                       // Node-time of the conflict
    const Vector<SCIP_VAR*>& a1_vars,        // Array of variables for agent 1
    const Vector<SCIP_VAR*>& a2_vars,        // Array of variables for agent 2
    SCIP_Result* result                      // Output result
)
{
    // Get problem data.
    auto probdata = SCIPgetProbData(scip);

    // Create constraint name.
#ifdef DEBUG
    const auto& map = SCIPprobdataGetMap(SCIPgetProbData(scip));
    const auto [x, y] = map.get_xy(nt.n);
    const auto name = fmt::format("goal_conflict({},{},({},{}),{})", a1, a2, x, y, nt.t);
#endif

    // Create a row.
    SCIP_ROW* row = nullptr;
    SCIP_CALL(SCIPcreateEmptyRowSepa(scip,
                                     &row,
                                     sepa,
#ifdef DEBUG
                                     name.c_str(),
#else
                                     "",
#endif
                                     -SCIPinfinity(scip),
                                     1.0,
                                     FALSE,
                                     TRUE,
                                     FALSE));
    debug_assert(row);

    // Add variables to the constraint.
    SCIP_CALL(SCIPcacheRowExtensions(scip, row));
    for (auto var : a1_vars)
    {
        // Get the path length.
        debug_assert(var);
        auto vardata = SCIPvarGetData(var);
        debug_assert(a1 == SCIPvardataGetAgent(vardata));
        const auto path_length = SCIPvardataGetPathLength(vardata);
#ifdef DEBUG
        const auto path = SCIPvardataGetPath(vardata);
        debug_assert(path[path_length - 1].n == nt.n);
#endif

        // Add coefficients.
        const auto t = path_length - 1;
        if (t <= nt.t)
        {
            // Print.
            debugln("      val: {:7.4f}:, agent: {:2d}, path: {}",
                    SCIPgetSolVal(scip, nullptr, var),
                    a1,
                    format_path_spaced(SCIPgetProbData(scip), path_length, path));

            // Add coefficient.
            SCIP_CALL(SCIPaddVarToRow(scip, row, var, 1.0));
        }
    }
    for (auto var : a2_vars)
    {
        // Get the path.
        debug_assert(var);
        auto vardata = SCIPvarGetData(var);
        debug_assert(a2 == SCIPvardataGetAgent(vardata));
        const auto path_length = SCIPvardataGetPathLength(vardata);
        const auto path = SCIPvardataGetPath(vardata);

        // Add coefficients.
        for (Time t = nt.t; t < path_length - 1; ++t)
            if (path[t].n == nt.n)
            {
                // Print.
#ifdef PRINT_DEBUG
                debugln("      val: {:7.4f}:, agent: {:2d}, path: {}",
                        SCIPgetSolVal(scip, nullptr, var),
                        a2,
                        format_path_spaced(SCIPgetProbData(scip), path_length, path));
#endif

                // Add coefficient.
                SCIP_CALL(SCIPaddVarToRow(scip, row, var, 1.0));
                break;
            }
    }
    SCIP_CALL(SCIPflushRowExtensions(scip, row));

    // Add the row to the LP.
    SCIP_Bool infeasible;
    SCIP_CALL(SCIPaddRow(scip, row, TRUE, &infeasible));

    // Set status.
    if (infeasible)
    {
        *result = SCIP_CUTOFF;
    }
    else
    {
        *result = SCIP_SEPARATED;
    }

    // Store the constraint by agent.
    {
        debug_assert(nt.n == SCIPprobdataGetAgentsData(probdata)[a1].goal);

        auto& goal_agent_goal_conflicts = SCIPprobdataGetGoalAgentGoalConflicts(probdata);
        goal_agent_goal_conflicts[a1].push_back({nt.t, row});
    }
    {

        auto& crossing_agent_goal_conflicts = SCIPprobdataGetCrossingAgentGoalConflicts(probdata);
        crossing_agent_goal_conflicts[a2].push_back({nt, row});
    }

    // Store the constraint.
    goal_conflicts.push_back({row, a1, a2, nt});

    // Done.
    return SCIP_OKAY;
}

// Separator
static
SCIP_RETCODE goal_conflicts_separate(
    SCIP* scip,            // SCIP
    SCIP_SEPA* sepa,       // Separator
    SCIP_RESULT* result    // Pointer to store the result of the separation call
)
{
    // Print.
    debugln("Starting separator for goal conflicts on solution with obj {:.6f}:",
            SCIPgetSolOrigObj(scip, nullptr));

    // Print paths.
#ifdef PRINT_DEBUG
    print_used_paths(scip);
#endif

    // Get problem data.
    auto probdata = SCIPgetProbData(scip);
    const auto N = SCIPprobdataGetN(probdata);
    const auto& agents = SCIPprobdataGetAgentsData(probdata);
    auto& goal_conflicts = SCIPprobdataGetGoalConflicts(probdata);

    // Skip this separator if an earlier separator found cuts.
    auto& found_cuts = SCIPprobdataGetFoundCutsIndicator(probdata);
    if (found_cuts)
        return SCIP_OKAY;

    // Get variables.
    const auto& agent_vars = SCIPprobdataGetAgentVars(probdata);

    // Force cuts for debugging.
//    {
//        const auto& map = SCIPprobdataGetMap(probdata);
//
//        Vector<Agent> a1s{};
//        Vector<Agent> a2s{};
//        Vector<Position> xs{};
//        Vector<Position> ys{};
//        Vector<Time> ts{};
//
//        for (size_t idx = 0; idx < a1s.size(); ++idx)
//        {
//            const auto a1 = a1s[idx];
//            const auto a2 = a2s[idx];
//            const auto nt = NodeTime(map.get_id(xs[idx], ys[idx]), ts[idx]);
//
//            SCIP_CALL(goal_conflicts_create_cut(scip,
//                                                sepa,
//                                                sepadata,
//                                                a1,
//                                                a2,
//                                                nt,
//                                                agent_vars[a1],
//                                                agent_vars[a2],
//                                                result));
//        }
//    }

    // Get the times that each agent finishes.
    Vector<Vector<Time>> finish_times(N);
    for (Agent a = 0; a < N; ++a)
        for (auto var : agent_vars[a])
        {
            // Get the path length.
            debug_assert(var);
            auto vardata = SCIPvarGetData(var);
            const auto path_length = SCIPvardataGetPathLength(vardata);

            // Get the variable value.
            const auto var_val = SCIPgetSolVal(scip, nullptr, var);

            // Store unique finish times.
            if (SCIPisPositive(scip, var_val))
            {
                const auto t = path_length - 1;
                if (std::find(finish_times[a].begin(), finish_times[a].end(), t) ==
                    finish_times[a].end())
                {
                    finish_times[a].push_back(t);
                }
            }
        }

    // Find conflicts.
    Vector<GoalConflictData> cuts;
    for (Agent a1 = 0; a1 < N; ++a1)
    {
        const auto conflict_node = agents[a1].goal;
        for (const auto conflict_time : finish_times[a1])
        {
            // Make the node-time of the conflict.
            const NodeTime nt{conflict_node, conflict_time};

            // Sum paths belonging to the agent of the conflicting goal.
            SCIP_Real lhs1 = 0.0;
            for (auto var : agent_vars[a1])
            {
                // Get the path length.
                debug_assert(var);
                auto vardata = SCIPvarGetData(var);
                const auto path_length = SCIPvardataGetPathLength(vardata);
#ifdef DEBUG
                const auto path = SCIPvardataGetPath(vardata);
                debug_assert(path[path_length - 1].n == conflict_node);
#endif

                // Check for conflicts.
                const auto t = path_length - 1;
                if (t <= nt.t)
                {
                    const auto var_val = SCIPgetSolVal(scip, nullptr, var);
                    lhs1 += var_val;
                }
            }

            // Check if any agent is trying to cross the goal.
            for (Agent a2 = 0; a2 < N; ++a2)
                if (a2 != a1)
                {
                    // Sum paths belonging to the agent trying to cross the goal.
                    SCIP_Real lhs2 = 0.0;
                    for (auto var : agent_vars[a2])
                    {
                        // Only check paths in use.
                        debug_assert(var);
                        const auto var_val = SCIPgetSolVal(scip, nullptr, var);
                        if (SCIPisPositive(scip, var_val))
                        {
                            // Get the path.
                            auto vardata = SCIPvarGetData(var);
                            debug_assert(a2 == SCIPvardataGetAgent(vardata));
                            const auto path_length = SCIPvardataGetPathLength(vardata);
                            const auto path = SCIPvardataGetPath(vardata);

                            // Check for conflicts.
                            for (Time t = nt.t; t < path_length - 1; ++t)
                                if (path[t].n == nt.n)
                                {
                                    lhs2 += var_val;
                                    break;
                                }
                        }
                    }

                    // Store a cut if violated.
                    const auto lhs = lhs1 + lhs2;
                    if (SCIPisSumGT(scip, lhs, 1.0 + CUT_VIOLATION) && SCIPisSumGT(scip, lhs2, 0))
                    {
                        cuts.emplace_back(GoalConflictData{lhs, a1, a2, nt});
                    }
                }
        }
    }

    // Create the most violated cuts.
    Vector<Int> agent_nb_cuts(N * N);
    std::sort(cuts.begin(),
              cuts.end(),
              [](const GoalConflictData& a, const GoalConflictData& b) { return a.lhs > b.lhs; });
    for (const auto& cut : cuts)
    {
        const auto& [lhs, a1, a2, nt] = cut;
        auto& nb_cuts = agent_nb_cuts[MATRIX(std::min(a1, a2), std::max(a1, a2))];
        if (nb_cuts < 5)
        {
            // Print.
#ifdef PRINT_DEBUG
            const auto& map = SCIPprobdataGetMap(probdata);
            debugln("   Creating goal conflict cut for goal agent {} and crossing agent {} at ({},{}) at time {} with "
                    "value {} in branch-and-bound node {}",
                    a1, a2, map.get_x(nt.n), map.get_y(nt.n), nt.t,
                    lhs,
                    SCIPnodeGetNumber(SCIPgetCurrentNode(scip)));
#endif

            // Create cut.
            SCIP_CALL(goal_conflicts_create_cut(scip,
                                                sepa,
                                                goal_conflicts,
                                                a1,
                                                a2,
                                                nt,
                                                agent_vars[a1],
                                                agent_vars[a2],
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
SCIP_DECL_SEPACOPY(sepaCopyGoalConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);

    // Include separator.
    SCIP_CALL(SCIPincludeSepaGoalConflicts(scip));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Separation method for LP solutions
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_SEPAEXECLP(sepaExeclpGoalConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);
    debug_assert(result);

    // Start.
    *result = SCIP_DIDNOTFIND;

    // Start separator.
    SCIP_CALL(goal_conflicts_separate(scip, sepa, result));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Create separator for goal conflicts constraints and include it in SCIP
SCIP_RETCODE SCIPincludeSepaGoalConflicts(
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
                                   sepaExeclpGoalConflicts,
                                   nullptr,
                                   nullptr));
    debug_assert(sepa);

    // Set callbacks.
    SCIP_CALL(SCIPsetSepaCopy(scip, sepa, sepaCopyGoalConflicts));

    // Done.
    return SCIP_OKAY;
}

SCIP_RETCODE goal_conflicts_add_var(
    SCIP* scip,                              // SCIP
    Vector<GoalConflict>& goal_conflicts,    // Goal conflicts
    SCIP_VAR* var,                           // Variable
    const Agent a,                           // Agent
    const Time path_length,                  // Path length
    const Edge* const path                   // Path
)
{
    // Check.
    debug_assert(var);
    debug_assert(SCIPvarIsTransformed(var));

    // Add variable to constraints.
    for (const auto& [row, a1, a2, nt] : goal_conflicts)
    {
        if (a == a1)
        {
            const auto t = path_length - 1;
            if (t <= nt.t)
            {
                SCIP_CALL(SCIPaddVarToRow(scip, row, var, 1.0));
            }
        }
        else if (a == a2)
        {
            for (Time t = nt.t; t < path_length - 1; ++t)
                if (path[t].n == nt.n)
                {
                    SCIP_CALL(SCIPaddVarToRow(scip, row, var, 1.0));
                    break;
                }
        }
    }

    // Return.
    return SCIP_OKAY;
}

#endif
