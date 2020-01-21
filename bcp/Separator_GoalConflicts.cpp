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

#define SEPA_NAME                          "goal_conflicts"
#define SEPA_DESC            "Separator for goal conflicts"
#define SEPA_PRIORITY                               +600000 // priority of the constraint handler for separation
#define SEPA_FREQ                                         1  // frequency for separating cuts; zero means to separate only in the root node
#define SEPA_MAXBOUNDDIST                               1.0
#define SEPA_USESSUBSCIP                              FALSE // does the separator use a secondary SCIP instance? */
#define SEPA_DELAY                                    FALSE // should separation method be delayed, if other separators found cuts? */

// Data for goal conflicts
struct GoalConflictsSepaData
{
    Vector<GoalConflict> conflicts;
};

SCIP_RETCODE goal_conflicts_create_cut(
    SCIP* scip,                          // SCIP
    SCIP_SEPA* sepa,                     // Separator
    GoalConflictsSepaData* sepadata,     // Separator data
    const Agent a1,                      // Agent of the goal
    const Agent a2,                      // Agent trying to use the goal vertex
    const NodeTime nt,                   // Node-time of the conflict
    const Vector<SCIP_VAR*>& a1_vars,    // Array of variables for agent 1
    const Vector<SCIP_VAR*>& a2_vars,    // Array of variables for agent 2
    SCIP_Result* result                  // Output result
)
{
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

    // Store the constraint.
    sepadata->conflicts.push_back({row, a1, a2, nt});

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

    // Get separator data.
    auto sepadata = reinterpret_cast<GoalConflictsSepaData*>(SCIPsepaGetData(sepa));
    debug_assert(sepadata);

    // Get problem data.
    auto probdata = SCIPgetProbData(scip);
    const auto N = SCIPprobdataGetN(probdata);
    const auto& agents = SCIPprobdataGetAgentsData(probdata);

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

                    // Create a cut only if violated.
                    if (SCIPisSumGT(scip, lhs1 + lhs2, 1.0) && lhs2 > 0)
                    {
                        // Print.
#ifdef PRINT_DEBUG
                        const auto& map = SCIPprobdataGetMap(probdata);
                        debugln("   Creating goal conflict cut for goal agent {} and "
                                "crossing agent {} at ({},{}) at time {} with value {} "
                                "in branch-and-bound node {}",
                                a1, a2, map.get_x(nt.n), map.get_y(nt.n), nt.t,
                                lhs1 + lhs2,
                                SCIPnodeGetNumber(SCIPgetCurrentNode(scip)));
#endif

                        // Create cut.
                        SCIP_CALL(goal_conflicts_create_cut(scip,
                                                            sepa,
                                                            sepadata,
                                                            a1,
                                                            a2,
                                                            nt,
                                                            agent_vars[a1],
                                                            agent_vars[a2],
                                                            result));
                    }
                }
        }
    }

    // Done.
    return SCIP_OKAY;
}

// Copy method for separator
static
SCIP_DECL_SEPACOPY(sepaCopyGoalConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);

    // Include separator.
    SCIP_SEPA* sepa_copy;
    SCIP_CALL(SCIPincludeSepaGoalConflicts(scip, &sepa_copy));

    // Done.
    return SCIP_OKAY;
}

// Free rows
static
SCIP_DECL_SEPAEXITSOL(sepaExitsolGoalConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);

    // Get separator data.
    auto sepadata = reinterpret_cast<GoalConflictsSepaData*>(SCIPsepaGetData(sepa));
    debug_assert(sepadata);

    // Free row for each goal conflict.
    for (auto& conflict : sepadata->conflicts)
    {
        SCIP_CALL(SCIPreleaseRow(scip, &conflict.row));
    }
    sepadata->conflicts.clear();

    // Done.
    return SCIP_OKAY;
}

// Free separator data
static
SCIP_DECL_SEPAFREE(sepaFreeGoalConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);

    // Get separator data.
    auto sepadata = reinterpret_cast<GoalConflictsSepaData*>(SCIPsepaGetData(sepa));
    debug_assert(sepadata);

    // Free memory.
    sepadata->~GoalConflictsSepaData();
    SCIPfreeBlockMemory(scip, &sepadata);

    // Done.
    return SCIP_OKAY;
}

// Separation method for LP solutions
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

// Create separator for goal conflicts constraints and include it in SCIP
SCIP_RETCODE SCIPincludeSepaGoalConflicts(
    SCIP* scip,         // SCIP
    SCIP_SEPA** sepa    // Output pointer to separator
)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);

    // Create separator data.
    GoalConflictsSepaData* sepadata = nullptr;
    SCIP_CALL(SCIPallocBlockMemory(scip, &sepadata));
    debug_assert(sepadata);
    new(sepadata) GoalConflictsSepaData;
    sepadata->conflicts.reserve(500);

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
                                   sepaExeclpGoalConflicts,
                                   nullptr,
                                   reinterpret_cast<SCIP_SEPADATA*>(sepadata)));
    debug_assert(*sepa);

    // Set callbacks.
    SCIP_CALL(SCIPsetSepaCopy(scip, *sepa, sepaCopyGoalConflicts));
    SCIP_CALL(SCIPsetSepaFree(scip, *sepa, sepaFreeGoalConflicts));
    SCIP_CALL(SCIPsetSepaExitsol(scip, *sepa, sepaExitsolGoalConflicts));

    // Done.
    return SCIP_OKAY;
}

SCIP_RETCODE goal_conflicts_add_var(
    SCIP* scip,                 // SCIP
    SCIP_SEPA* sepa,            // Separator for goal conflicts
    SCIP_VAR* var,              // Variable
    const Agent a,              // Agent
    const Time path_length,     // Path length
    const Edge* const path      // Path
)
{
    // Get separator data.
    debug_assert(sepa);
    auto sepadata = reinterpret_cast<GoalConflictsSepaData*>(SCIPsepaGetData(sepa));
    debug_assert(sepadata);
    auto& conflicts = sepadata->conflicts;

    // Check.
    debug_assert(var);
    debug_assert(SCIPvarIsTransformed(var));

    // Add variable to constraints.
    for (const auto& [row, a1, a2, nt] : conflicts)
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

const Vector<GoalConflict>& goal_conflicts_get_constraints(
    SCIP_ProbData* probdata    // Problem data
)
{
    auto sepa = SCIPprobdataGetGoalConflictsSepa(probdata);
    debug_assert(sepa);
    auto sepadata = reinterpret_cast<GoalConflictsSepaData*>(SCIPsepaGetData(sepa));
    debug_assert(sepadata);
    return sepadata->conflicts;
}

#endif
