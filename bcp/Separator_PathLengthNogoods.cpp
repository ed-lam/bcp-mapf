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

#ifdef USE_PATH_LENGTH_NOGOODS

#define PRINT_DEBUG

#include "Separator_PathLengthNogoods.h"
#include "ProblemData.h"
#include "VariableData.h"

#define SEPA_NAME         "path_length_nogoods"
#define SEPA_DESC         "Separator for path length nogoods"
#define SEPA_PRIORITY     10001    // priority of the constraint handler for separation
#define SEPA_FREQ         1        // frequency for separating cuts; zero means to separate only in the root node
#define SEPA_MAXBOUNDDIST 1.0
#define SEPA_USESSUBSCIP  FALSE    // does the separator use a secondary SCIP instance? */
#define SEPA_DELAY        FALSE    // should separation method be delayed, if other separators found cuts? */

SCIP_RETCODE path_length_nogoods_create_cut(
    SCIP* scip,                                              // SCIP
    SCIP_SEPA* sepa,                                         // Separator
    const Vector<Vector<SCIP_VAR*>>& agent_vars,             // Variables for each agent
    Vector<PathLengthNogood>& path_length_nogoods,           // Existing nogoods
    const Vector<Pair<Agent, Time>>& latest_finish_times,    // New nogood
    SCIP_Result* result                                      // Output result
)
{
    // Create constraint name.
#ifdef DEBUG
    String str;
    for (const auto [a, t] : latest_finish_times)
    {
        if (!str.empty())
        {
            str += ",";
        }
        str += fmt::format("({},{})", a, t);
    }
    auto name = fmt::format("path_length_nogood({})", str);
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
                                     latest_finish_times.size() - 1.0,
                                     FALSE,
                                     TRUE,
                                     FALSE));
    debug_assert(row);

    // Add variables to the constraint.
    SCIP_CALL(SCIPcacheRowExtensions(scip, row));
    for (const auto& [a, t] : latest_finish_times)
        for (const auto& [var, _] : agent_vars[a])
        {
            // Get the path length.
            debug_assert(var);
            auto vardata = SCIPvarGetData(var);
            const auto path_length = SCIPvardataGetPathLength(vardata);

            // Add coefficients.
            if (path_length - 1 <= t)
            {
                // Print.
                debugln("      val: {:7.4f}:, agent: {:2d}, path: {}",
                        SCIPgetSolVal(scip, nullptr, var),
                        a,
                        format_path_spaced(SCIPgetProbData(scip), path_length, SCIPvardataGetPath(vardata)));

                // Add coefficient.
                SCIP_CALL(SCIPaddVarToRow(scip, row, var, 1.0));
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
    path_length_nogoods.push_back({row, latest_finish_times});

    // Done.
    return SCIP_OKAY;
}

// Separator
static
SCIP_RETCODE path_length_nogoods_separate(
    SCIP* scip,            // SCIP
    SCIP_SEPA* sepa,       // Separator
    SCIP_RESULT* result    // Pointer to store the result of the separation call
)
{
    // Print.
    debugln("Starting separator for path length nogoods on solution with obj {:.6f}:",
            SCIPgetSolOrigObj(scip, nullptr));

    // Print paths.
#ifdef PRINT_DEBUG
    print_used_paths(scip);
#endif

    // Get problem data.
    auto probdata = SCIPgetProbData(scip);
    auto& path_length_nogoods = SCIPprobdataGetPathLengthNogoods(probdata);

    // Skip this separator if an earlier separator found cuts.
    auto& found_cuts = SCIPprobdataGetFoundCutsIndicator(probdata);
    if (found_cuts)
    {
        return SCIP_OKAY;
    }

    // Get variables.
    const auto& agent_vars = SCIPprobdataGetAgentVars(probdata);

    // Find conflicts.
    static int iter = 0;
    if (iter++ == 0)
    {
        for (const auto& latest_finish_times : Vector<Vector<Pair<Agent, Time>>>{
            {{1, 27}, {2, 27}},
            {{3, 23}, {4, 23}},
            {{5, 19}, {6, 19}},
            {{7, 15}, {8, 15}},
            {{9, 11}, {10, 11}},
        })
        {
            // Print.
#ifdef PRINT_DEBUG
            {
                String str;
                String str_t;
                for (const auto [a, t] : latest_finish_times)
                {
                    if (!str.empty())
                    {
                        str += ", ";
                    }
                    str += fmt::format("({},{})", a, t);
                }
                SCIP_Real lhs = 0;
                for (const auto [a, t] : latest_finish_times)
                    for (const auto& [var, var_val] : agent_vars[a])
                    {
                        debug_assert(var);
                        debug_assert(var_val == SCIPgetSolVal(scip, nullptr, var));

                        // Get the path length.
                        auto vardata = SCIPvarGetData(var);
                        const auto path_length = SCIPvardataGetPathLength(vardata);

                        // Sum the LHS.
                        if (path_length - 1 <= t)
                        {
                            lhs += var_val;
                        }
                    }
                const SCIP_Real rhs = latest_finish_times.size() - 1;
                debugln("   Creating path length nogood on agent-time pairs {} with value {} and RHS {} in "
                        "branch-and-bound node {}",
                        str,
                        lhs,
                        rhs,
                        SCIPnodeGetNumber(SCIPgetCurrentNode(scip)));
            }
#endif

            // Create the cut.
            SCIP_CALL(path_length_nogoods_create_cut(scip,
                                                     sepa,
                                                     agent_vars,
                                                     path_length_nogoods,
                                                     latest_finish_times,
                                                     result));
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
SCIP_DECL_SEPACOPY(sepaCopyPathLengthNogoods)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);

    // Include separator.
    SCIP_CALL(SCIPincludeSepaPathLengthNogoods(scip));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Separation method for LP solutions
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_SEPAEXECLP(sepaExeclpPathLengthNogoods)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);
    debug_assert(result);

    // Start.
    *result = SCIP_DIDNOTFIND;

    // Start separator.
    SCIP_CALL(path_length_nogoods_separate(scip, sepa, result));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Create separator for path length nogoods and include it in SCIP
SCIP_RETCODE SCIPincludeSepaPathLengthNogoods(
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
                                   sepaExeclpPathLengthNogoods,
                                   nullptr,
                                   nullptr));
    debug_assert(sepa);

    // Set callbacks.
    SCIP_CALL(SCIPsetSepaCopy(scip, sepa, sepaCopyPathLengthNogoods));

    // Done.
    return SCIP_OKAY;
}

SCIP_RETCODE path_length_nogoods_add_var(
    SCIP* scip,                                       // SCIP
    Vector<PathLengthNogood>& path_length_nogoods,    // Data for the nogoods
    SCIP_VAR* var,                                    // Variable
    const Agent a,                                    // Agent
    const Time path_length                            // Path length
)
{
    // Check.
    debug_assert(var);
    debug_assert(SCIPvarIsTransformed(var));

    // Add variable to constraints.
    for (const auto& [row, latest_finish_times] : path_length_nogoods)
        for (const auto& [nogood_a, nogood_t] : latest_finish_times)
            if (a == nogood_a && path_length - 1 <= nogood_t)
            {
                SCIP_CALL(SCIPaddVarToRow(scip, row, var, 1.0));
            }

    // Return.
    return SCIP_OKAY;
}

#endif
