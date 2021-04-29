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

#include "ConstraintHandler_VertexConflicts.h"
#include "ProblemData.h"
#include "VariableData.h"

#define CONSHDLR_NAME                                 "vertex_conflicts"
#define CONSHDLR_DESC          "Constraint handler for vertex conflicts"
#define CONSHDLR_SEPAPRIORITY                                    +500000 // priority of the constraint handler for separation
#define CONSHDLR_ENFOPRIORITY                                    -900000 // priority of the constraint handler for constraint enforcing
#define CONSHDLR_CHECKPRIORITY                                   -900000 // priority of the constraint handler for checking feasibility
#define CONSHDLR_SEPAFREQ                                              1 // frequency for separating cuts; zero means to separate only in the root node
#define CONSHDLR_EAGERFREQ                                             1 // frequency for using all instead of only the useful constraints in separation,
                                                                         // propagation and enforcement, -1 for no eager evaluations, 0 for first only
#define CONSHDLR_DELAYSEPA                                         FALSE // should separation method be delayed, if other separators found cuts?
#define CONSHDLR_NEEDSCONS                                          TRUE // should the constraint handler be skipped, if no constraints are available?

// Data for vertex conflicts
struct VertexConflictsConsData
{
    Vector<VertexConflict> conflicts;
};

// Create a constraint for vertex conflicts and include it
SCIP_RETCODE SCIPcreateConsBasicVertexConflicts(
    SCIP* scip,          // SCIP
    SCIP_CONS** cons,    // Pointer to hold the created constraint
    const char* name     // Name of constraint
)
{
    SCIP_CALL(SCIPcreateConsVertexConflicts(scip,
                                            cons,
                                            name,
                                            TRUE,
                                            TRUE,
                                            TRUE,
                                            TRUE,
                                            TRUE,
                                            FALSE,
                                            FALSE,
                                            FALSE,
                                            FALSE,
                                            FALSE));
    return SCIP_OKAY;
}

SCIP_RETCODE SCIPcreateConsVertexConflicts(
    SCIP* scip,                 // SCIP
    SCIP_CONS** cons,           // Pointer to hold the created constraint
    const char* name,           // Name of constraint
    SCIP_Bool initial,          // Should the LP relaxation of constraint be in the initial LP?
    SCIP_Bool separate,         // Should the constraint be separated during LP processing?
    SCIP_Bool enforce,          // Should the constraint be enforced during node processing?
    SCIP_Bool check,            // Should the constraint be checked for feasibility?
    SCIP_Bool propagate,        // Should the constraint be propagated during node processing?
    SCIP_Bool local,            // Is constraint only valid locally?
    SCIP_Bool modifiable,       // Is constraint modifiable (subject to column generation)?
    SCIP_Bool dynamic,          // Is constraint subject to aging?
    SCIP_Bool removable,        // Should the relaxation be removed from the LP due to aging or cleanup?
    SCIP_Bool stickingatnode    // Should the constraint always be kept at the node where it was added, even
                                // if it may be moved to a more global node?
)
{
    // Find constraint handler.
    SCIP_CONSHDLR* conshdlr = SCIPfindConshdlr(scip, CONSHDLR_NAME);
    release_assert(conshdlr, "Constraint handler for vertex conflicts is not found");

    // Create constraint data.
    VertexConflictsConsData* consdata = nullptr;
    SCIP_CALL(SCIPallocBlockMemory(scip, &consdata));
    debug_assert(consdata);
    new(consdata) VertexConflictsConsData;
    consdata->conflicts.reserve(5000);

    // Create constraint.
    SCIP_CALL(SCIPcreateCons(scip,
                             cons,
                             name,
                             conshdlr,
                             reinterpret_cast<SCIP_CONSDATA*>(consdata),
                             initial,
                             separate,
                             enforce,
                             check,
                             propagate,
                             local,
                             modifiable,
                             dynamic,
                             removable,
                             stickingatnode));

    // Done.
    return SCIP_OKAY;
}

SCIP_RETCODE vertex_conflicts_create_cut(
    SCIP* scip,                           // SCIP
    SCIP_CONS* cons,                      // Constraint
    VertexConflictsConsData* consdata,    // Constraint data
    const NodeTime nt,                    // Node-time of the conflict
    const Vector<SCIP_VAR*>& vars,        // Array of variables
    SCIP_Result* result                   // Output result
)
{
    // Create constraint name.
#ifdef DEBUG
    auto probdata = SCIPgetProbData(scip);
    const auto& map = SCIPprobdataGetMap(probdata);
    const auto [x, y] = map.get_xy(nt.n);
    const auto name = fmt::format("vertex_conflict(({},{}),{})", x, y, nt.t);
#endif

    // Create a row.
    SCIP_ROW* row = nullptr;
    SCIP_CALL(SCIPcreateEmptyRowCons(scip,
                                     &row,
                                     cons,
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
    for (auto var : vars)
    {
        // Get the path.
        debug_assert(var);
        auto vardata = SCIPvarGetData(var);
        const auto path_length = SCIPvardataGetPathLength(vardata);
        const auto path = SCIPvardataGetPath(vardata);

        // Add coefficients.
        if ((nt.t < path_length && path[nt.t].n == nt.n) ||
            (nt.t >= path_length && path[path_length - 1].n == nt.n))
        {
            // Print.
            debugln("      Agent: {:2d}, Val: {:7.4f}, Path: {}",
                    SCIPvardataGetAgent(vardata),
                    SCIPgetSolVal(scip, nullptr, var),
                    format_path_spaced(SCIPgetProbData(scip), path_length, path));

            // Add the coefficient.
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
    consdata->conflicts.push_back({row, nt});

    // Done.
    return SCIP_OKAY;
}

// Checker
static
SCIP_RETCODE vertex_conflicts_check(
    SCIP* scip,            // SCIP
    SCIP_SOL* sol,         // Solution
    SCIP_RESULT* result    // Pointer to store the result
)
{
    // Print.
    debugln("Starting checker for vertex conflicts on solution with obj {:.6f}:",
            SCIPgetSolOrigObj(scip, sol));

    // Get problem data.
    auto probdata = SCIPgetProbData(scip);

    // Get variables.
    const auto& vars = SCIPprobdataGetVars(probdata);

    // Find the makespan.
    Time makespan = 0;
    for (auto var : vars)
    {
        // Get the path length.
        debug_assert(var);
        auto vardata = SCIPvarGetData(var);
        const auto path_length = SCIPvardataGetPathLength(vardata);

        // Get the variable value.
        const auto var_val = SCIPgetSolVal(scip, sol, var);

        // Store the length of the longest path.
        if (path_length > makespan && SCIPisPositive(scip, var_val))
        {
            makespan = path_length;
        }
    }

    // Calculate the number of times a vertex is used by summing the columns.
    HashTable<NodeTime, SCIP_Real> vertex_times_used;
    for (auto var : vars)
    {
        // Get the path.
        debug_assert(var);
        auto vardata = SCIPvarGetData(var);
        const auto path_length = SCIPvardataGetPathLength(vardata);
        const auto path = SCIPvardataGetPath(vardata);

        // Get the variable value.
        const auto var_val = SCIPgetSolVal(scip, sol, var);

        // Sum vertex value.
        if (SCIPisPositive(scip, var_val))
        {
            Time t = 1;
            for (; t < path_length; ++t)
            {
                const NodeTime nt{path[t].n, t};
                vertex_times_used[nt] += var_val;
            }
            const auto n = path[path_length - 1].n;
            for (; t < makespan; ++t)
            {
                const NodeTime nt{n, t};
                vertex_times_used[nt] += var_val;
            }
        }
    }

    // Check for conflicts.
    for (const auto [nt, val] : vertex_times_used)
        if (SCIPisGT(scip, val, 1.0))
        {
            // Print.
#ifdef PRINT_DEBUG
            {
                const auto& map = SCIPprobdataGetMap(probdata);
                const auto [x, y] = map.get_xy(nt.n);
                debugln("   Infeasible solution has vertex (({},{}),{}) with value {}",
                        x, y, nt.t, val);
            }
#endif

            // Infeasible.
            *result = SCIP_INFEASIBLE;
            return SCIP_OKAY;
        }

    // Done.
    return SCIP_OKAY;
}

// Separator
static
SCIP_RETCODE vertex_conflicts_separate(
    SCIP* scip,                 // SCIP
    SCIP_CONS* cons,            // Constraint
    SCIP_SOL* sol,              // Solution
    SCIP_RESULT* result         // Pointer to store the result
)
{
    // Print.
    debugln("Starting separator for vertex conflicts on solution with obj {:.6f}:",
            SCIPgetSolOrigObj(scip, nullptr));

    // Print paths.
#ifdef PRINT_DEBUG
    print_used_paths(scip);
#endif

    // Get constraint data.
    auto consdata = reinterpret_cast<VertexConflictsConsData*>(SCIPconsGetData(cons));
    debug_assert(consdata);

    // Get problem data.
    auto probdata = SCIPgetProbData(scip);

    // Get variables.
    const auto& vars = SCIPprobdataGetVars(probdata);

    // Find the makespan.
    Time makespan = 0;
    for (auto var : vars)
    {
        // Get the path length.
        debug_assert(var);
        auto vardata = SCIPvarGetData(var);
        const auto path_length = SCIPvardataGetPathLength(vardata);

        // Get the variable value.
        const auto var_val = SCIPgetSolVal(scip, sol, var);

        // Store the length of the longest path.
        if (path_length > makespan && SCIPisPositive(scip, var_val))
        {
            makespan = path_length;
        }
    }

    // Calculate the number of times a vertex is used by summing the columns.
    HashTable<NodeTime, SCIP_Real> vertex_times_used;
    for (auto var : vars)
    {
        // Get the path.
        debug_assert(var);
        auto vardata = SCIPvarGetData(var);
        const auto path_length = SCIPvardataGetPathLength(vardata);
        const auto path = SCIPvardataGetPath(vardata);

        // Get the variable value.
        const auto var_val = SCIPgetSolVal(scip, sol, var);

        // Sum vertex value.
        if (SCIPisPositive(scip, var_val))
        {
            Time t = 1;
            for (; t < path_length; ++t)
            {
                const NodeTime nt{path[t].n, t};
                vertex_times_used[nt] += var_val;
            }
            const auto n = path[path_length - 1].n;
            for (; t < makespan; ++t)
            {
                const NodeTime nt{n, t};
                vertex_times_used[nt] += var_val;
            }
        }
    }

    // Create cuts.
    for (const auto [nt, val] : vertex_times_used)
        if (SCIPisGT(scip, val, 1.0))
        {
            // Print.
#ifdef PRINT_DEBUG
            {
                const auto& map = SCIPprobdataGetMap(probdata);
                const auto [x, y] = map.get_xy(nt.n);
                debugln("   Creating vertex conflict cut on (({},{}),{}) with value {} in branch-and-bound node {}",
                        x, y, nt.t, val, SCIPnodeGetNumber(SCIPgetCurrentNode(scip)));
            }
#endif

            // Create cut.
            SCIP_CALL(vertex_conflicts_create_cut(scip, cons, consdata, nt, vars, result));
        }

    // Done.
    return SCIP_OKAY;
}

// Copy method for constraint handler
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_CONSHDLRCOPY(conshdlrCopyVertexConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(conshdlr);
    debug_assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME) == 0);
    debug_assert(valid);

    // Include constraint handler.
    SCIP_CALL(SCIPincludeConshdlrVertexConflicts(scip));

    // Done.
    *valid = TRUE;
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Free constraint data
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_CONSDELETE(consDeleteVertexConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(conshdlr);
    debug_assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME) == 0);
    debug_assert(cons);
    debug_assert(consdata);
    debug_assert(*consdata);

    // Override type.
    auto consdata2 = reinterpret_cast<VertexConflictsConsData**>(consdata);

    // Free memory.
    (*consdata2)->~VertexConflictsConsData();
    SCIPfreeBlockMemory(scip, consdata2);

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Free rows
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_CONSEXITSOL(consExitsolVertexConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(conshdlr);
    debug_assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME) == 0);
    debug_assert(nconss == 0 || conss);

    // Loop through all constraints.
    for (Int c = 0; c < nconss; ++c)
    {
        // Get constraint.
        auto cons = conss[c];
        debug_assert(cons);

        // Get constraint data.
        auto consdata = reinterpret_cast<VertexConflictsConsData*>(SCIPconsGetData(cons));
        debug_assert(consdata);

        // Free row for each vertex conflict.
        for (auto [row, nt] : consdata->conflicts)
        {
            SCIP_CALL(SCIPreleaseRow(scip, &row));
        }
        consdata->conflicts.clear();
    }

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Transform constraint data into data belonging to the transformed problem
static
SCIP_DECL_CONSTRANS(consTransVertexConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(conshdlr);
    debug_assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME) == 0);
    debug_assert(sourcecons);
    debug_assert(targetcons);

    // Get data of original constraint.
    auto sourcedata =
        reinterpret_cast<VertexConflictsConsData*>(SCIPconsGetData(sourcecons));
    debug_assert(sourcedata);

    // Create constraint data.
    VertexConflictsConsData* targetdata = nullptr;
    SCIP_CALL(SCIPallocBlockMemory(scip, &targetdata));
    debug_assert(targetdata);
    new(targetdata) VertexConflictsConsData(*sourcedata);

    // Must begin with no vertex conflicts.
    release_assert(sourcedata->conflicts.empty(),
                   "Vertex conflicts exist in original problem before transformation");

    // Create constraint.
    char name[SCIP_MAXSTRLEN];
    SCIPsnprintf(name, SCIP_MAXSTRLEN, "t_%s", SCIPconsGetName(sourcecons));
    SCIP_CALL(SCIPcreateCons(scip,
                             targetcons,
                             name,
                             conshdlr,
                             reinterpret_cast<SCIP_CONSDATA*>(targetdata),
                             SCIPconsIsInitial(sourcecons),
                             SCIPconsIsSeparated(sourcecons),
                             SCIPconsIsEnforced(sourcecons),
                             SCIPconsIsChecked(sourcecons),
                             SCIPconsIsPropagated(sourcecons),
                             SCIPconsIsLocal(sourcecons),
                             SCIPconsIsModifiable(sourcecons),
                             SCIPconsIsDynamic(sourcecons),
                             SCIPconsIsRemovable(sourcecons),
                             SCIPconsIsStickingAtNode(sourcecons)));

    // Done.
    return SCIP_OKAY;
}

// Feasibility check method for integral solutions
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_CONSCHECK(consCheckVertexConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(conshdlr);
    debug_assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME) == 0);
    debug_assert(nconss == 0 || conss);
    debug_assert(result);

    // Start.
    *result = SCIP_FEASIBLE;

    // Start checker.
    debug_assert(sol);
    SCIP_CALL(vertex_conflicts_check(scip, sol, result));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Constraint enforcing method for LP solutions
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_CONSENFOLP(consEnfolpVertexConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(conshdlr);
    debug_assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME) == 0);
    debug_assert(conss);
    debug_assert(result);

    // Start.
    *result = SCIP_FEASIBLE;

    // Get constraint.
    debug_assert(nconss == 1);
    auto cons = conss[0];
    debug_assert(cons);

    // Start separator.
    SCIP_CALL(vertex_conflicts_separate(scip, cons, nullptr, result));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Constraint enforcing method for pseudo solutions
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_CONSENFOPS(consEnfopsVertexConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(conshdlr);
    debug_assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME) == 0);
    debug_assert(conss);
    debug_assert(result);

    // Start.
    *result = SCIP_FEASIBLE;

    // Start checker.
    SCIP_CALL(vertex_conflicts_check(scip, nullptr, result));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Separation method for LP solutions
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_CONSSEPALP(consSepalpVertexConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(conshdlr);
    debug_assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME) == 0);
    debug_assert(conss);
    debug_assert(result);

    // Start.
    *result = SCIP_DIDNOTFIND;

    // Get constraint.
    debug_assert(nconss == 1);
    auto cons = conss[0];
    debug_assert(cons);

    // Start separator.
    SCIP_CALL(vertex_conflicts_separate(scip, cons, nullptr, result));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Separation method for arbitrary primal solutions
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_CONSSEPASOL(consSepasolVertexConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(conshdlr);
    debug_assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME) == 0);
    debug_assert(conss);
    debug_assert(result);

    // Start.
    *result = SCIP_DIDNOTFIND;

    // Get constraint.
    debug_assert(nconss == 1);
    auto cons = conss[0];
    debug_assert(cons);

    // Start separator.
    debug_assert(sol);
    SCIP_CALL(vertex_conflicts_separate(scip, cons, sol, result));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Variable rounding lock method of constraint handler
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_CONSLOCK(consLockVertexConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(conshdlr);
    debug_assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME) == 0);
    debug_assert(cons);

    // Get problem data.
    auto probdata = SCIPgetProbData(scip);

    // Lock rounding of variables. (Round up may invalidate the constraint.)
    const auto& vars = SCIPprobdataGetVars(probdata);
    for (auto var : vars)
    {
        debug_assert(var);
        SCIP_CALL(SCIPaddVarLocksType(scip, var, locktype, nlocksneg, nlockspos));
    }

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Copying constraint of constraint handler
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_CONSCOPY(consCopyVertexConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sourceconshdlr);
    debug_assert(strcmp(SCIPconshdlrGetName(sourceconshdlr), CONSHDLR_NAME) == 0);
    debug_assert(cons);
    debug_assert(sourcescip);
    debug_assert(sourcecons);
    debug_assert(varmap);

    // Stop if invalid.
    if (*valid)
    {
        // Create copied constraint.
        if (!name)
        {
            name = SCIPconsGetName(sourcecons);
        }
        SCIP_CALL(SCIPcreateConsVertexConflicts(scip,
                                                cons,
                                                name,
                                                initial,
                                                separate,
                                                enforce,
                                                check,
                                                propagate,
                                                local,
                                                modifiable,
                                                dynamic,
                                                removable,
                                                stickingatnode));

        // Mark as valid.
        *valid = TRUE;
    }

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Creates constraint handler for vertex conflicts constraints and include it in SCIP
SCIP_RETCODE SCIPincludeConshdlrVertexConflicts(
    SCIP* scip    // SCIP
)
{
    // Include constraint handler.
    SCIP_CONSHDLR* conshdlr = nullptr;
    SCIP_CALL(SCIPincludeConshdlrBasic(scip,
                                       &conshdlr,
                                       CONSHDLR_NAME,
                                       CONSHDLR_DESC,
                                       CONSHDLR_ENFOPRIORITY,
                                       CONSHDLR_CHECKPRIORITY,
                                       CONSHDLR_EAGERFREQ,
                                       CONSHDLR_NEEDSCONS,
                                       consEnfolpVertexConflicts,
                                       consEnfopsVertexConflicts,
                                       consCheckVertexConflicts,
                                       consLockVertexConflicts,
                                       nullptr));
    debug_assert(conshdlr);

    // Set callbacks.
    SCIP_CALL(SCIPsetConshdlrDelete(scip,
                                    conshdlr,
                                    consDeleteVertexConflicts));
    SCIP_CALL(SCIPsetConshdlrExitsol(scip,
                                     conshdlr,
                                     consExitsolVertexConflicts));
    SCIP_CALL(SCIPsetConshdlrCopy(scip,
                                  conshdlr,
                                  conshdlrCopyVertexConflicts,
                                  consCopyVertexConflicts));
    SCIP_CALL(SCIPsetConshdlrTrans(scip,
                                   conshdlr,
                                   consTransVertexConflicts));
    SCIP_CALL(SCIPsetConshdlrSepa(scip,
                                  conshdlr,
                                  consSepalpVertexConflicts,
                                  consSepasolVertexConflicts,
                                  CONSHDLR_SEPAFREQ,
                                  CONSHDLR_SEPAPRIORITY,
                                  CONSHDLR_DELAYSEPA));

    // Done.
    return SCIP_OKAY;
}

SCIP_RETCODE vertex_conflicts_add_var(
    SCIP* scip,                 // SCIP
    SCIP_CONS* cons,            // Vertex conflicts constraint
    SCIP_VAR* var,              // Variable
    const Time path_length,     // Path length
    const Edge* const path      // Path
)
{
    // Get constraint data.
    debug_assert(cons);
    auto consdata = reinterpret_cast<VertexConflictsConsData*>(SCIPconsGetData(cons));
    debug_assert(consdata);
    auto& conflicts = consdata->conflicts;

    // Check.
    debug_assert(var);
    debug_assert(SCIPconsIsTransformed(cons));
    debug_assert(SCIPvarIsTransformed(var));

    // Add rounding lock to the new variable.
    SCIP_CALL(SCIPlockVarCons(scip, var, cons, FALSE, TRUE));

    // Add variable to constraints.
    for (auto [row, nt] : conflicts)
        if ((nt.t < path_length && path[nt.t].n == nt.n) ||
            (nt.t >= path_length && path[path_length - 1].n == nt.n))
        {
            SCIP_CALL(SCIPaddVarToRow(scip, row, var, 1.0));
        }

    // Return.
    return SCIP_OKAY;
}

const Vector<VertexConflict>& vertex_conflicts_get_constraints(
    SCIP_ProbData* probdata    // Problem data
)
{
    auto cons = SCIPprobdataGetVertexConflictsCons(probdata);
    debug_assert(cons);
    auto consdata = reinterpret_cast<VertexConflictsConsData*>(SCIPconsGetData(cons));
    debug_assert(consdata);
    return consdata->conflicts;
}
