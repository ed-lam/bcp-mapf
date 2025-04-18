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

#include "branching/vertex_branching_constraint.h"
#include "problem/problem.h"
#include "problem/variable_data.h"

// Constraint handler properties
#define CONSHDLR_NAME          "vertex_branching"
#define CONSHDLR_DESC          "Stores the vertex branching decisions"
#define CONSHDLR_ENFOPRIORITY  0          // priority of the constraint handler for constraint enforcing
#define CONSHDLR_CHECKPRIORITY 9999999    // priority of the constraint handler for checking feasibility
#define CONSHDLR_PROPFREQ      1          // frequency for propagating domains; zero means only preprocessing propagation
#define CONSHDLR_EAGERFREQ     1          // frequency for using all instead of only the useful constraints in separation,
                                          // propagation and enforcement, -1 for no eager evaluations, 0 for first only
#define CONSHDLR_DELAYPROP     FALSE      // should propagation method be delayed, if other propagators found reductions?
#define CONSHDLR_NEEDSCONS     TRUE       // should the constraint handler be skipped, if no constraints are available?

#define CONSHDLR_PROP_TIMING   SCIP_PROPTIMING_BEFORELP

// Constraint data
struct VertexBranchingConsData
{
    VertexBranchDirection dir;    // Branch direction
    Agent a;                      // Agent
    NodeTime nt;                  // Node-time
    Int npropagatedvars;          // Number of variables that existed when the related node
                                  // was propagated the last time. Used to determine whether
                                  // the constraint should be repropagated
    bool propagated : 1;          // Is the constraint already propagated?
    SCIP_NODE* node;              // The node of the branch-and-bound tree for this constraint
};

// Create constraint data
static
SCIP_RETCODE consdataCreate(
    SCIP* scip,                            // SCIP
    VertexBranchingConsData** consdata,    // Pointer to the constraint data
    const VertexBranchDirection dir,       // Branch direction
    const Agent a,                         // Agent
    const NodeTime nt,                     // Node-time
    SCIP_NODE* node                        // Node of the branch-and-bound tree for this constraint
)
{
    // Check.
    debug_assert(scip);
    debug_assert(consdata);

    // Create constraint data.
    SCIP_CALL(SCIPallocBlockMemory(scip, consdata));

    // Store data.
    (*consdata)->dir = dir;
    (*consdata)->a = a;
    (*consdata)->nt = nt;
    (*consdata)->npropagatedvars = 0;
    (*consdata)->propagated = false;
    (*consdata)->node = node;

    // Done.
    return SCIP_OKAY;
}

// Fix a variable to zero if its path is not valid for this constraint/branch
static inline
SCIP_RETCODE check_variable(
    SCIP* scip,                         // SCIP
    const VertexBranchDirection dir,    // Branch direction
    const Agent a,                      // Agent
    const NodeTime nt,                  // Node-time
    SCIP_VAR* var,                      // Variable to check
    Int& nfixedvars,                    // Pointer to store the number of fixed variables
    SCIP_Bool* cutoff                   // Pointer to store if a cutoff was detected
)
{
    // Check.
    debug_assert(scip);
    debug_assert(var);
    debug_assert(cutoff);

    // If the variable is locally fixed to zero, continue to next variable.
    if (SCIPvarGetUbLocal(var) < 0.5)
        return SCIP_OKAY;

    // Get the path.
    auto vardata = SCIPvarGetData(var);
    const auto path_a = SCIPvardataGetAgent(vardata);
    const auto path_length = SCIPvardataGetPathLength(vardata);
    const auto path = SCIPvardataGetPath(vardata);

    // Disable a path of the same agent if it doesn't visit the vertex at the timestep.
    // Disable a path of a different agent if the branch direction requires the vertex and
    // this agent uses the vertex.
    const auto disable_path_of_same_agent = (
        path_a == a &&
        (nt.t < path_length && path[nt.t].n == nt.n) != static_cast<bool>(dir)
    );
    const auto disable_path_of_diff_agent = (
        path_a != a &&
        dir == VertexBranchDirection::Use &&
        (nt.t < path_length && path[nt.t].n == nt.n)
    );

    // Disable the variable if its path doesn't satisfy the branching decision.
    if (disable_path_of_same_agent || disable_path_of_diff_agent)
    {
        // Disable the variable.
        SCIP_Bool success;
        SCIP_Bool fixing_is_infeasible;
        SCIP_CALL(SCIPfixVar(scip, var, 0.0, &fixing_is_infeasible, &success));

        // Print.
        debugln("      Disabling variable for agent {}, path {}",
                path_a, format_path(SCIPgetProbData(scip), path_length, path));

        // Cut off the node if the fixing is infeasible.
        if (fixing_is_infeasible)
        {
            debug_assert(SCIPvarGetLbLocal(var) > 0.5);
            debugln("         Node is infeasible - cut off");
            (*cutoff) = TRUE;
        }
        else
        {
            debug_assert(success);
            nfixedvars++;
        }
    }

    // Done.
    return SCIP_OKAY;
}

// Fix variables to zero if its path is not valid for this constraint/branching
static
SCIP_RETCODE fix_variables(
    SCIP* scip,                                        // SCIP
    VertexBranchingConsData* consdata,                 // Constraint data
    const Vector<Pair<SCIP_VAR*, SCIP_Real>>& vars,    // Array of variables
    SCIP_RESULT* result                                // Pointer to store the result of the fixing
)
{
    // Print.
    debugln("   Checking variables {} to {}:", consdata->npropagatedvars, vars.size());

    // Check.
    const auto nvars = static_cast<Int>(vars.size());
    debug_assert(consdata->npropagatedvars <= nvars);

    // Check every variable.
    Int nfixedvars = 0;
    SCIP_Bool cutoff = FALSE;
    for (Int v = consdata->npropagatedvars; v < nvars && !cutoff; ++v)
    {
        debug_assert(v < nvars);
        SCIP_CALL(check_variable(scip,
                                 consdata->dir,
                                 consdata->a,
                                 consdata->nt,
                                 vars[v].first,
                                 nfixedvars,
                                 &cutoff));
    }

    // Print.
    debugln("   Disabled {} variables", nfixedvars);

    // Done.
    if (cutoff)
    {
        *result = SCIP_CUTOFF;
    }
    else if (nfixedvars > 0)
    {
        *result = SCIP_REDUCEDDOM;
    }
    return SCIP_OKAY;
}

// Check if all variables are valid for the given constraint
#ifdef DEBUG
static
void check_propagation(
    SCIP_PROBDATA* probdata,              // Problem data
    VertexBranchingConsData* consdata,    // Constraint data
    SCIP_Bool beforeprop                  // Is this check performed before propagation?
)
{
    // Get problem data.
    const auto& map = SCIPprobdataGetMap(probdata);

    // Get variables.
    const auto& vars = SCIPprobdataGetVars(probdata);
    const auto nvars = beforeprop ? consdata->npropagatedvars : static_cast<Int>(vars.size());
    release_assert(nvars <= static_cast<Int>(vars.size()));

    // Get the branching decision.
    const auto a = consdata->a;
    const auto nt = consdata->nt;
    const auto dir = consdata->dir;

    // Check that the path of every variable is feasible for this constraint.
    for (Int v = 0; v < nvars; ++v)
    {
        // Get variable.
        debug_assert(v < static_cast<Int>(vars.size()));
        const auto var = vars[v].first;
        debug_assert(var);

        // If the variable is locally fixed to zero, continue.
        if (SCIPvarGetUbLocal(var) < 0.5)
            continue;

        // Get the path.
        auto vardata = SCIPvarGetData(var);
        const auto path_a = SCIPvardataGetAgent(vardata);
        const auto path_length = SCIPvardataGetPathLength(vardata);
        const auto path = SCIPvardataGetPath(vardata);

        // Calculate condition.
        const auto disable_path_of_same_agent = (
            path_a == a &&
            (nt.t < path_length && path[nt.t].n == nt.n) != static_cast<bool>(dir)
        );
        const auto disable_path_of_diff_agent = (
            path_a != a &&
            dir == VertexBranchDirection::Use &&
            (nt.t < path_length && path[nt.t].n == nt.n)
        );

        // Check.
        release_assert(!(disable_path_of_same_agent || disable_path_of_diff_agent),
                       "Branching decision to {} agent {} at (({},{}),{}) is not propagated correctly for agent {}, "
                       "path {}",
                       dir == VertexBranchDirection::Use ? "use" : "forbid",
                       a,
                       map.get_x(nt.n),
                       map.get_y(nt.n),
                       nt.t,
                       path_a,
                       format_path(probdata, path_length, path));
    }
}
#endif

// Free constraint data
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_CONSDELETE(consDeleteVertexBranching)
{
    // Check.
    debug_assert(conshdlr);
    debug_assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME) == 0);
    debug_assert(consdata);
    debug_assert(*consdata);

    // Free memory.
    SCIPfreeBlockMemory(scip, reinterpret_cast<VertexBranchingConsData**>(consdata));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Transform constraint data into data belonging to the transformed problem
static
SCIP_DECL_CONSTRANS(consTransVertexBranching)
{
    // Check.
    debug_assert(conshdlr);
    debug_assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME) == 0);
    debug_assert(SCIPgetStage(scip) == SCIP_STAGE_TRANSFORMING);
    debug_assert(sourcecons);
    debug_assert(targetcons);

    // Get data of original constraint.
    auto sourcedata =
        reinterpret_cast<VertexBranchingConsData*>(SCIPconsGetData(sourcecons));
    debug_assert(sourcedata);

    // Create constraint data for target constraint.
    VertexBranchingConsData* targetdata;
    SCIP_CALL(consdataCreate(scip,
                             &targetdata,
                             sourcedata->dir,
                             sourcedata->a,
                             sourcedata->nt,
                             sourcedata->node));
    debug_assert(targetdata);

    // Create target constraint.
    char name[SCIP_MAXSTRLEN];
    SCIPsnprintf(name, SCIP_MAXSTRLEN, "t_%s", SCIPconsGetName(sourcecons));
    SCIP_CALL(SCIPcreateCons(scip,
                             targetcons,
                             SCIPconsGetName(sourcecons),
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

// Domain propagation method of constraint handler
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_CONSPROP(consPropVertexBranching)
{
    // Check.
    debug_assert(scip);
    debug_assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME) == 0);
    debug_assert(result);

    // Get problem data.
    auto probdata = SCIPgetProbData(scip);

    // Get variables.
    const auto& vars = SCIPprobdataGetVars(probdata);
    const auto nvars = static_cast<Int>(vars.size());

    // Propagate constraints.
    *result = SCIP_DIDNOTFIND;
    for (Int c = 0; c < nconss; ++c)
    {
        // Get constraint data.
        auto consdata = reinterpret_cast<VertexBranchingConsData*>(SCIPconsGetData(conss[c]));
        debug_assert(consdata);

        // Check if all variables are valid for this constraint.
#ifdef DEBUG
        check_propagation(probdata, consdata, TRUE);
#endif

        // Propagate.
#ifndef USE_LNS2_REPAIR_PRIMAL_HEURISTIC
        if (!consdata->propagated)
#endif
        {
            // Print.
#ifdef PRINT_DEBUG
            {
                const auto& map = SCIPprobdataGetMap(probdata);
                const auto [x, y] = map.get_xy(consdata->nt.n);
                debugln("Propagating vertex branching constraint branch_{}({},({},{}),{})"
                        " from node {} at depth {} in node {} at depth {}",
                        consdata->dir == VertexBranchDirection::Use ? "use" : "forbid",
                        consdata->a,
                        x, y,
                        consdata->nt.t,
                        SCIPnodeGetNumber(consdata->node),
                        SCIPnodeGetDepth(consdata->node),
                        SCIPnodeGetNumber(SCIPgetCurrentNode(scip)),
                        SCIPgetDepth(scip));
            }
#endif

            // Propagate.
            SCIP_CALL(fix_variables(scip, consdata, vars, result));

            // Set status.
            if (*result != SCIP_CUTOFF)
            {
                consdata->propagated = TRUE;
                debug_assert(consdata->npropagatedvars <= nvars);
                consdata->npropagatedvars = nvars;
            }
            else
            {
                break;
            }
        }

        // Check if constraint is completely propagated.
#ifdef DEBUG
        check_propagation(probdata, consdata, FALSE);
#endif
    }

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Constraint activation notification method of constraint handler
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_CONSACTIVE(consActiveVertexBranching)
{
    // Check.
    debug_assert(scip);
    debug_assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME) == 0);
    debug_assert(cons);

    // Get problem data.
    auto probdata = SCIPgetProbData(scip);

    // Get variables.
    const auto& vars = SCIPprobdataGetVars(probdata);
    const auto nvars = static_cast<Int>(vars.size());

    // Get constraint data.
    auto consdata = reinterpret_cast<VertexBranchingConsData*>(SCIPconsGetData(cons));
    debug_assert(consdata);
    debug_assert(consdata->npropagatedvars <= nvars);

    // Print.
#ifdef PRINT_DEBUG
    {
        const auto& map = SCIPprobdataGetMap(probdata);
        const auto [x, y] = map.get_xy(consdata->nt.n);
        debugln("Activating vertex branching constraint branch_{}({},({},{}),{}) from "
                "node {} at depth {} in node {} at depth {}",
                consdata->dir == VertexBranchDirection::Use ? "use" : "forbid",
                consdata->a,
                x, y,
                consdata->nt.t,
                SCIPnodeGetNumber(consdata->node),
                SCIPnodeGetDepth(consdata->node),
                SCIPnodeGetNumber(SCIPgetCurrentNode(scip)),
                SCIPgetDepth(scip));
    }
#endif

    // Mark constraint as to be repropagated.
    if (consdata->npropagatedvars != nvars)
    {
        consdata->propagated = FALSE;
        SCIP_CALL(SCIPrepropagateNode(scip, consdata->node));
    }

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Constraint deactivation notification method of constraint handler
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_CONSDEACTIVE(consDeactiveVertexBranching)
{
    // Check.
    debug_assert(scip);
    debug_assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME) == 0);
    debug_assert(cons);

    // Get problem data.
    auto probdata = SCIPgetProbData(scip);

    // Get constraint data.
    auto consdata = reinterpret_cast<VertexBranchingConsData*>(SCIPconsGetData(cons));
    debug_assert(consdata);
    debug_assert(consdata->propagated || SCIPgetNChildren(scip) == 0);

    // Print.
#ifdef PRINT_DEBUG
    {
        const auto& map = SCIPprobdataGetMap(probdata);
        const auto [x, y] = map.get_xy(consdata->nt.n);
        debugln("Deactivating vertex branching constraint branch_{}({},({},{}),{})",
                consdata->dir == VertexBranchDirection::Use ? "use" : "forbid",
                consdata->a,
                x, y,
                consdata->nt.t);
    }
#endif

    // Set the number of propagated variables to the current number of variables because the new variables always
    // respect the constraint (branching decision) unless using the LNS2 primal heuristic which can add variables that
    // ignore the branching decisions.
#ifndef USE_LNS2_REPAIR_PRIMAL_HEURISTIC
    const auto& vars = SCIPprobdataGetVars(probdata);
    consdata->npropagatedvars = vars.size();
#endif

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Create the constraint handler for a branch and include it
SCIP_RETCODE SCIPincludeConshdlrVertexBranching(
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
                                       nullptr,
                                       nullptr,
                                       nullptr,
                                       nullptr,
                                       nullptr));
    debug_assert(conshdlr);

    // Set callbacks.
    SCIP_CALL(SCIPsetConshdlrDelete(scip, conshdlr, consDeleteVertexBranching));
    SCIP_CALL(SCIPsetConshdlrTrans(scip, conshdlr, consTransVertexBranching));
    SCIP_CALL(SCIPsetConshdlrProp(scip,
                                  conshdlr,
                                  consPropVertexBranching,
                                  CONSHDLR_PROPFREQ,
                                  CONSHDLR_DELAYPROP,
                                  CONSHDLR_PROP_TIMING));
    SCIP_CALL(SCIPsetConshdlrActive(scip, conshdlr, consActiveVertexBranching));
    SCIP_CALL(SCIPsetConshdlrDeactive(scip, conshdlr, consDeactiveVertexBranching));

    // Done.
    return SCIP_OKAY;
}

// Create and capture a constraint enforcing a branch
SCIP_RETCODE SCIPcreateConsVertexBranching(
    SCIP* scip,                         // SCIP
    SCIP_CONS** cons,                   // Pointer to the created constraint
    const char* name,                   // Name of constraint
    const VertexBranchDirection dir,    // Branch direction
    const Agent a,                      // Agent
    const NodeTime nt,                  // Node-time
    SCIP_NODE* node,                    // The node of the branch-and-bound tree for this constraint
    SCIP_Bool local                     // Is this constraint only valid locally?
)
{
    // Find the constraint handler.
    auto conshdlr = SCIPfindConshdlr(scip, CONSHDLR_NAME);
    debug_assert(conshdlr);

    // Create constraint data.
    VertexBranchingConsData* consdata;
    SCIP_CALL(consdataCreate(scip, &consdata, dir, a, nt, node));
    debug_assert(consdata);

    // Create constraint.
    SCIP_CALL(SCIPcreateCons(scip,
                             cons,
                             name,
                             conshdlr,
                             reinterpret_cast<SCIP_CONSDATA*>(consdata),
                             FALSE,
                             FALSE,
                             FALSE,
                             FALSE,
                             TRUE,
                             local,
                             FALSE,
                             FALSE,
                             FALSE,
                             TRUE));

    // Print.
#ifdef PRINT_DEBUG
    {
        auto probdata = SCIPgetProbData(scip);
        const auto& map = SCIPprobdataGetMap(probdata);
        const auto [x, y] = map.get_xy(consdata->nt.n);
        debugln("Creating vertex branching constraint branch_{}({},({},{}),{}) in node "
                "{} at depth {} (parent {})",
                consdata->dir == VertexBranchDirection::Use ? "use" : "forbid",
                consdata->a,
                x, y,
                consdata->nt.t,
                SCIPnodeGetNumber(consdata->node),
                SCIPnodeGetDepth(consdata->node),
                SCIPnodeGetNumber(SCIPnodeGetParent(consdata->node)));
    }
#endif

    // Done.
    return SCIP_OKAY;
}

// Get branch direction
VertexBranchDirection SCIPgetVertexBranchingDirection(
    SCIP_CONS* cons    // Constraint enforcing vertex branching
)
{
    debug_assert(cons);
    auto consdata = reinterpret_cast<VertexBranchingConsData*>(SCIPconsGetData(cons));
    debug_assert(consdata);
    return consdata->dir;
}

// Get agent
Agent SCIPgetVertexBranchingAgent(
    SCIP_CONS* cons    // Constraint enforcing vertex branching
)
{
    debug_assert(cons);
    auto consdata = reinterpret_cast<VertexBranchingConsData*>(SCIPconsGetData(cons));
    debug_assert(consdata);
    return consdata->a;
}

// Get node-time
NodeTime SCIPgetVertexBranchingNodeTime(
    SCIP_CONS* cons    // Constraint enforcing vertex branching
)
{
    debug_assert(cons);
    auto consdata = reinterpret_cast<VertexBranchingConsData*>(SCIPconsGetData(cons));
    debug_assert(consdata);
    return consdata->nt;
}
