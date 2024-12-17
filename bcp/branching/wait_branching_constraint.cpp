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

////#define PRINT_DEBUG
//
//#include "branching/wait_branching_constraint.h"
//#include "problem/problem.h"
//#include "problem/variable_data.h"
//
// Constraint handler properties
//#define CONSHDLR_NAME          "wait_branching"
//#define CONSHDLR_DESC          "Stores the wait branching decisions"
//#define CONSHDLR_ENFOPRIORITY  0          // priority of the constraint handler for constraint enforcing
//#define CONSHDLR_CHECKPRIORITY 9999999    // priority of the constraint handler for checking feasibility
//#define CONSHDLR_PROPFREQ      1          // frequency for propagating domains; zero means only preprocessing propagation
//#define CONSHDLR_EAGERFREQ     1          // frequency for using all instead of only the useful constraints in separation,
//                                          // propagation and enforcement, -1 for no eager evaluations, 0 for first only
//#define CONSHDLR_DELAYPROP     FALSE      // should propagation method be delayed, if other propagators found reductions?
//#define CONSHDLR_NEEDSCONS     TRUE       // should the constraint handler be skipped, if no constraints are available?
//
//#define CONSHDLR_PROP_TIMING   SCIP_PROPTIMING_BEFORELP
//
//// Constraint data
//struct WaitBranchingConsData
//{
//    WaitBranchDirection dir;    // Branch direction
//    Agent a;                    // Agent
//    Time t;                     // Time
//    Count npropagatedvars;      // Number of variables that existed when the related node
//                                // was propagated the last time. Used to determine whether
//                                // the constraint should be repropagated
//    bool propagated : 1;        // Is the constraint already propagated?
//    SCIP_NODE* node;            // The node of the branch-and-bound tree for this constraint
//};
//
//// Create constraint data
//static
//SCIP_RETCODE consdataCreate(
//    SCIP* scip,                          // SCIP
//    WaitBranchingConsData** consdata,    // Pointer to the constraint data
//    const WaitBranchDirection dir,       // Branch direction
//    const Agent a,                       // Agent
//    const Time t,                        // Time
//    SCIP_NODE* node                      // Node of the branch-and-bound tree for this constraint
//)
//{
//    // Check.
//    debug_assert(scip);
//    debug_assert(consdata);
//
//    // Create constraint data.
//    SCIP_CALL(SCIPallocBlockMemory(scip, consdata));
//
//    // Store data.
//    (*consdata)->dir = dir;
//    (*consdata)->a = a;
//    (*consdata)->t = t;
//    (*consdata)->npropagatedvars = 0;
//    (*consdata)->propagated = false;
//    (*consdata)->node = node;
//
//    // Done.
//    return SCIP_OKAY;
//}
//
//// Fix a variable to zero if its path is not valid for this constraint/branch
//static inline
//SCIP_RETCODE check_variable(
//    SCIP* scip,                       // SCIP
//    const WaitBranchDirection dir,    // Branch direction
//    const Agent a,                    // Agent
//    const Time t,                     // Time
//    SCIP_VAR* var,                    // Variable to check
//    Count& nfixedvars,                // Pointer to store the number of fixed variables
//    SCIP_Bool* cutoff                 // Pointer to store if a cutoff was detected
//)
//{
//    // Check.
//    debug_assert(scip);
//    debug_assert(var);
//    debug_assert(cutoff);
//
//    // If the variable is locally fixed to zero, continue to next variable.
//    if (SCIPvarGetUbLocal(var) < 0.5)
//        return SCIP_OKAY;
//
//    // Get the path.
//    auto vardata = SCIPvarGetData(var);
//    const auto path_length = SCIPvardataGetPathLength(vardata);
//    const auto path = SCIPvardataGetPath(vardata);
//    debug_assert(a == SCIPvardataGetAgent(vardata));
//
//    // Disable the path if the agent does or doesn't wait at the required time.
//    const auto exists = (t < path_length && path[t].d == Direction::WAIT);
//    if (exists != static_cast<bool>(dir))
//    {
//        // Disable the variable.
//        SCIP_Bool success;
//        SCIP_Bool fixing_is_infeasible;
//        SCIP_CALL(SCIPfixVar(scip, var, 0.0, &fixing_is_infeasible, &success));
//
//        // Print.
//        debugln("      Disabling variable for agent {}, path {}",
//                a, format_path(SCIPgetProbData(scip), path_length, path));
//
//        // Cut off the node if the fixing is infeasible.
//        if (fixing_is_infeasible)
//        {
//            debug_assert(SCIPvarGetLbLocal(var) > 0.5);
//            debugln("         Node is infeasible - cut off")
//            (*cutoff) = TRUE;
//        }
//        else
//        {
//            debug_assert(success);
//            nfixedvars++;
//        }
//    }
//
//    // Done.
//    return SCIP_OKAY;
//}
//
//// Fix variables to zero if its path is not valid for this constraint/branching
//static
//SCIP_RETCODE fix_variables(
//    SCIP* scip,                         // SCIP
//    WaitBranchingConsData* consdata,    // Constraint data
//    const Count nvars,                  // Number of variables
//    SCIP_VAR** vars,                    // Array of variables
//    SCIP_RESULT* result                 // Pointer to store the result of the fixing
//)
//{
//    // Print.
//    debugln("   Checking variables {} to {}:", consdata->npropagatedvars, nvars);
//
//    // Check.
//    debug_assert(consdata->npropagatedvars <= nvars);
//
//    // Check every variable.
//    Count nfixedvars = 0;
//    SCIP_Bool cutoff = FALSE;
//    for (Count var = consdata->npropagatedvars; var < nvars && !cutoff; ++var)
//    {
//        SCIP_CALL(check_variable(scip,
//                                 consdata->dir,
//                                 consdata->a,
//                                 consdata->t,
//                                 vars[var],
//                                 nfixedvars,
//                                 &cutoff));
//    }
//
//    // Print.
//    debugln("   Disabled {} variables", nfixedvars);
//
//    // Done.
//    if (cutoff)
//    {
//        *result = SCIP_CUTOFF;
//    }
//    else if (nfixedvars > 0)
//    {
//        *result = SCIP_REDUCEDDOM;
//    }
//    return SCIP_OKAY;
//}
//
//// Check if all variables are valid for the given constraint
//#ifdef DEBUG
//static
//void check_propagation(
//    SCIP_PROBDATA* probdata,            // Problem data
//    WaitBranchingConsData* consdata,    // Constraint data
//    SCIP_Bool beforeprop                // Is this check performed before propagation?
//)
//{
//    // Get the branching decision.
//    const auto a = consdata->a;
//    const auto t = consdata->t;
//    const auto dir = consdata->dir;
//
//    // Get variables.
//    const auto nvars = beforeprop ?
//                       consdata->npropagatedvars :
//                       SCIPprobdataGetAgentNVars(probdata)[a];
//    release_assert(nvars <= SCIPprobdataGetAgentNVars(probdata)[a]);
//    auto vars = SCIPprobdataGetAgentVars(probdata)[a];
//
//    // Check that the path of every variable is feasible for this constraint.
//    for (Count v = 0; v < nvars; ++v)
//    {
//        // Get variable.
//        auto var = vars[v];
//        debug_assert(var);
//
//        // If the variable is locally fixed to zero, continue.
//        if (SCIPvarGetUbLocal(var) < 0.5)
//            continue;
//
//        // Get the path.
//        auto vardata = SCIPvarGetData(var);
//        const auto path_length = SCIPvardataGetPathLength(vardata);
//        const auto path = SCIPvardataGetPath(vardata);
//        debug_assert(a == SCIPvardataGetAgent(vardata));
//
//        // Calculate condition.
//        const auto exists = (t < path_length && path[t].d == Direction::WAIT);
//
//        // Check.
//        release_assert(!(exists != static_cast<bool>(dir)),
//                       "Branching decision is not propagated correctly for agent {},"
//                       "path {}",
//                       a, format_path(probdata, path_length, path));
//    }
//}
//#endif
//
//// Free constraint data
//static
//SCIP_DECL_CONSDELETE(consDeleteWaitBranching)
//{
//    // Check.
//    debug_assert(conshdlr);
//    debug_assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME) == 0);
//    debug_assert(consdata);
//    debug_assert(*consdata);
//
//    // Free memory.
//    SCIPfreeBlockMemory(scip, reinterpret_cast<WaitBranchingConsData**>(consdata));
//
//    // Done.
//    return SCIP_OKAY;
//}
//
//// Transform constraint data into data belonging to the transformed problem
//static
//SCIP_DECL_CONSTRANS(consTransWaitBranching)
//{
//    // Check.
//    debug_assert(conshdlr);
//    debug_assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME) == 0);
//    debug_assert(SCIPgetStage(scip) == SCIP_STAGE_TRANSFORMING);
//    debug_assert(sourcecons);
//    debug_assert(targetcons);
//
//    // Get data of original constraint.
//    auto sourcedata =
//        reinterpret_cast<WaitBranchingConsData*>(SCIPconsGetData(sourcecons));
//    debug_assert(sourcedata);
//
//    // Create constraint data for target constraint.
//    WaitBranchingConsData* targetdata;
//    SCIP_CALL(consdataCreate(scip,
//                             &targetdata,
//                             sourcedata->dir,
//                             sourcedata->a,
//                             sourcedata->t,
//                             sourcedata->node));
//    debug_assert(targetdata);
//
//    // Create target constraint.
//    char name[SCIP_MAXSTRLEN];
//    SCIPsnprintf(name, SCIP_MAXSTRLEN, "t_%s", SCIPconsGetName(sourcecons));
//    SCIP_CALL(SCIPcreateCons(scip,
//                             targetcons,
//                             SCIPconsGetName(sourcecons),
//                             conshdlr,
//                             reinterpret_cast<SCIP_CONSDATA*>(targetdata),
//                             SCIPconsIsInitial(sourcecons),
//                             SCIPconsIsSeparated(sourcecons),
//                             SCIPconsIsEnforced(sourcecons),
//                             SCIPconsIsChecked(sourcecons),
//                             SCIPconsIsPropagated(sourcecons),
//                             SCIPconsIsLocal(sourcecons),
//                             SCIPconsIsModifiable(sourcecons),
//                             SCIPconsIsDynamic(sourcecons),
//                             SCIPconsIsRemovable(sourcecons),
//                             SCIPconsIsStickingAtNode(sourcecons)));
//
//    // Done.
//    return SCIP_OKAY;
//}
//
//// Domain propagation method of constraint handler
//static
//SCIP_DECL_CONSPROP(consPropWaitBranching)
//{
//    // Check.
//    debug_assert(scip);
//    debug_assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME) == 0);
//    debug_assert(result);
//
//    // Get problem data.
//    auto probdata = SCIPgetProbData(scip);
//    const auto& agent_vars = SCIPprobdataGetAgentVars(probdata);
//    const auto nvars = agent_vars.size();
//
//    // Propagate constraints.
//    *result = SCIP_DIDNOTFIND;
//    for (Count c = 0; c < nconss; ++c)
//    {
//        // Get constraint data.
//        auto consdata =
//            reinterpret_cast<WaitBranchingConsData*>(SCIPconsGetData(conss[c]));
//        debug_assert(consdata);
//        const auto a = consdata->a;
//
//        // Check if all variables are valid for this constraint.
//#ifdef DEBUG
//        check_propagation(probdata, consdata, TRUE);
//#endif
//
//        // Propagate.
//        if (!consdata->propagated)
//        {
//            // Print.
//#ifdef PRINT_DEBUG
//            debugln("Propagating wait branching constraint branch_{}({},{}) from node {} "
//                    "at depth {} in node {} at depth {}",
//                    consdata->dir == WaitBranchDirection::MustWait ?
//                        "must_wait" : "cannot_wait",
//                    consdata->a,
//                    consdata->t,
//                    SCIPnodeGetNumber(consdata->node),
//                    SCIPnodeGetDepth(consdata->node),
//                    SCIPnodeGetNumber(SCIPgetCurrentNode(scip)),
//                    SCIPgetDepth(scip));
//#endif
//
//            // Propagate.
//            SCIP_CALL(fix_variables(scip,
//                                    consdata,
//                                    agent_nvars[a],
//                                    agent_vars[a],
//                                    result));
//
//            // Set status.
//            if (*result != SCIP_CUTOFF)
//            {
//                consdata->propagated = TRUE;
//                debug_assert(consdata->npropagatedvars <= agent_nvars[a]);
//                consdata->npropagatedvars = agent_nvars[a];
//            }
//            else
//            {
//                break;
//            }
//        }
//
//        // Check if constraint is completely propagated.
//#ifdef DEBUG
//        check_propagation(probdata, consdata, FALSE);
//#endif
//    }
//
//    // Done.
//    return SCIP_OKAY;
//}
//
//// Constraint activation notification method of constraint handler
//static
//SCIP_DECL_CONSACTIVE(consActiveWaitBranching)
//{
//    // Check.
//    debug_assert(scip);
//    debug_assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME) == 0);
//    debug_assert(cons);
//
//    // Get problem data.
//    auto probdata = SCIPgetProbData(scip);
//    auto agent_nvars = SCIPprobdataGetAgentNVars(probdata);
//
//    // Get constraint data.
//    auto consdata = reinterpret_cast<WaitBranchingConsData*>(SCIPconsGetData(cons));
//    debug_assert(consdata);
//    debug_assert(consdata->npropagatedvars <= agent_nvars[consdata->a]);
//
//    // Print.
//#ifdef PRINT_DEBUG
//    debugln("Activating wait branching constraint branch_{}({},{}) from node {} at depth "
//            "{} in node {} at depth {}",
//            consdata->dir == WaitBranchDirection::MustWait ? "must_wait" : "cannot_wait",
//            consdata->a,
//            consdata->t,
//            SCIPnodeGetNumber(consdata->node),
//            SCIPnodeGetDepth(consdata->node),
//            SCIPnodeGetNumber(SCIPgetCurrentNode(scip)),
//            SCIPgetDepth(scip));
//#endif
//
//    // Mark constraint as to be repropagated.
//    if (consdata->npropagatedvars != agent_nvars[consdata->a])
//    {
//        consdata->propagated = FALSE;
//        SCIP_CALL(SCIPrepropagateNode(scip, consdata->node));
//    }
//
//    // Done.
//    return SCIP_OKAY;
//}
//
//// Constraint deactivation notification method of constraint handler
//static
//SCIP_DECL_CONSDEACTIVE(consDeactiveWaitBranching)
//{
//    // Check.
//    debug_assert(scip);
//    debug_assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME) == 0);
//    debug_assert(cons);
//
//    // Get problem data.
//    auto probdata = SCIPgetProbData(scip);
//    auto agent_nvars = SCIPprobdataGetAgentNVars(probdata);
//
//    // Get constraint data.
//    auto consdata = reinterpret_cast<WaitBranchingConsData*>(SCIPconsGetData(cons));
//    debug_assert(consdata);
//    debug_assert(consdata->propagated || SCIPgetNChildren(scip) == 0);
//
//    // Print.
//#ifdef PRINT_DEBUG
//    debugln("Deactivating wait branching constraint branch_{}({},{})",
//            consdata->dir == WaitBranchDirection::MustWait ? "must_wait" : "cannot_wait",
//            consdata->a,
//            consdata->t);
//#endif
//
//    // Set the number of propagated variables to the current number of variables.
//    consdata->npropagatedvars = agent_nvars[consdata->a];
//
//    // Done.
//    return SCIP_OKAY;
//}
//
//// Create the constraint handler for a branch and include it
//SCIP_RETCODE SCIPincludeConshdlrWaitBranching(
//    SCIP* scip    // SCIP
//)
//{
//    // Include constraint handler.
//    SCIP_CONSHDLR* conshdlr = nullptr;
//    SCIP_CALL(SCIPincludeConshdlrBasic(scip,
//                                       &conshdlr,
//                                       CONSHDLR_NAME,
//                                       CONSHDLR_DESC,
//                                       CONSHDLR_ENFOPRIORITY,
//                                       CONSHDLR_CHECKPRIORITY,
//                                       CONSHDLR_EAGERFREQ,
//                                       CONSHDLR_NEEDSCONS,
//                                       nullptr,
//                                       nullptr,
//                                       nullptr,
//                                       nullptr,
//                                       nullptr));
//    debug_assert(conshdlr);
//
//    // Set callbacks.
//    SCIP_CALL(SCIPsetConshdlrDelete(scip, conshdlr, consDeleteWaitBranching));
//    SCIP_CALL(SCIPsetConshdlrTrans(scip, conshdlr, consTransWaitBranching));
//    SCIP_CALL(SCIPsetConshdlrProp(scip,
//                                  conshdlr,
//                                  consPropWaitBranching,
//                                  CONSHDLR_PROPFREQ,
//                                  CONSHDLR_DELAYPROP,
//                                  CONSHDLR_PROP_TIMING));
//    SCIP_CALL(SCIPsetConshdlrActive(scip, conshdlr, consActiveWaitBranching));
//    SCIP_CALL(SCIPsetConshdlrDeactive(scip, conshdlr, consDeactiveWaitBranching));
//
//    // Done.
//    return SCIP_OKAY;
//}
//
//// Create and capture a constraint enforcing a branch
//SCIP_RETCODE SCIPcreateConsWaitBranching(
//    SCIP* scip,                       // SCIP
//    SCIP_CONS** cons,                 // Pointer to the created constraint
//    const char* name,                 // Name of constraint
//    const WaitBranchDirection dir,    // Branch direction
//    const Agent a,                    // Agent
//    const Time t,                     // Time
//    SCIP_NODE* node,                  // The node of the branch-and-bound tree for this constraint
//    SCIP_Bool local                   // Is this constraint only valid locally?
//)
//{
//    // Find the constraint handler.
//    auto conshdlr = SCIPfindConshdlr(scip, CONSHDLR_NAME);
//    debug_assert(conshdlr);
//
//    // Create constraint data.
//    WaitBranchingConsData* consdata;
//    SCIP_CALL(consdataCreate(scip, &consdata, dir, a, t, node));
//    debug_assert(consdata);
//
//    // Create constraint.
//    SCIP_CALL(SCIPcreateCons(scip,
//                             cons,
//                             name,
//                             conshdlr,
//                             reinterpret_cast<SCIP_CONSDATA*>(consdata),
//                             FALSE,
//                             FALSE,
//                             FALSE,
//                             FALSE,
//                             TRUE,
//                             local,
//                             FALSE,
//                             FALSE,
//                             FALSE,
//                             TRUE));
//
//    // Print.
//#ifdef PRINT_DEBUG
//    debugln("Creating wait branching constraint branch_{}({},{}) in node {} at depth {} "
//            "(parent {})",
//            consdata->dir == WaitBranchDirection::MustWait ? "must_wait" : "cannot_wait",
//            consdata->a,
//            consdata->t,
//            SCIPnodeGetNumber(consdata->node),
//            SCIPnodeGetDepth(consdata->node),
//            SCIPnodeGetNumber(SCIPnodeGetParent(consdata->node)));
//#endif
//
//    // Done.
//    return SCIP_OKAY;
//}
//
//// Get branch direction
//WaitBranchDirection SCIPgetWaitBranchingDirection(
//    SCIP_CONS* cons    // Constraint enforcing wait branching
//)
//{
//    debug_assert(cons);
//    auto consdata = reinterpret_cast<WaitBranchingConsData*>(SCIPconsGetData(cons));
//    debug_assert(consdata);
//    return consdata->dir;
//}
//
//// Get agent
//Agent SCIPgetWaitBranchingAgent(
//    SCIP_CONS* cons    // Constraint enforcing wait branching
//)
//{
//    debug_assert(cons);
//    auto consdata = reinterpret_cast<WaitBranchingConsData*>(SCIPconsGetData(cons));
//    debug_assert(consdata);
//    return consdata->a;
//}
//
//// Get time
//Time SCIPgetWaitBranchingTime(
//    SCIP_CONS* cons    // Constraint enforcing wait branching
//)
//{
//    debug_assert(cons);
//    auto consdata = reinterpret_cast<WaitBranchingConsData*>(SCIPconsGetData(cons));
//    debug_assert(consdata);
//    return consdata->t;
//}
