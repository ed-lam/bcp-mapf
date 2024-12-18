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

//#ifndef MAPF_CONSTRAINT_WAITBRANCHING_H
//#define MAPF_CONSTRAINT_WAITBRANCHING_H
//
//#include "problem/includes.h"
//#include "types/map_types.h"
//
//enum WaitBranchDirection : bool
//{
//    CannotWait = false,
//    MustWait = true
//};
//
//// Create the constraint handler for a branch and include it
//SCIP_RETCODE SCIPincludeConshdlrWaitBranching(
//    SCIP* scip    // SCIP
//);
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
//);
//
//// Get branch direction
//WaitBranchDirection SCIPgetWaitBranchingDirection(
//    SCIP_CONS* cons    // Constraint enforcing wait branching
//);
//
//// Get agent
//Agent SCIPgetWaitBranchingAgent(
//    SCIP_CONS* cons    // Constraint enforcing wait branching
//);
//
//// Get time
//Time SCIPgetWaitBranchingTime(
//    SCIP_CONS* cons    // Constraint enforcing wait branching
//);
//
//#endif
