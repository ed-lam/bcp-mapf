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

#ifndef MAPF_CONSTRAINT_VERTEXBRANCHING_H
#define MAPF_CONSTRAINT_VERTEXBRANCHING_H

#include "Includes.h"
#include "Coordinates.h"

enum VertexBranchDirection : bool
{
    Forbid = false,
    Use = true
};

// Create the constraint handler for a branch and include it
SCIP_RETCODE SCIPincludeConshdlrVertexBranching(
    SCIP* scip    // SCIP
);

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
);

// Get branch direction
VertexBranchDirection SCIPgetVertexBranchingDirection(
    SCIP_CONS* cons    // Constraint enforcing vertex branching
);

// Get agent
Agent SCIPgetVertexBranchingAgent(
    SCIP_CONS* cons    // Constraint enforcing vertex branching
);

// Get node-time
NodeTime SCIPgetVertexBranchingNodeTime(
    SCIP_CONS* cons    // Constraint enforcing vertex branching
);

#endif
