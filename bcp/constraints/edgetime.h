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

#pragma once

#include "problem/includes.h"
#include "pricing/coordinates.h"
#include "problem/problem.h"

struct EdgeConflict
{
    SCIP_ROW* row;           // LP row
#ifdef USE_WAITEDGE_CONFLICTS
    Array<Edge, 3> edges;    // Edges in the conflict
#else
    Array<Edge, 2> edges;    // Edges in the conflict
#endif
    Time t;                  // Time of the conflict
};

// Create the constraint handler for edge conflicts and include it
SCIP_RETCODE SCIPincludeConshdlrEdgeConflicts(
    SCIP* scip    // SCIP
);

// Create a constraint for edge conflicts and include it
SCIP_RETCODE SCIPcreateConsEdgeConflicts(
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
);

SCIP_RETCODE edge_conflicts_add_var(
    SCIP* scip,                // SCIP
    SCIP_CONS* cons,           // Edge conflicts constraint
    SCIP_VAR* var,             // Variable
    const Time path_length,    // Path length
    const Edge* const path     // Path
);

const HashTable<EdgeTime, EdgeConflict>& edge_conflicts_get_constraints(
    SCIP_ProbData* probdata    // Problem data
);

