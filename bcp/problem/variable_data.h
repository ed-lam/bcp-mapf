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

#ifndef MAPF_VARIABLEDATA_H
#define MAPF_VARIABLEDATA_H

#include "pricing/coordinates.h"

// Create variable data
SCIP_RETCODE SCIPvardataCreate(
    SCIP* scip,                // SCIP
    const Agent a,             // Agent of the path
    const Time path_length,    // Length of the path
    const Edge* const path,    // Edges in the path
    SCIP_VARDATA** vardata     // Output variable data
);

// Create variable
SCIP_RETCODE SCIPcreateVar(
    SCIP* scip,                // SCIP
    SCIP_VAR** var,            // Pointer to variable
    const char* const name,    // Name of variable
    const SCIP_Real obj,       // Objective coefficient
    SCIP_VARDATA* vardata      // Data associated with the variable
);

// Get the agent of the path
Agent SCIPvardataGetAgent(
    SCIP_VARDATA* vardata    // Variable data
);

// Get the length of the path
Time SCIPvardataGetPathLength(
    SCIP_VARDATA* vardata    // Variable data
);

// Get the edges in the path
const Edge* SCIPvardataGetPath(
    SCIP_VARDATA* vardata    // Variable data
);

#endif
