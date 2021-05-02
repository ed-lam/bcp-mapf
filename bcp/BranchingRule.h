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

#ifndef MAPF_BRANCHINGRULE_H
#define MAPF_BRANCHINGRULE_H

#include "scip/scip.h"
#include "Coordinates.h"

// Create the branching rule and include it in SCIP
SCIP_RETCODE SCIPincludeBranchrule(
    SCIP* scip    // SCIP
);

// Branching on fractional LP solution
SCIP_RETCODE branch_lp(
    SCIP* scip,            // SCIP
    SCIP_RESULT* result    // Output status
);

// Branching on pseudosolution
SCIP_RETCODE branch_pseudosolution(
    SCIP* scip,            // SCIP
    SCIP_RESULT* result    // Output status
);

// Get the earliest time an agent reaches its goal
Vector<NodeTime> get_time_of_earliest_goal(
    SCIP* scip,                 // SCIP
    SCIP_PROBDATA* probdata,    // Problem data
    const Agent N               // Number of agents
);

// Create children nodes.
SCIP_RETCODE
branch_on_vertex(
    SCIP* scip,                   // SCIP
    const AgentNodeTime ant,      // Branch decision
    const bool prefer_branch_0    // Preferred branch direction
);
//SCIP_RETCODE
//branch_on_wait(
//    SCIP* scip,                   // SCIP
//    const AgentTime at,           // Branch decision
//    const bool prefer_branch_0    // Preferred branch direction
//);
SCIP_RETCODE
branch_on_length(
    SCIP* scip,                   // SCIP
    const AgentNodeTime ant,      // Branch decision
    const bool prefer_branch_0    // Preferred branch direction
);

#endif
