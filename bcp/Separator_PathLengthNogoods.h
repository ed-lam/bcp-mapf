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

#ifndef MAPF_SEPARATOR_PATHLENGTHNOGOODS_H
#define MAPF_SEPARATOR_PATHLENGTHNOGOODS_H

#include "problem/includes.h"
#include "pricing/coordinates.h"
#include "problem/problem.h"

// Create separator for path length nogoods and include it
SCIP_RETCODE SCIPincludeSepaPathLengthNogoods(
    SCIP* scip    // SCIP
);

SCIP_RETCODE path_length_nogoods_add_var(
    SCIP* scip,                                       // SCIP
    Vector<PathLengthNogood>& path_length_nogoods,    // Data for the nogoods
    SCIP_VAR* var,                                    // Variable
    const Agent a,                                    // Agent
    const Time path_length                            // Path length
);

#endif

#endif
