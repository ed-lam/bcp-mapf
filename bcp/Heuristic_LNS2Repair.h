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

#ifdef USE_LNS2_REPAIR_PRIMAL_HEURISTIC

#ifndef MAPF_HEURISTIC_LNS2REPAIR_H
#define MAPF_HEURISTIC_LNS2REPAIR_H

#include "scip/def.h"
#include "scip/type_retcode.h"
#include "scip/type_scip.h"

// Create the LNS2 repair primal heuristic and include it in SCIP
SCIP_RETCODE SCIPincludeHeurLNS2Repair(
    SCIP* scip
);

#endif

#endif
