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

#ifndef MAPF_CLOCK_H
#define MAPF_CLOCK_H

#include "scip/def.h"
#include "scip/clock.h"
#include "scip/struct_scip.h"
#include "scip/struct_set.h"
#include "scip/struct_stat.h"

inline SCIP_Real get_clock(SCIP* scip)
{
    return SCIPclockGetTime(scip->stat->solvingtime);
}

inline SCIP_Real get_time_remaining(SCIP* scip)
{
    return scip->set->limit_time - get_clock(scip);
}

#endif
