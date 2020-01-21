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

#ifndef TRUFFLEHOG_CROSSINGS_H
#define TRUFFLEHOG_CROSSINGS_H

#include "Coordinates.h"

namespace TruffleHog
{

#ifdef USE_GOAL_CONFLICTS
struct GoalCrossing
{
    Float dual;
    NodeTime nt;
};
static_assert(sizeof(GoalCrossing) == 8 + 8);
#endif

}

#endif
