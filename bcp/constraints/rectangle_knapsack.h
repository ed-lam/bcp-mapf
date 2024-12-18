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

#ifdef USE_RECTANGLE_KNAPSACK_CONFLICTS

#pragma once

#include "problem/includes.h"
#include "types/vector.h"

struct RectangleKnapsackCut
{
    Int idx;           // Index of the cut
    Int out1_begin;    // Index of the first out edge for agent 1
    Int in2_begin;     // Index of the first in edge for agent 2
    Int out2_begin;    // Index of the first out edge for agent 2
};

// Create separator for rectangle knapsack conflicts and include it
SCIP_RETCODE SCIPincludeSepaRectangleKnapsackConflicts(
    SCIP* scip,         // SCIP
    SCIP_SEPA** sepa    // Output pointer to separator
);

// Get additional data about rectangle knapsack cuts
const Vector<RectangleKnapsackCut>& rectangle_knapsack_get_cuts(
    SCIP_ProbData* probdata    // Problem data
);

#endif

