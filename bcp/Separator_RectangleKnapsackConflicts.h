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

#ifndef MAPF_SEPARATOR_RECTANGLEKNAPSACKCONFLICTS_H
#define MAPF_SEPARATOR_RECTANGLEKNAPSACKCONFLICTS_H

#include "Includes.h"
#include "scip/scip.h"

struct RectangleKnapsackCut
{
    Int idx;                // Index of the cut
    uint16_t out1_begin;    // Index of the first out edge for agent 1
    uint16_t in2_begin;     // Index of the first in edge for agent 2
    uint16_t out2_begin;    // Index of the first out edge for agent 2
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

#endif
