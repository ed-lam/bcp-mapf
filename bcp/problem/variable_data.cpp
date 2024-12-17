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

#include "VariableData.h"
#include "ProblemData.h"

// Struct has undefined size - cannot use sizeof(SCIP_VarData)
struct SCIP_VarData
{
    Agent a;             // Agent of the path
    Time path_length;    // Length of the path
    Edge path[0];        // Edges in the path
};

// Create variable data
SCIP_RETCODE SCIPvardataCreate(
    SCIP* scip,                // SCIP
    const Agent a,             // Agent of the path
    const Time path_length,    // Length of the path
    const Edge* const path,    // Edges in the path
    SCIP_VARDATA** vardata     // Output variable data
)
{
    // Allocate memory.
    debug_assert(vardata);
    const auto size = sizeof(**vardata) + sizeof(*path) * path_length;
    SCIP_CALL(SCIPallocBlockMemorySize(scip, vardata, size));
    debug_assert(*vardata);

    // Copy data about the agent.
    (*vardata)->a = a;

    // Copy data about the path.
    (*vardata)->path_length = path_length;
    memcpy((*vardata)->path, path, sizeof(*path) * path_length);

    // Check.
#ifdef DEBUG
    const auto& map = SCIPprobdataGetMap(SCIPgetProbData(scip));
    for (Time t = 0; t < path_length - 1; ++t)
    {
        const auto i = path[t].n;
        const auto j = path[t + 1].n;
        if (path[t].d == Direction::NORTH)
        {
            debug_assert(j == map.get_north(i));
        }
        else if (path[t].d == Direction::SOUTH)
        {
            debug_assert(j == map.get_south(i));
        }
        else if (path[t].d == Direction::EAST)
        {
            debug_assert(j == map.get_east(i));
        }
        else if (path[t].d == Direction::WEST)
        {
            debug_assert(j == map.get_west(i));
        }
        else if (path[t].d == Direction::WAIT)
        {
            debug_assert(j == map.get_wait(i));
        }
        else
        {
            err("Invalid direction {}", static_cast<int>(path[t].d));
        }
    }
#endif

    // Done.
    return SCIP_OKAY;
}

// Free variable data of transformed variable
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_VARDELTRANS(vardataDelTrans)
{
    // Free memory.
    debug_assert(vardata);
    if (*vardata)
    {
        const auto size = sizeof(**vardata) + sizeof(*(*vardata)->path) * (*vardata)->path_length;
        SCIPfreeBlockMemorySize(scip, vardata, size);
    }

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Create variable
SCIP_RETCODE SCIPcreateVar(
    SCIP* scip,                // SCIP
    SCIP_VAR** var,            // Pointer to variable
    const char* const name,    // Name of variable
    const SCIP_Real obj,       // Objective coefficient
    SCIP_VARDATA* vardata      // Data associated with the variable
)
{
    // Check.
    debug_assert(scip);
    debug_assert(var);

    // Create the variable.
    SCIP_CALL(SCIPcreateVar(scip,
                            var,
                            name,
                            0.0,
                            1.0,
                            obj,
                            SCIP_VARTYPE_BINARY,
                            FALSE,
                            TRUE,
                            nullptr,
                            nullptr,
                            vardataDelTrans,
                            nullptr,
                            vardata));
    debug_assert(*var);

    // Done.
    return SCIP_OKAY;
}

// Get the agent of the path
Agent SCIPvardataGetAgent(
    SCIP_VARDATA* vardata    // Variable data
)
{
    debug_assert(vardata);
    debug_assert(vardata->a >= 0);
    return vardata->a;
}

// Get the length of the path
Time SCIPvardataGetPathLength(
    SCIP_VARDATA* vardata    // Variable data
)
{
    debug_assert(vardata);
    debug_assert(vardata->path_length >= 1);
    return vardata->path_length;
}

// Get the edges in the path
const Edge* SCIPvardataGetPath(
    SCIP_VARDATA* vardata    // Variable data
)
{
    debug_assert(vardata);
    debug_assert(vardata->path);
    return vardata->path;
}
