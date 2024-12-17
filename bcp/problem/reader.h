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

#ifndef MAPF_READER_H
#define MAPF_READER_H

#include "Includes.h"
#include <filesystem>

// Read instance from file
SCIP_RETCODE read_instance(
    SCIP* scip,                                                  // SCIP
    const std::filesystem::path& scenario_path,                  // File path to scenario
    const Agent nb_agents = std::numeric_limits<Agent>::max()    // Number of agents to read
);

#endif
