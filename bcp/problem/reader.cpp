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

#include "problem/reader.h"
#include "problem/includes.h"
#include "problem/problem.h"
#include <regex>

#include "problem/instance.h"
#include "pricing/astar.h"

// Read instance from file
SCIP_RETCODE read_instance(
    SCIP* scip,                                    // SCIP
    const std::filesystem::path& scenario_path,    // File path to scenario
    const Agent nb_agents                          // Number of agents to read
)
{
    // Get instance name.
    auto instance_name = scenario_path.stem().string();
    if (nb_agents < std::numeric_limits<Agent>::max())
    {
        instance_name += fmt::format("-{}agents", nb_agents);
    }

    // Load instance.
    auto instance = std::make_shared<Instance>(scenario_path, nb_agents);

    // Create pricing solver.
    auto astar = std::make_shared<AStar>(instance->map);

    // Create the problem.
    SCIP_CALL(SCIPprobdataCreate(scip, instance_name.c_str(), instance, astar));

    // Done.
    return SCIP_OKAY;
}
