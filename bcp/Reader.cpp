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

#include "Reader.h"
#include "Includes.h"
#include "ProblemData.h"
#include <regex>

#include "trufflehog/Instance.h"
#include "trufflehog/AStar.h"

// Read instance from file
SCIP_RETCODE read_instance(
    SCIP* scip,                   // SCIP
    const char* scenario_path,    // File path to scenario
    const Agent nb_agents         // Number of agents to read
)
{
    // Get instance name.
    String instance_name("mapf");
    {
        const String str(scenario_path);
        std::smatch m;
        if (std::regex_match(str, m, std::regex(".*\\/(.+)\\.map.scen")))
        {
            instance_name = m.str(1);
        }
        else if (std::regex_match(str, m, std::regex(".*\\/(.+)\\.scen")))
        {
            instance_name = m.str(1);
        }
    }
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
