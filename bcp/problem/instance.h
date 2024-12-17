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

#ifndef TRUFFLEHOG_INSTANCE_H
#define TRUFFLEHOG_INSTANCE_H

#include "AgentsData.h"
#include "Map.h"
#include <filesystem>

namespace TruffleHog
{

struct Instance
{
    std::filesystem::path scenario_path;
    std::filesystem::path map_path;

    Map map;
    AgentsData agents;

  public:
    // Constructors
    Instance() = default;
    Instance(const std::filesystem::path& scenario_path, const Agent agent_limit = std::numeric_limits<Agent>::max());
    Instance(const Instance&) = default;
    Instance(Instance&&) = default;
    Instance& operator=(const Instance&) = default;
    Instance& operator=(Instance&&) = default;
    ~Instance() = default;
};

}

#endif
