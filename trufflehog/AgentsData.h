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

#ifndef TRUFFLEHOG_AGENTSDATA_H
#define TRUFFLEHOG_AGENTSDATA_H

#include "Includes.h"
#include "Coordinates.h"
#include "Map.h"

namespace TruffleHog
{

class AgentsData
{
    struct AgentData
    {
        Node start;
        Node goal;
        Position start_x;
        Position start_y;
        Position goal_x;
        Position goal_y;
    };

    Vector<AgentData> data;

  public:
    // Constructors
    AgentsData() = default;
    ~AgentsData() = default;
    AgentsData(const AgentsData&) = default;
    AgentsData(AgentsData&&) = default;
    AgentsData& operator=(const AgentsData&) = default;
    AgentsData& operator=(AgentsData&&) = default;

    // Getters
    inline Agent size() const
    {
        return data.size();
    }
    inline bool empty() const
    {
        return !size();
    }
    inline const AgentData& operator[](const Agent a) const
    {
        debug_assert(a < size());
        return data[a];
    }

    // Add agent
    inline void add_agent(const Position start_x,
                          const Position start_y,
                          const Position goal_x,
                          const Position goal_y,
                          const Map& map)
    {
        data.push_back({map.get_id(start_x, start_y),
                        map.get_id(goal_x, goal_y),
                        start_x, start_y,
                        goal_x, goal_y});
    }
};

}

#endif
