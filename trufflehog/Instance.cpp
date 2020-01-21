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

#include <fstream>
#include "Instance.h"

namespace TruffleHog
{

struct AgentMapData
{
    String map_path;
    Position map_width;
    Position map_height;
};

void read_map(const char* const map_path, Map& map)
{
    // Open map file.
    std::ifstream map_file;
    map_file.open(map_path, std::ios::in);
    release_assert(map_file.good(), "Invalid map file {}", map_path);

    // Read map.
    {
        char buf[1024];
        map_file.getline(buf, 1024);
        release_assert(strstr(buf, "type octile"), "Invalid map file format");
    }

    // Read map size.
    Position width = 0;
    Position height = 0;
    String param;
    {
        Int value;
        for (Int i = 0; i < 2; ++i)
        {
            map_file >> param >> value;
            if (param == "width")
            {
                width = value;
            }
            else if (param == "height")
            {
                height = value;
            }
            else
            {
                err("Invalid input in map file {}", param);
            }
        }
        release_assert(height > 0, "Invalid map height {}", height);
        release_assert(width > 0, "Invalid map width {}", width);
        height += 2; // Add padding.
        width += 2;
    }

    // Create map.
    map.resize(width, height);

    // Read grid.
    map_file >> param;
    release_assert(param == "map", "Invalid map file format");
    {
        auto c = static_cast<char>(map_file.get());
        release_assert(map_file.good(), "Invalid map format");
        if (c == '\r')
        {
            c = static_cast<char>(map_file.get());
            release_assert(map_file.good(), "Invalid map format");
        }
        release_assert(c == '\n', "Invalid map format");
    }
    Node n = width + 1; // Start reading into the second row, second column of the grid
    while (true)
    {
        auto c = static_cast<char>(map_file.get());
        if (!map_file.good())
        {
            // eof
            break;
        }

        switch (c)
        {
            case '\n':
                if (auto c2 = map_file.peek(); map_file.good() && c2 != '\r' && c2 != '\n')
                {
                    n += 2;
                }
                break;
            case ' ':
            case '\t':
            case '\r':
                continue;
            case '.':
                release_assert(n < map.size(),
                               "More tiles in the map file than its size");
                map.set_passable(n);
                [[fallthrough]];
            default:
                n++;
                break;
        }
    }
    n += width + 1; // Should be +2 but already counted a +1 from the previous \n
    release_assert(n == map.size(), "Unexpected number of tiles");

    // Close file.
    map_file.close();
}

Instance::Instance(const char* scenario_path, const Agent nb_agents)
{
    // Read agents.
    Vector<AgentMapData> agents_map_data;
    {
        // Open scenario file.
        std::ifstream scen_file;
        scen_file.open(scenario_path, std::ios::in);
        release_assert(scen_file.good(), "Cannot find scenario file {}", scenario_path);

        // Check file format.
        {
            char buf[1024];
            scen_file.getline(buf, 1024);
            release_assert(strstr(buf, "version 1"), "Invalid scenario file format");
        }

        // Read agents data.
        {
            Position start_x;
            Position start_y;
            Position goal_x;
            Position goal_y;
            Float tmp;
            AgentMapData agent_map_data;
            while (agents.size() < nb_agents &&
                   (scen_file >> tmp >>
                    agent_map_data.map_path >>
                    agent_map_data.map_width >> agent_map_data.map_height >>
                    start_x >> start_y >>
                    goal_x >> goal_y >> tmp))
            {
                // Add padding.
                agent_map_data.map_width += 2;
                agent_map_data.map_height += 2;
                start_x++;
                start_y++;
                goal_x++;
                goal_y++;

                // Read map.
                if (map.empty())
                {
                    // Prepend the directory of the scenario file.
                    String map_path_str;
                    const String scenario_path_str(scenario_path);
                    const auto last_slash_idx = scenario_path_str.rfind('/');
                    if (std::string::npos != last_slash_idx)
                    {
                        map_path_str = scenario_path_str.substr(0, last_slash_idx);
                    }
                    map_path_str += "/" + agent_map_data.map_path;

                    // Read map.
                    read_map(map_path_str.c_str(), map);
                }

                // Store.
                agents.add_agent(start_x, start_y, goal_x, goal_y, map);
                agents_map_data.push_back(agent_map_data);
            }
        }
        release_assert(
            nb_agents == std::numeric_limits<Int>::max() || agents.size() == nb_agents,
            "Scenario file contained {} agents. Not enough to read {} agents",
            agents.size(), nb_agents);
        if (agents.empty())
        {
            err("No agents in scenario file {}", scenario_path);
        }

        // Close file.
        scen_file.close();
    }

    // Check.
    for (Agent a = 0; a < agents.size(); ++a)
    {
        const auto [start_id, goal_id, start_x, start_y, goal_x, goal_y] = agents[a];
        const auto [map_path, map_width2, map_height2] = agents_map_data[a];

        release_assert(map_path == agents_map_data[0].map_path,
                       "Agent {} uses a different map");
        release_assert(map_width2 == map.width(),
                       "Map width of agent {} does not match actual map size", a);
        release_assert(map_height2 == map.height(),
                       "Map width of agent {} does not match actual map size", a);

        const auto start_tile = start_y * map.width() + start_x;
        release_assert(start_tile < map.size() && map[start_tile],
                       "Agent {} starts at an obstacle", a);

        const auto goal_tile = goal_y * map.width() + goal_x;
        release_assert(goal_tile < map.size() && map[goal_tile],
                       "Agent {} ends at an obstacle", a);
    }
}

}
