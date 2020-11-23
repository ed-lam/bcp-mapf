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

#define PRINT_DEBUG

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <vector>
#include <string>
#include <queue>
#include <unordered_map>
#include "Debug.h"
#include "Includes.h"
#include "Coordinates.h"
#include "LabelPool.h"
#include "AgentsData.h"
#include "Map.h"
#include "Instance.h"
#include "EdgePenalties.h"
#include "Crossings.h"
#include "Heuristic.h"
#include "AStar.h"

int main()
{
    using namespace TruffleHog;

    // Load instance.
    const char* scen_path = "instances/movingai_2018/dao_maps/lak503dmap-100agents-47.scen";
    const auto instance = Instance(scen_path);
    const auto& map = instance.map;
    const auto& agents = instance.agents;
    const auto A = agents.size();

    // Create shortest path algorithm.
    AStar astar(map);
    astar.set_verbose(true);

    // Create edge dual values.
    auto& edge_penalties = astar.edge_penalties();
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(12986,30)); penalties.north=0.0; penalties.south=0.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=1.0; } // (50,66)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(13393,57)); penalties.north=0.0; penalties.south=0.0; penalties.east=0.0; penalties.west=1.0; penalties.wait=0.0; } // (65,68)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(16159,87)); penalties.north=0.0; penalties.south=0.0; penalties.east=0.0; penalties.west=999876.0; penalties.wait=0.0; } // (87,82)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(13182,30)); penalties.north=1.0; penalties.south=0.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=0.0; } // (50,67)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(15962,87)); penalties.north=0.0; penalties.south=999876.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=0.0; } // (86,81)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(26310,64)); penalties.north=0.0; penalties.south=0.0; penalties.east=0.0; penalties.west=18.0; penalties.wait=0.0; } // (46,134)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(12985,30)); penalties.north=0.0; penalties.south=0.0; penalties.east=1.0; penalties.west=0.0; penalties.wait=0.0; } // (49,66)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(10917,40)); penalties.north=0.0; penalties.south=0.0; penalties.east=0.0; penalties.west=1.0; penalties.wait=0.0; } // (137,55)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(13392,57)); penalties.north=0.0; penalties.south=0.0; penalties.east=1.0; penalties.west=0.0; penalties.wait=1.0; } // (64,68)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(26113,64)); penalties.north=0.0; penalties.south=18.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=0.0; } // (45,133)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(16158,87)); penalties.north=0.0; penalties.south=0.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=999876.0; } // (86,82)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(27082,96)); penalties.north=0.0; penalties.south=0.0; penalties.east=0.0; penalties.west=1.0; penalties.wait=0.0; } // (34,138)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(16743,78)); penalties.north=0.0; penalties.south=0.0; penalties.east=0.0; penalties.west=1.0; penalties.wait=0.0; } // (83,85)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(27079,44)); penalties.north=0.0; penalties.south=0.0; penalties.east=0.0; penalties.west=1.0; penalties.wait=0.0; } // (31,138)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(26321,75)); penalties.north=0.0; penalties.south=0.0; penalties.east=0.0; penalties.west=1.0; penalties.wait=0.0; } // (57,134)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(26309,64)); penalties.north=0.0; penalties.south=0.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=18.0; } // (45,134)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(10916,40)); penalties.north=0.0; penalties.south=0.0; penalties.east=1.0; penalties.west=0.0; penalties.wait=1.0; } // (136,55)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(27088,149)); penalties.north=0.0; penalties.south=0.0; penalties.east=0.0; penalties.west=999902.0; penalties.wait=0.0; } // (40,138)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(16354,87)); penalties.north=999876.0; penalties.south=0.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=0.0; } // (86,83)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(16546,78)); penalties.north=0.0; penalties.south=1.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=0.0; } // (82,84)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(26310,156)); penalties.north=0.0; penalties.south=0.0; penalties.east=0.0; penalties.west=45.0; penalties.wait=0.0; } // (46,134)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(19950,54)); penalties.north=0.0; penalties.south=1.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=0.0; } // (154,101)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(26882,44)); penalties.north=0.0; penalties.south=1.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=0.0; } // (30,137)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(26124,75)); penalties.north=0.0; penalties.south=1.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=0.0; } // (56,133)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(26320,36)); penalties.north=0.0; penalties.south=0.0; penalties.east=0.0; penalties.west=1.0; penalties.wait=0.0; } // (56,134)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(26891,149)); penalties.north=0.0; penalties.south=999902.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=0.0; } // (39,137)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(26310,111)); penalties.north=0.0; penalties.south=0.0; penalties.east=0.0; penalties.west=47.0; penalties.wait=0.0; } // (46,134)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(16157,87)); penalties.north=0.0; penalties.south=0.0; penalties.east=999876.0; penalties.west=0.0; penalties.wait=0.0; } // (85,82)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(26113,156)); penalties.north=0.0; penalties.south=45.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=0.0; } // (45,133)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(27081,96)); penalties.north=0.0; penalties.south=0.0; penalties.east=1.0; penalties.west=0.0; penalties.wait=1.0; } // (33,138)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(26505,64)); penalties.north=18.0; penalties.south=0.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=0.0; } // (45,135)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(26113,111)); penalties.north=0.0; penalties.south=47.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=0.0; } // (45,133)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(28324,127)); penalties.north=0.0; penalties.south=0.0; penalties.east=0.0; penalties.west=1.0; penalties.wait=0.0; } // (100,144)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(20146,54)); penalties.north=1.0; penalties.south=0.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=1.0; } // (154,102)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(16742,78)); penalties.north=0.0; penalties.south=0.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=1.0; } // (82,85)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(27078,44)); penalties.north=0.0; penalties.south=0.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=1.0; } // (30,138)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(26320,75)); penalties.north=0.0; penalties.south=0.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=1.0; } // (56,134)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(26308,64)); penalties.north=0.0; penalties.south=0.0; penalties.east=18.0; penalties.west=0.0; penalties.wait=0.0; } // (44,134)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(27087,149)); penalties.north=0.0; penalties.south=0.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=999902.0; } // (39,138)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(26309,156)); penalties.north=0.0; penalties.south=0.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=45.0; } // (45,134)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(28127,127)); penalties.north=0.0; penalties.south=1.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=0.0; } // (99,143)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(10917,81)); penalties.north=0.0; penalties.south=0.0; penalties.east=0.0; penalties.west=1.0; penalties.wait=0.0; } // (137,55)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(26319,36)); penalties.north=0.0; penalties.south=0.0; penalties.east=1.0; penalties.west=0.0; penalties.wait=1.0; } // (55,134)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(26309,111)); penalties.north=0.0; penalties.south=0.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=47.0; } // (45,134)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(16938,78)); penalties.north=1.0; penalties.south=0.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=0.0; } // (82,86)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(27274,44)); penalties.north=1.0; penalties.south=0.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=0.0; } // (30,139)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(26516,75)); penalties.north=1.0; penalties.south=0.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=0.0; } // (56,135)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(27283,149)); penalties.north=999902.0; penalties.south=0.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=0.0; } // (39,139)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(28325,171)); penalties.north=0.0; penalties.south=0.0; penalties.east=0.0; penalties.west=1.0; penalties.wait=0.0; } // (101,144)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(10720,81)); penalties.north=0.0; penalties.south=1.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=0.0; } // (136,54)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(26505,156)); penalties.north=45.0; penalties.south=0.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=0.0; } // (45,135)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(28323,127)); penalties.north=0.0; penalties.south=0.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=1.0; } // (99,144)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(16741,78)); penalties.north=0.0; penalties.south=0.0; penalties.east=1.0; penalties.west=0.0; penalties.wait=0.0; } // (81,85)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(27077,44)); penalties.north=0.0; penalties.south=0.0; penalties.east=1.0; penalties.west=0.0; penalties.wait=0.0; } // (29,138)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(26319,75)); penalties.north=0.0; penalties.south=0.0; penalties.east=1.0; penalties.west=0.0; penalties.wait=0.0; } // (55,134)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(27086,149)); penalties.north=0.0; penalties.south=0.0; penalties.east=999902.0; penalties.west=0.0; penalties.wait=0.0; } // (38,138)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(28128,171)); penalties.north=0.0; penalties.south=1.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=0.0; } // (100,143)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(26505,111)); penalties.north=47.0; penalties.south=0.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=0.0; } // (45,135)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(26488,108)); penalties.north=0.0; penalties.south=1.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=0.0; } // (28,135)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(26308,156)); penalties.north=0.0; penalties.south=0.0; penalties.east=45.0; penalties.west=0.0; penalties.wait=0.0; } // (44,134)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(10916,81)); penalties.north=0.0; penalties.south=0.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=1.0; } // (136,55)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(26308,111)); penalties.north=0.0; penalties.south=0.0; penalties.east=47.0; penalties.west=0.0; penalties.wait=0.0; } // (44,134)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(28519,127)); penalties.north=1.0; penalties.south=0.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=0.0; } // (99,145)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(28324,171)); penalties.north=0.0; penalties.south=0.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=1.0; } // (100,144)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(26684,108)); penalties.north=1.0; penalties.south=0.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=1.0; } // (28,136)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(28322,127)); penalties.north=0.0; penalties.south=0.0; penalties.east=1.0; penalties.west=0.0; penalties.wait=0.0; } // (98,144)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(12987,30)); penalties.north=0.0; penalties.south=0.0; penalties.east=0.0; penalties.west=1.0; penalties.wait=0.0; } // (51,66)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(11112,81)); penalties.north=1.0; penalties.south=0.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=0.0; } // (136,56)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(12790,30)); penalties.north=0.0; penalties.south=1.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=0.0; } // (50,65)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(28520,171)); penalties.north=1.0; penalties.south=0.0; penalties.east=0.0; penalties.west=0.0; penalties.wait=0.0; } // (100,145)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(10915,81)); penalties.north=0.0; penalties.south=0.0; penalties.east=1.0; penalties.west=0.0; penalties.wait=0.0; } // (135,55)
    { auto& penalties = edge_penalties.get_edge_penalties(NodeTime(28323,171)); penalties.north=0.0; penalties.south=0.0; penalties.east=1.0; penalties.west=0.0; penalties.wait=0.0; } // (99,144)

    // Get agent.
    const auto a = 1;
//    1
//    4
//    6
//    9
//    11
//    14
//    24
//    28
//    30
//    34
//    37
//    39
//    40
//    45
//    49
//    50
//    52
//    60
//    63
//    68
//    75
//    79
//    93
//    97
//    for (Agent a = 0; a < A; ++a)
    {
        println("Agent {}", a);

        const auto start = NodeTime(agents[a].start, 0);
        const auto goal = agents[a].goal;

        // Solve.
        astar.compute_h(goal);
        const auto path = astar.solve<false>(start, goal);

        if (path.second != astar.heuristic_.compute_h(goal)[start.n])
        {
            println("{} {}", a, path.second - astar.heuristic_.compute_h(goal)[start.n]);
            printf("");
        }
        fflush(stdout);
    }

    // Exit.
    return 0;
}
