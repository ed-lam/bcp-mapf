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
    const char* scen_path =
        "2018_instances/10obs-20x20map/10obs-20x20map-50agents-18.scen";
    const auto instance = Instance(scen_path);
    const auto& map = instance.map;
    const auto& agents = instance.agents;

    // Create shortest path algorithm.
    AStar astar(map);
    astar.set_verbose(true);

    // Create edge dual values.
//    auto& edge_duals = astar.edge_duals();
//    {
//        auto& duals = edge_duals.get_edge_duals(NodeTime(5, 2));
//        duals.north -= 1;
//        duals.south -= 2;
//        duals.east -= 3;
//        duals.west -= 4;
//        duals.wait -= 5;
//        printf("");
//    }
//    {
//        auto& duals = edge_duals.get_edge_duals(NodeTime(5, 3));
//        duals.north -= 1;
//        duals.south -= 2;
//        duals.east -= 3;
//        duals.west -= 4;
//        duals.wait -= 5;
//        printf("");
//    }
//    {
//        auto& duals = edge_duals.get_edge_duals(NodeTime(5, 7));
//        duals.north -= 1;
//        duals.south -= 2;
//        duals.east -= 3;
//        duals.west -= 4;
//        duals.wait -= 5;
//        printf("");
//    }
//    {
//        auto& duals = edge_duals.get_edge_duals(NodeTime(5, 2));
//        duals.north -= 1;
//        duals.south -= 2;
//        duals.east -= 3;
//        duals.west -= 4;
//        duals.wait -= 5;
//        printf("");
//    }

    // Create goal crossings.
//    auto& goal_crossings = astar.goal_crossings();
//    {
//        auto& goal = goal_crossings.emplace_back();
//        goal.dual = -20;
//        goal.nt = NodeTime(292, 2);
//    }

    // Solve.
    for (Int a = 0; a < agents.size(); ++a)
    {
        const auto start = agents[a].start;
        const auto goal = agents[a].goal;

        auto& reserv = astar.reservation_table();
        reserv.reserve(NodeTime{316, 1});
        reserv.reserve(NodeTime{341, 1});
        const auto x = reserv.is_reserved(NodeTime{316, 1});

        const auto path = astar.solve<false>(NodeTime(start, 0), goal);
        fflush(stdout);

        exit(1);
    }

    // Solve.
//    const auto start = NodeTime(agents[0].start, 5);
//    const auto goal = agents[0].goal;
//    const auto path = astar.solve(start, goal, 40, 75);
//    fmt::print("{}\n", fmt::join(path.first, ","));
//    fflush(stdout);

    // Done.
    return 0;
}
