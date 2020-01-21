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

#ifndef TRUFFLEHOG_EDGEPENALTIES_H
#define TRUFFLEHOG_EDGEPENALTIES_H

#include "Includes.h"
#include "Coordinates.h"

namespace TruffleHog
{

union EdgeCosts
{
    Cost d[5];
    struct
    {
        Cost north;
        Cost south;
        Cost east;
        Cost west;
        Cost wait;
    };

    inline EdgeCosts() : north(0), south(0), east(0), west(0), wait(0) {}
    inline EdgeCosts(const Cost default_cost) :
        north(default_cost),
        south(default_cost),
        east(default_cost),
        west(default_cost),
        wait(default_cost)
    {
    }
};
static_assert(std::is_trivially_copyable<EdgeCosts>::value);
static_assert(sizeof(EdgeCosts) == 5 * 8);

class EdgePenalties
{
    HashTable<NodeTime, EdgeCosts> edge_penalties_;

  public:
    // Constructors
    EdgePenalties() noexcept = default;
    EdgePenalties(const EdgePenalties& other) = default;
    EdgePenalties(EdgePenalties&& other) noexcept = default;
    EdgePenalties& operator=(const EdgePenalties& other) = default;
    EdgePenalties& operator=(EdgePenalties&& other) noexcept = default;
    ~EdgePenalties() noexcept = default;

    // Return the edge costs of a node-time
    template<IntCost default_cost>
    inline EdgeCosts get_edge_costs(const NodeTime nt) const
    {
        // Make default edge costs.
        EdgeCosts costs(default_cost);

        // Find the edge penalties.
        auto it = edge_penalties_.find(nt);
        if (it != edge_penalties_.end())
        {
            const auto& values = it->second;

            costs.north += values.north;
            costs.south += values.south;
            costs.east += values.east;
            costs.west += values.west;
            costs.wait += values.wait;
        }

        // Return.
        return costs;
    }

    // Create or return the outgoing edge penalties of a node-time
    inline EdgeCosts& get_edge_penalties(const NodeTime nt)
    {
        return edge_penalties_[nt];
    }
    inline EdgeCosts& get_edge_penalties(const Node n, const Time t)
    {
        return get_edge_penalties(NodeTime(n, t));
    }

    // Reset for next run
//    inline void reset()
//    {
//        edge_penalties_.clear();
//    }

    // Debug
    void print()
    {
        println("Edge penalties:");
        for (const auto& [nt, values] : edge_penalties_)
        {
            println("   n: {}, t: {}, nt: {}, north: {}, south: {}, east: {}, west {}, wait {}",
                    nt.n, nt.t, nt.nt,
                    values.north, values.south, values.east, values.west, values.wait);
        }
        println("");
    }
};

}

#endif
