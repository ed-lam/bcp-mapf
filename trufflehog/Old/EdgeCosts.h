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

#ifndef TRUFFLEHOG_EDGEDUALS_H
#define TRUFFLEHOG_EDGEDUALS_H

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
};
static_assert(std::is_trivially_copyable<EdgeCosts>::value);
static_assert(sizeof(EdgeCosts) == 5 * 8);

class EdgeDualValues
{
    Vector<Vector<Time>> time_;
    Vector<Vector<EdgeCosts>> duals_;
//    UniquePtr<Vector<Time>[]> time_; // TODO
//    UniquePtr<Vector<EdgeCosts>[]> duals_;
    Int map_size_;

  public:
    // Constructors
    EdgeDualValues() noexcept = delete;
    EdgeDualValues(const Int map_size) noexcept;
    EdgeDualValues(const EdgeDualValues& other) = default;
    EdgeDualValues(EdgeDualValues&& other) noexcept = default;
    EdgeDualValues& operator=(const EdgeDualValues& other) = default;
    EdgeDualValues& operator=(EdgeDualValues&& other) noexcept = default;
    ~EdgeDualValues() noexcept = default;

    // Get map size
    Int map_size() const;

    // Return the edge costs of a node-time
    template<IntCost default_cost>
    EdgeCosts get_edge_costs(const NodeTime nt) const;

    // Create or return the outgoing edge duals of a node-time
    EdgeCosts& get_edge_duals(const Node n, const Time t);
    EdgeCosts& get_edge_duals(const NodeTime nt);

    // Remove edges with default costs
    void clean_up(const Cost default_cost);

    // Reset for next run
    void reset();

    // Debug
    void print();
};

}

#endif
