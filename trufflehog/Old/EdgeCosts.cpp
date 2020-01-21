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

//#define PRINT_DEBUG

#include "EdgeCosts.h"

#define isEQ(x,y) (std::abs((x)-(y)) <= (1e-06))

namespace TruffleHog
{

EdgeDualValues::EdgeDualValues(const Int map_size) noexcept
    : time_(map_size),
      duals_(map_size),
    //: time_(std::make_unique<Vector<Time>[]>(map_size)),
    //duals_(std::make_unique<Vector<EdgeCosts>[]>(map_size)),
      map_size_(map_size)
{
}

Int EdgeDualValues::map_size() const
{
    return map_size_;
}

template<IntCost default_cost>
EdgeCosts EdgeDualValues::get_edge_costs(const NodeTime nt) const
{
    // Make default edge costs.
    EdgeCosts costs{{default_cost,
                     default_cost,
                     default_cost,
                     default_cost,
                     default_cost}};

    // Find the time.
    debug_assert(0 <= nt.n && nt.n < map_size_);
    const auto& time_list = time_[nt.n];
    const auto time_it = std::lower_bound(time_list.begin(), time_list.end(), nt.t);

    // Find the edge duals at the time.
    if (time_it != time_list.end() && *time_it == nt.t)
    {
        const auto idx = time_it - time_list.begin();

        const auto& duals_list = duals_[nt.n];
        const auto& duals = duals_list[idx];

        costs.north -= duals.north;
        costs.south -= duals.south;
        costs.east -= duals.east;
        costs.west -= duals.west;
        costs.wait -= duals.wait;
    }

    // Return.
    return costs;
}
template EdgeCosts EdgeDualValues::get_edge_costs<0>(const NodeTime nt) const;
template EdgeCosts EdgeDualValues::get_edge_costs<1>(const NodeTime nt) const;

EdgeCosts& EdgeDualValues::get_edge_duals(const Node n, const Time t)
{
    return get_edge_duals(NodeTime(n, t));
}

EdgeCosts& EdgeDualValues::get_edge_duals(const NodeTime nt)
{
    // Find a place to insert the time.
    debug_assert(0 <= nt.n && nt.n < map_size_);
    auto& time_list = time_[nt.n];
    auto time_it = std::lower_bound(time_list.begin(), time_list.end(), nt.t);
    const auto idx = time_it - time_list.begin();

    // Insert or return pointer to existing edge dual values.
    auto& duals_list = duals_[nt.n];
    if (time_it != time_list.end() && *time_it == nt.t)
    {
        auto& duals = duals_list[idx];
        return duals;
    }
    else
    {
        time_list.insert(time_it, nt.t);

        auto& duals = *duals_list.emplace(duals_list.begin() + idx);
        duals.north = 0;
        duals.south = 0;
        duals.east = 0;
        duals.west = 0;
        duals.wait = 0;
        return duals;
    }
}

void EdgeDualValues::clean_up(const Cost default_cost)
{
#ifdef PRINT_DEBUG
    Int count = 0;
#endif
    for (Node n = 0; n < static_cast<Node>(time_.size()); ++n)
        for (Int idx = 0; idx < static_cast<Int>(time_[n].size()); ++idx)
        {
            const auto& costs = duals_[n][idx];

            if (isEQ(costs.north, default_cost) &&
                isEQ(costs.south, default_cost) &&
                isEQ(costs.east, default_cost) &&
                isEQ(costs.west, default_cost) &&
                isEQ(costs.wait, default_cost))
            {
                time_[n].erase(time_[n].begin() + idx);
                duals_[n].erase(duals_[n].begin() + idx);

#ifdef PRINT_DEBUG
                count++;
#endif
            }
        }
#ifdef PRINT_DEBUG
    debugln("Deleted {} vertices with default outgoing costs", count);
#endif
}

void EdgeDualValues::reset()
{
    for (Int idx = 0; idx < map_size_; ++idx)
    {
        time_[idx].clear();
        duals_[idx].clear();
    }
}

void EdgeDualValues::print()
{
    println("Edge costs:");
    for (Node n = 0; n < static_cast<Node>(time_.size()); ++n)
        for (Int idx = 0; idx < static_cast<Int>(time_[n].size()); ++idx)
        {
            const auto t = time_[n][idx];
            const auto costs = duals_[n][idx];
            println("   n: {}, t: {}, north: {}, south: {}, east: {}, west {}, wait {}",
                    n, t, costs.north, costs.south, costs.east, costs.west, costs.wait);
        }
    println("");
}

// Find the position of an existing edge costs or the position to insert a new one
//Vector<Time>::const_iterator EdgeDualValues::find_pos_lb(const NodeTime nt)
//{
//#ifndef NDEBUG
//    assert(nt.n < map_size());
//#endif
//
//    auto& time_list = time_[nt.n];
//    auto time_it = std::lower_bound(time_list.begin(), time_list.end(), nt.t);
//    return time_it;
//}
//
//// Remove all edge costs at a node
//void EdgeDualValues::clear_constraint_at(const Node n)
//{
//    auto& costs = duals_[n];
//    costs.clear();
//}

// Get all edge costs at a node
//const Vector<EdgeCosts::EdgeCosts>&
//EdgeDualValues::get_all_edge_costs(const Node n) const
//{
//#ifndef NDEBUG
//    assert(n < map_size());
//#endif
//    return duals_[n];
//}

// remove all constraints with default values
//    void clean_up()
//    {
//        for (auto& xy_cons : duals_)
//        {
//            bool deleted = false;
//            for (auto it = xy_cons.begin(); it != xy_cons.end();)
//            {
//                auto& con = *it;
//                if (con.e_[warthog::cbs::move::NORTH] == 1 &&
//                    con.e_[warthog::cbs::move::SOUTH] == 1 &&
//                    con.e_[warthog::cbs::move::EAST] == 1 &&
//                    con.e_[warthog::cbs::move::WEST] == 1 &&
//                    con.e_[warthog::cbs::move::WAIT] == 1)
//                {
//                    std::swap(*it, xy_cons.back());
//                    xy_cons.pop_back();
//                    deleted = true;
//                }
//                else
//                {
//                    ++it;
//                }
//            }
//            if (deleted)
//            {
//                std::sort(xy_cons.begin(),
//                          xy_cons.end(),
//                          [](const EdgeCosts& a, const EdgeCosts& b)
//                          { return a.timestep_ < b.timestep_; });
//            }
//        }
//    }

// Add or replace outgoing edge costs of a node-time
//    void add_or_replace(const NodeTime nt, const EdgeCosts& costs)
//    {
//        auto& time_list = time_[nt.n];
//        auto& costs_list = duals_[nt.n];
//
//        const auto time_it = find_pos(nt);
//        const auto idx = time_it - time_list.begin();
//        if (time_it != time_list.end() && *time_it == nt.t)
//        {
//            costs_list[idx] = costs;
//        }
//        else
//        {
//            time_list.insert(time_it, nt.t);
//            costs_list.insert(costs_list.begin() + idx, costs);
//        }
//    }

}
