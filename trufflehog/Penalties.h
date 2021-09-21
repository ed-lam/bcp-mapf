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
#include "Map.h"

namespace TruffleHog
{

// Data structure for storing the costs of traversing an edge
struct EdgeCosts
{
    union
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
    bool used;

    inline EdgeCosts(const Cost x) : north(x), south(x), east(x), west(x), wait(x), used(false) {}
    inline EdgeCosts() : EdgeCosts(0) {}
};
static_assert(std::is_trivially_copyable<EdgeCosts>::value);
static_assert(sizeof(EdgeCosts) == 6 * 8);

// Penalties for crossing an edge
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

    // Iterators
    inline auto begin() { return edge_penalties_.begin(); }
    inline auto begin() const { return edge_penalties_.begin(); }
    inline auto end() { return edge_penalties_.end(); }
    inline auto end() const { return edge_penalties_.end(); }
    inline auto find(const NodeTime nt) { return edge_penalties_.find(nt); }
    inline auto find(const NodeTime nt) const { return edge_penalties_.find(nt); }

    // Return the edge costs of a node-time
    template<IntCost default_cost>
    inline EdgeCosts get_edge_costs(const NodeTime nt)
    {
        // Make default edge costs.
        EdgeCosts costs(default_cost);

        // Find the edge penalties.
        auto it = find(nt);
        if (it != end())
        {
            auto& penalties = it->second;

            penalties.used = true;

            debug_assert(penalties.north >= 0);
            debug_assert(penalties.south >= 0);
            debug_assert(penalties.east >= 0);
            debug_assert(penalties.west >= 0);
            debug_assert(penalties.wait >= 0);

            costs.north += penalties.north;
            costs.south += penalties.south;
            costs.east += penalties.east;
            costs.west += penalties.west;
            costs.wait += penalties.wait;
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
        return get_edge_penalties(NodeTime{n, t});
    }

    // Clear for next run
    inline void clear()
    {
        edge_penalties_.clear();
    }

    // Nothing to do before solving
    inline void before_solve() const
    {
        // Check.
#ifdef DEBUG
        for (const auto& [nt, penalties] : edge_penalties_)
        {
            debug_assert(penalties.north >= 0);
            debug_assert(penalties.south >= 0);
            debug_assert(penalties.east >= 0);
            debug_assert(penalties.west >= 0);
            debug_assert(penalties.wait >= 0);
            debug_assert(!penalties.used);
        }
#endif
    }

    // Debug
    void print(const Map& map)
    {
        HashTable<NodeTime, EdgeCosts> incoming_penalties;
        for (const auto [outgoing_nt, penalties] : edge_penalties_)
        {
            if (penalties.north != 0)
            {
                const auto incoming_n = map.get_north(outgoing_nt.n);
                const auto incoming_t = outgoing_nt.t + 1;
                const NodeTime incoming_nt{incoming_n, incoming_t};
                incoming_penalties[incoming_nt].south += penalties.north;
            }
            if (penalties.south != 0)
            {
                const auto incoming_n = map.get_south(outgoing_nt.n);
                const auto incoming_t = outgoing_nt.t + 1;
                const NodeTime incoming_nt{incoming_n, incoming_t};
                incoming_penalties[incoming_nt].north += penalties.south;
            }
            if (penalties.east != 0)
            {
                const auto incoming_n = map.get_east(outgoing_nt.n);
                const auto incoming_t = outgoing_nt.t + 1;
                const NodeTime incoming_nt{incoming_n, incoming_t};
                incoming_penalties[incoming_nt].west += penalties.east;
            }
            if (penalties.west != 0)
            {
                const auto incoming_n = map.get_west(outgoing_nt.n);
                const auto incoming_t = outgoing_nt.t + 1;
                const NodeTime incoming_nt{incoming_n, incoming_t};
                incoming_penalties[incoming_nt].east += penalties.west;
            }
            if (penalties.wait != 0)
            {
                const auto incoming_n = map.get_wait(outgoing_nt.n);
                const auto incoming_t = outgoing_nt.t + 1;
                const NodeTime incoming_nt{incoming_n, incoming_t};
                incoming_penalties[incoming_nt].wait += penalties.wait;
            }
        }

        println("Edge penalties:");
        println("{:>20s}{:>8s}{:>8s}{:>8s}{:>8s}{:>15s}{:>15s}{:>15s}{:>15s}{:>15s}",
                "NT", "N", "T", "X", "Y", "From North", "From South", "From East", "From West", "From Wait");
        for (const auto [nt, penalties] : incoming_penalties)
        {
            println("{:>20d}{:>8d}{:>8d}{:>8d}{:>8d}{:>15.2f}{:>15.2f}{:>15.2f}{:>15.2f}{:>15.2f}",
                    nt.nt, nt.n, nt.t,
                    map.get_x(nt.n), map.get_y(nt.n),
                    penalties.north, penalties.south, penalties.east, penalties.west, penalties.wait);
        }
        println("");
    }
    void print_used(const Map& map)
    {
        Vector<Pair<NodeTime, EdgeCosts>> all_penalties;
        for (const auto [nt, penalties] : edge_penalties_)
            if (penalties.used)
            {
                all_penalties.emplace_back(nt, penalties);
            }
        std::sort(all_penalties.begin(),
                  all_penalties.end(),
                  [](const Pair<NodeTime, EdgeCosts>& a, const Pair<NodeTime, EdgeCosts>& b)
                  {
                      return a.first.t < b.first.t;
                  });

        println("Used edge penalties:");
        println("{:>20s}{:>8s}{:>8s}{:>8s}{:>8s}{:>15s}{:>15s}{:>15s}{:>15s}{:>15s}",
                "NT", "N", "T", "X", "Y", "To North", "To South", "To East", "To West", "To Wait");
        for (const auto& [nt, penalties]: all_penalties)
        {
            println("{:>20d}{:>8d}{:>8d}{:>8d}{:>8d}{:>15.2f}{:>15.2f}{:>15.2f}{:>15.2f}{:>15.2f}",
                    nt.nt, nt.n, nt.t,
                    map.get_x(nt.n), map.get_y(nt.n),
                    penalties.north, penalties.south, penalties.east, penalties.west, penalties.wait);
        }
        println("");
    }
};

// Penalties for crossing the goal of another agent
#ifdef USE_GOAL_CONFLICTS
class GoalPenalties
{
  public:
    struct GoalPenalty
    {
        NodeTime nt;
        Cost cost;
    };
    static_assert(sizeof(GoalPenalty) == 2*8);

  private:
    Vector<GoalPenalty> goal_penalties_;

  public:
    // Constructors
    GoalPenalties() noexcept = default;
    GoalPenalties(const GoalPenalties& other) = default;
    GoalPenalties(GoalPenalties&& other) noexcept = default;
    GoalPenalties& operator=(const GoalPenalties& other) = default;
    GoalPenalties& operator=(GoalPenalties&& other) noexcept = default;
    ~GoalPenalties() noexcept = default;

    // Iterators
    inline auto begin() { return goal_penalties_.begin(); }
    inline auto begin() const { return goal_penalties_.begin(); }
    inline auto end() { return goal_penalties_.end(); }
    inline auto end() const { return goal_penalties_.end(); }

    // Getters
    inline Int size() const { return goal_penalties_.size(); }
    inline bool empty() const { return !size(); }
    inline const auto& data() const { return goal_penalties_; }
    const auto& operator[](const Int idx) const { return goal_penalties_[idx]; }

    // Clear for next run
    inline void clear()
    {
        goal_penalties_.clear();
    }

    // Add a goal penalty for crossing the goal of another agent
    void add(const NodeTime nt, const Cost cost)
    {
        debug_assert(cost >= 0);
        goal_penalties_.push_back({nt, cost});
    }

    // Nothing to do before solving
    inline void before_solve() const {}

    // Print
#ifdef DEBUG
    void print() const
    {
        if (!empty())
        {
            println("Goal crossings:");
            for (const auto& crossing : *this)
            {
                println("   nt: {}, n: {}, t: {}, cost: {}",
                        crossing.nt.nt, crossing.nt.n, crossing.nt.t, crossing.cost);
            }
        }
    }
#endif
};
#endif

// Penalties finishing at a particular time
class FinishTimePenalties
{
    Vector<Cost> finish_time_penalties_;
    Vector<Cost> finish_time_h_;

  public:
    // Constructors
    FinishTimePenalties() noexcept = default;
    FinishTimePenalties(const FinishTimePenalties& other) = default;
    FinishTimePenalties(FinishTimePenalties&& other) noexcept = default;
    FinishTimePenalties& operator=(const FinishTimePenalties& other) = default;
    FinishTimePenalties& operator=(FinishTimePenalties&& other) noexcept = default;
    ~FinishTimePenalties() noexcept = default;

    // Iterators
    inline auto begin() { return finish_time_penalties_.begin(); }
    inline auto begin() const { return finish_time_penalties_.begin(); }
    inline auto end() { return finish_time_penalties_.end(); }
    inline auto end() const { return finish_time_penalties_.end(); }

    // Getters
    inline Time size() const { return finish_time_penalties_.size(); }
    inline bool empty() const { return !size(); }
    inline const auto& data() const { return finish_time_penalties_; }
    auto operator[](const Time t) const { return finish_time_penalties_[t]; }

    // Clear for next run
    inline void clear()
    {
        finish_time_penalties_.clear();
        finish_time_h_.clear();
    }

    // Add a finish time penalty for finishing at or before time t_max
    void add(const Time t_max, const Cost cost)
    {
        debug_assert(cost >= 0);

        if (t_max + 1 >= size())
        {
            finish_time_penalties_.resize(t_max + 1);
        }
        for (Time t = 0; t <= t_max; ++t)
        {
            finish_time_penalties_[t] += cost;
        }
    }

    // Sum up the penalties as a lower bound (h value)
    void before_solve()
    {
        debug_assert(finish_time_h_.empty());
        finish_time_h_.resize(finish_time_penalties_.size());
        for (Time t = 0; t < static_cast<Time>(finish_time_h_.size()); ++t)
        {
            // Wait until after all finish time penalties have elapsed and then finish at no cost.
            finish_time_h_[t] = finish_time_penalties_.size() - t;

            // Wait until time i and then finish with a penalty.
            for (Time i = t; i < static_cast<Time>(finish_time_penalties_.size()); ++i)
            {
                finish_time_h_[t] = std::min(finish_time_h_[t], i - t + finish_time_penalties_[i]);
            }
        }
    }

    // Get the lower bound (h value) at the current time (assuming the agent is already at the goal location)
    inline Cost get_h(const Time t) const
    {
        return t < static_cast<Time>(finish_time_h_.size()) ? finish_time_h_[t] : 0.0;
    }
    inline Cost get_penalty(const Time t) const
    {
        return t < static_cast<Time>(finish_time_penalties_.size()) ? finish_time_penalties_[t] : 0.0;
    }
};

}

#endif
