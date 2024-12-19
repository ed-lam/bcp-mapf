#pragma once

#include "output/formatting.h"
#include "problem/map.h"
#include "types/basic_types.h"
#include "types/float_compare.h"
#include "types/hash_map.h"

// Data structure for storing the costs of traversing an edge
struct DirectionalCosts
{
    union
    {
        Cost d[5];
        struct
        {
            Cost north;
            Cost south;
            Cost west;
            Cost east;
            Cost wait;
        };
    };
    Bool used;

    inline DirectionalCosts(const Cost x) : north(x), south(x), west(x), east(x), wait(x), used(false) {}
    inline DirectionalCosts() : DirectionalCosts(0) {}
    inline auto operator[](const Size i) const { return d[i]; }
};
static_assert(std::is_trivially_copyable<DirectionalCosts>::value);
static_assert(sizeof(DirectionalCosts) == 6 * 8);
static_assert(Direction::NORTH == 0);
static_assert(Direction::SOUTH == 1);
static_assert(Direction::WEST == 2);
static_assert(Direction::EAST == 3);
static_assert(Direction::WAIT == 4);

class EdgeTimePenalties
{
    HashMap<NodeTime, DirectionalCosts> data_;

  public:
    // Constructors and destructor
    EdgeTimePenalties() noexcept = default;
    EdgeTimePenalties(const EdgeTimePenalties& other) = default;
    EdgeTimePenalties(EdgeTimePenalties&& other) noexcept = default;
    EdgeTimePenalties& operator=(const EdgeTimePenalties& other) = default;
    EdgeTimePenalties& operator=(EdgeTimePenalties&& other) noexcept = default;
    ~EdgeTimePenalties() noexcept = default;

    // Getters
    // inline Bool empty() const { return data_.empty(); }

    // Iterators
    inline auto begin() { return data_.begin(); }
    inline auto begin() const { return data_.begin(); }
    inline auto end() { return data_.end(); }
    inline auto end() const { return data_.end(); }
    inline auto erase(HashMap<NodeTime, DirectionalCosts>::iterator& it) { return data_.erase(it); }
    inline auto find(const NodeTime nt) { return data_.find(nt); }
    inline auto find(const NodeTime nt) const { return data_.find(nt); }

    // Get penalties at a nodetime
    // Create or return the outgoing edge penalties of a node-time
    inline DirectionalCosts& insert(const NodeTime nt)
    {
        return data_[nt];
    }
    inline DirectionalCosts& insert(const Node n, const Time t)
    {
        return insert(NodeTime{n, t});
    }

    // Return the edge costs of a node-time
    template<Time default_cost>
    inline DirectionalCosts get_costs(const NodeTime nt)
    {
        // Make default edge costs.
        DirectionalCosts costs(default_cost);

        // Find the edge penalties.
        auto it = find(nt);
        if (it != end())
        {
            auto& penalties = it->second;

            penalties.used = true;

            debug_assert(penalties.north >= 0);
            debug_assert(penalties.south >= 0);
            debug_assert(penalties.west >= 0);
            debug_assert(penalties.east >= 0);
            debug_assert(penalties.wait >= 0);

            costs.north += penalties.north;
            costs.south += penalties.south;
            costs.west += penalties.west;
            costs.east += penalties.east;
            costs.wait += penalties.wait;
        }

        // Return.
        return costs;
    }
    // template<Time default_cost>
    // inline DirectionalCosts get_costs(const NodeTime nt)
    // {
    //     debug_assert(nt.t >= 0);
    //     const auto it = data_.find(nt);
    //     if (it != data_.end())
    //     {
    //         auto& penalties = it->second;
    //         debug_assert(penalties.north >= 0);
    //         debug_assert(penalties.south >= 0);
    //         debug_assert(penalties.west >= 0);
    //         debug_assert(penalties.east >= 0);
    //         debug_assert(penalties.wait >= 0);
    //         penalties.used = true;
    //         return penalties;
    //     }
    //     else
    //     {
    //         return DirectionalCosts{};
    //     }
    // }

    // Clear
    inline void clear() { data_.clear(); }
    inline void move_to_old() { /* Nothing to do. */ }

    // Add a penalty into a nodetime
    void add_nodetime_penalty(const NodeTime nt, const Cost cost)
    {
        debug_assert(nt.t >= 0);
        debug_assert(cost >= 0);
        auto& penalties = data_[nt];
        penalties.north += cost;
        penalties.south += cost;
        penalties.west += cost;
        penalties.east += cost;
        penalties.wait += cost;
    }

    // Add a penalty to an edgetime
    // The penalty is stored in the direction of movement into the nodetime
    void add_edgetime_penalty(const NodeTime nt, const Direction d, const Cost cost)
    {
        debug_assert(cost >= 0);
        auto& penalties = data_[nt];
        penalties.d[d] += cost;
    }

    // Prepare data structures for efficient lookup before solving
    inline void finalise() { /* Nothing to do. */ }

    // Print
    void print_outgoing(const Map& map) const
    {
        println("Outgoing edgetime penalties:");
        println("{:>20s}{:>8s}{:>15s}{:>16s}{:>14s}{:>16s}{:>14s}{:>16s}{:>14s}{:>16s}{:>14s}{:>16s}{:>14s}",
                "NT", "N", "XYT", "To North", "Penalty", "To South", "Penalty", "To West", "Penalty", "To East", "Penalty", "To Wait", "Penalty");
        for (const auto& [nt, penalties] : data_)
        {
            if (penalties.north != 0 || penalties.south != 0 || penalties.west != 0 || penalties.east != 0 ||
                penalties.wait != 0)
            {
                const NodeTime north_nt{map.get_destination(nt.n, Direction::NORTH), nt.t + 1};
                const NodeTime south_nt{map.get_destination(nt.n, Direction::SOUTH), nt.t + 1};
                const NodeTime west_nt{map.get_destination(nt.n, Direction::WEST), nt.t + 1};
                const NodeTime east_nt{map.get_destination(nt.n, Direction::EAST), nt.t + 1};
                const NodeTime wait_nt{map.get_destination(nt.n, Direction::WAIT), nt.t + 1};
                println("{:>20d}{:>8d}{:>15s}{:>16s}{:>14.4f}{:>16s}{:>14.4f}{:>16s}{:>14.4f}{:>16s}{:>14.4f}{:>16s}{:>14.4f}",
                        nt.nt, nt.n,
                        format_nodetime(nt, map),
                        format_nodetime(north_nt, map),
                        penalties.north,
                        format_nodetime(south_nt, map),
                        penalties.south,
                        format_nodetime(west_nt, map),
                        penalties.west,
                        format_nodetime(east_nt, map),
                        penalties.east,
                        format_nodetime(wait_nt, map),
                        penalties.wait);
            }
        }
        println("");
    }
    // void print(const Map& map) const
    // {
    //     println("Incoming edgetime penalties:");
    //     println("{:>20s}{:>8s}{:>15s}{:>16s}{:>14s}{:>16s}{:>14s}{:>16s}{:>14s}{:>16s}{:>14s}{:>16s}{:>14s}",
    //             "NT", "N", "XYT", "To North", "Penalty", "To South", "Penalty", "To West", "Penalty", "To East", "Penalty", "To Wait", "Penalty");
    //     for (const auto& [nt, penalties] : data_)
    //     {
    //         if (penalties.north != 0 || penalties.south != 0 || penalties.west != 0 || penalties.east != 0 ||
    //             penalties.wait != 0)
    //         {
    //             const NodeTime north_nt{map.get_destination(nt.n, Direction::NORTH), nt.t-1};
    //             const NodeTime south_nt{map.get_destination(nt.n, Direction::SOUTH), nt.t-1};
    //             const NodeTime west_nt{map.get_destination(nt.n, Direction::WEST), nt.t-1};
    //             const NodeTime east_nt{map.get_destination(nt.n, Direction::EAST), nt.t-1};
    //             const NodeTime wait_nt{map.get_destination(nt.n, Direction::WAIT), nt.t-1};
    //             println("{:>20d}{:>8d}{:>15s}{:>16s}{:>14.4f}{:>16s}{:>14.4f}{:>16s}{:>14.4f}{:>16s}{:>14.4f}{:>16s}{:>14.4f}",
    //                     nt.nt, nt.n,
    //                     format_nodetime(nt, map),
    //                     format_nodetime(south_nt, map),
    //                     penalties.north,
    //                     format_nodetime(north_nt, map),
    //                     penalties.south,
    //                     format_nodetime(east_nt, map),
    //                     penalties.west,
    //                     format_nodetime(west_nt, map),
    //                     penalties.east,
    //                     format_nodetime(wait_nt, map),
    //                     penalties.wait);
    //         }
    //     }
    //     println("");
    // }
};
