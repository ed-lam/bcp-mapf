#pragma once

#include "output/formatting.h"
#include "types/hash_map.h"
#include "types/map_types.h"
#include "types/vector.h"

class NodeCrossingPenalties
{
    // Node crossing
    struct NodeCrossing
    {
        NodeTime nt;
        Cost penalty;
    };
    static_assert(sizeof(NodeCrossing) == 2*8);

    Vector<NodeCrossing> data_;

  public:
    // Constructors and destructor
    NodeCrossingPenalties() noexcept = default;
    NodeCrossingPenalties(const NodeCrossingPenalties& other) = default;
    NodeCrossingPenalties(NodeCrossingPenalties&& other) noexcept = default;
    NodeCrossingPenalties& operator=(const NodeCrossingPenalties& other) = default;
    NodeCrossingPenalties& operator=(NodeCrossingPenalties&& other) noexcept = default;
    ~NodeCrossingPenalties() noexcept = default;

    // Getters
    auto empty() const { return data_.empty(); }
    Size size() const { return data_.size(); }
    const auto& operator[](const Size index) const { return data_[index]; }
    // inline const auto& data() const { return goal_penalties_; }

    // Iterators
    // inline auto begin() { return goal_penalties_.begin(); }
    // inline auto begin() const { return goal_penalties_.begin(); }
    // inline auto end() { return goal_penalties_.end(); }
    // inline auto end() const { return goal_penalties_.end(); }

    // Clear
    inline void clear()
    {
        data_.clear();
    }

    // Add a penalty for crossing a node (e.g., the target of another agent) at or later than nt.t
    void add(const NodeTime nt, const Cost cost)
    {
        debug_assert(nt.t > 0);
        debug_assert(cost >= 0);
        data_.push_back({nt, cost});
    }

    // Nothing to do before solving
    inline void finalise() const {}

    // Print
    void print(const Map& map) const
    {
        println("Node crossing penalties:");
        println("{:>12s}{:>8s}{:>18s}", "N", "T", "Penalty");
        for (Size index = 0; index < data_.size(); ++index)
        {
            const auto& [nt, penalty] = data_[index];
            println("{:>12s}{:>8d}{:>18.2f}", format_node(nt.n, map), nt.t, penalty);
        }
        println("");
    }
};
