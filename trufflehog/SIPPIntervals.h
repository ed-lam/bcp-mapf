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

#ifndef TRUFFLEHOG_SIPPINTERVALS_H
#define TRUFFLEHOG_SIPPINTERVALS_H

#include "problem/includes.h"
#include "pricing/coordinates.h"
#include "problem/map.h"
#include "pricing/penalties.h"

namespace TruffleHog
{

union TimeDirectionNode
{
    struct
    {
        Time t : 29;
        Direction d : 3;
        Node n;
    };
    uint64_t id;

    TimeDirectionNode() noexcept = default;
    inline explicit TimeDirectionNode(const Time t, const Direction d, const Node n) noexcept : t{t}, d{d}, n{n} {}

    inline bool operator<(const TimeDirectionNode other) const { return id < other.id; }
};

struct SIPPInterval
{
    Time start;
    Time end;
    Cost penalty;

    inline bool operator==(const SIPPInterval& other) const { return start == other.start && end == other.end && penalty == other.penalty; }
};

class SIPPIntervals
{
#ifdef USE_PRIORITIZED_PLANNING_PRIMAL_HEURISTIC
    using IntervalIndex = uint32_t;
#else
    using IntervalIndex = uint16_t;
#endif

    const Map& map_;
    Int map_size_;

    Vector<Pair<TimeDirectionNode, Cost>> edge_penalties_;

    Vector<SIPPInterval> intervals_;
    Vector<IntervalIndex> intervals_range_;
    size_t clear_from_;
    size_t clear_to_;

  public:
    // Constructors
    SIPPIntervals() = delete;
    SIPPIntervals(const Map& map);
    SIPPIntervals(const SIPPIntervals&) = delete;
    SIPPIntervals(SIPPIntervals&&) = delete;
    SIPPIntervals& operator=(const SIPPIntervals&) = delete;
    SIPPIntervals& operator=(SIPPIntervals&&) = delete;
    ~SIPPIntervals() = default;

    // Store edge penalties
    // void clear();
    // void add_edge_penalty(const NodeTime nt, const Direction d, const Cost cost);

    // Solve
    void create_intervals(const Vector<NodeTime>& waypoints,
                          const Node goal,
                          const EdgePenalties& edge_penalties,
                          const FinishTimePenalties& finish_time_penalties);
    Pair<SIPPInterval*, SIPPInterval*> get_intervals(const Node n, const Direction d);
};

}

#endif