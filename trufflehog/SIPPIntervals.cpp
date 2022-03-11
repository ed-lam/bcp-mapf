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

// #define PRINT_DEBUG

#include "SIPPIntervals.h"

namespace TruffleHog
{

SIPPIntervals::SIPPIntervals(const Map& map) :
    map_(map),
    map_size_(map.size()),

    edge_penalties_(),

    intervals_(),
    intervals_range_(map_size_ * 5 * 2),
    clear_from_(),
    clear_to_()
{
}

Pair<SIPPInterval*, SIPPInterval*> SIPPIntervals::get_intervals(const Node n, const Direction d)
{
    return {intervals_.data() + intervals_range_[2 * (n * 5 + d)],
            intervals_.data() + intervals_range_[2 * (n * 5 + d) + 1]};
}

// void SIPPIntervals::clear()
// {
//     edge_penalties_.clear();
// }

// void SIPPIntervals::add_edge_penalty(const NodeTime nt, const Direction d, const Cost cost)
// {
//     debug_assert(map_[nt.n]);
//     edge_penalties_.emplace_back(TimeDirectionNode{nt.t, d, nt.n}, cost);
// }

void SIPPIntervals::create_intervals(const Vector<NodeTime>& waypoints,
                                     const Node goal,
                                     const EdgePenalties& edge_penalties,
                                     const FinishTimePenalties& finish_time_penalties)
{
    // Reorder edge penalties.
    edge_penalties_.clear();
    for (const auto& [nt, edge_penalty] : edge_penalties)
        if (map_[nt.n])
            for (Int d = 0; d < 5; ++d)
                if (const auto penalty = edge_penalty.d[d]; penalty != 0)
                {
                    edge_penalties_.emplace_back(TimeDirectionNode{nt.t, static_cast<Direction>(d), nt.n}, penalty);
                }

    // Add extra intervals to correctly expand to the waypoints (which includes the goal).
    for (const auto nt : waypoints)
        if (const Time t = nt.t; t > 0)
        {
            edge_penalties_.emplace_back(TimeDirectionNode{t - 1, Direction::WAIT, nt.n}, 0.0);
        }

    // Add extra intervals to correctly expand to the dummy end node.
    for (Time t = 1; t < static_cast<Time>(finish_time_penalties.size()); ++t)
        if (finish_time_penalties[t] != finish_time_penalties[t - 1])
        {
            edge_penalties_.emplace_back(TimeDirectionNode{t - 1, Direction::WAIT, goal}, 0.0);
        }
    if (const Time t = finish_time_penalties.size(); t > 0)
    {
        edge_penalties_.emplace_back(TimeDirectionNode{t - 1, Direction::WAIT, goal}, 0.0);
    }

    // Create intervals from edge penalties.
    intervals_.clear();
    memset(intervals_range_.data() + clear_from_, 0, sizeof(IntervalIndex) * (clear_to_ - clear_from_));
#ifdef DEBUG
    for (const auto x : intervals_range_)
    {
        debug_assert(x == 0);
    }
#endif
    clear_from_ = 0;
    clear_to_ = 0;
    if (!edge_penalties_.empty())
    {
        // Sort edge penalties.
        std::sort(edge_penalties_.begin(),
                  edge_penalties_.end(),
                  [](const Pair<TimeDirectionNode, Cost>& a, const Pair<TimeDirectionNode, Cost>& b)
                  { return a.first < b.first; });
#ifdef DEBUG
        for (Int idx = 0; idx < static_cast<Int>(edge_penalties_.size()) - 1; ++idx)
        {
            const auto& a = edge_penalties_[idx];
            const auto& b = edge_penalties_[idx + 1];
            debug_assert((a.first.n <  b.first.n) ||
                         (a.first.n == b.first.n && a.first.d <  b.first.d) ||
                         (a.first.n == b.first.n && a.first.d == b.first.d && a.first.t <  b.first.t) ||
                         (a.first.n == b.first.n && a.first.d == b.first.d && a.first.t == b.first.t));
        }
#endif

        // Start creating intervals.
        SIPPInterval* prev_interval = nullptr;
        SIPPInterval* interval = nullptr;
        Node prev_interval_n = -2;
        Node interval_n = -1;
        Direction prev_interval_d = Direction::INVALID;
        Direction interval_d = Direction::INVALID;
        for (auto it = edge_penalties_.begin(), end = edge_penalties_.end(); it != end;)
        {
            // Get the edge and its penalty.
            const auto n = it->first.n;
            const auto d = it->first.d;
            const auto t = it->first.t;
            debugln("n {}, xy ({},{}), t {}, d {}, penalty {:.6f}",
                    n, map_.get_x(n), map_.get_y(n), t, static_cast<Direction>(d), it->second);

            // Check that we need a new interval.
            debug_assert(interval_n != n || interval_d != d || interval->start != t);

            // Merge the latest interval into the previous interval if they are consecutive. Otherwise advance one
            // interval.
            if (prev_interval_n == interval_n &&
                prev_interval_d == interval_d &&
                interval_d != Direction::WAIT &&
                prev_interval->end == interval->start &&
                prev_interval->penalty == interval->penalty)
            {
                prev_interval->end = interval->end;
                debug_assert(interval == &intervals_.back());
                intervals_.pop_back();
                --interval;
            }
            else
            {
                prev_interval = interval;
                prev_interval_n = interval_n;
                prev_interval_d = interval_d;
            }

            // Create a zero-penalty interval if there is a gap between the previous interval and the next interval.
            if (prev_interval_n == n &&
                prev_interval_d == d &&
                d != Direction::WAIT)
            {
                const auto prev_interval_end = prev_interval->end;
                if (prev_interval_end != t)
                {
                    prev_interval = &intervals_.emplace_back();
                    prev_interval->start = prev_interval_end;
                    prev_interval->end = t;
                    prev_interval->penalty = 0;
                }
            }

            // Create an interval.
            if (prev_interval_n != n || prev_interval_d != d)
            {
                // Store the end index of the previous interval and the start index of the next interval.
                release_assert(intervals_.size() < std::numeric_limits<IntervalIndex>::max() - 10,
                               "SIPP intervals overflow");
                if (prev_interval_d != Direction::INVALID)
                {
                    debug_assert(intervals_range_[2 * (prev_interval_n * 5 + prev_interval_d) + 1] == 0);
                    intervals_range_[2 * (prev_interval_n * 5 + prev_interval_d) + 1] = intervals_.size();
                }
                debug_assert(intervals_range_[2 * (n * 5 + d)] == 0);
                intervals_range_[2 * (n * 5 + d)] = intervals_.size();
            }
            interval_n = n;
            interval_d = static_cast<Direction>(d);
            interval = &intervals_.emplace_back();
            prev_interval = interval - 1;
            interval->start = t;
            interval->end = t + 1;
            interval->penalty = 0;
            while (it != end &&
                   it->first.n == interval_n &&
                   it->first.d == interval_d &&
                   it->first.t == interval->start)
            {
                interval->penalty += it->second;
                ++it;
            }
        }

        // Merge the latest interval into the previous interval if they are consecutive.
        if (prev_interval_n == interval_n &&
            prev_interval_d == interval_d &&
            interval_d != Direction::WAIT &&
            prev_interval->end == interval->start &&
            prev_interval->penalty == interval->penalty)
        {
            prev_interval->end = interval->end;
            debug_assert(interval == &intervals_.back());
            intervals_.pop_back();
            // --interval;
        }

        // Store the end index of the previous interval
        debug_assert(interval_d != Direction::INVALID);
        debug_assert(intervals_range_[2 * (interval_n * 5 + interval_d) + 1] == 0);
        intervals_range_[2 * (interval_n * 5 + interval_d) + 1] = intervals_.size();

        // Store where to clear memory for the next iteration.
        clear_from_ = 2 * (edge_penalties_.front().first.n * 5 + edge_penalties_.front().first.d);
        clear_to_ = 2 * (interval_n * 5 + interval_d) + 1 + 1;
        debug_assert(clear_from_ < clear_to_);
#ifdef DEBUG
        for (size_t idx = 0; idx < clear_from_; ++idx)
        {
            debug_assert(intervals_range_[idx] == 0);
        }
        for (size_t idx = clear_to_; idx < intervals_range_.size(); ++idx)
        {
            debug_assert(intervals_range_[idx] == 0);
        }
#endif

        // Print.
#ifdef PRINT_DEBUG
        {
            for (Node n = 0; n < map_size_; ++n)
                for (Int d = 0; d < 5; ++d)
                    for (auto [it, end] = get_intervals(n, static_cast<Direction>(d)); it != end; ++it)
                    {
                        println("{} {} ({},{}) to ({},{}):",
                                n,
                                static_cast<Direction>(d),
                                map_.get_x(n),
                                map_.get_y(n),
                                map_.get_destination_xy(Edge{n, Direction(d)}).first,
                                map_.get_destination_xy(Edge{n, Direction(d)}).second);
                        for (auto [it, end] = get_intervals(n, static_cast<Direction>(d)); it != end; ++it)
                        {
                            const auto& interval = *it;
                            println("    start {}, end {}, penalty {:.6f}", interval.start, interval.end, interval.penalty);
                        }
                    }
            println("");
        }
#endif

        // Check for no intervals at obstacles.
#ifdef DEBUG
        for (Node n = 0; n < map_size_; ++n)
            if (!map_[n])
                for (Int d = 0; d < 5; ++d)
                {
                    auto [it, end] = get_intervals(n, static_cast<Direction>(d));
                    debug_assert(it == end);
                }
#endif

        // Check order.
#ifdef DEBUG
        for (Node n = 0; n < map_size_; ++n)
        {
            for (Int d = 0; d < Direction::WAIT; ++d)
                for (auto [it, end] = get_intervals(n, static_cast<Direction>(d)); it != end && (it + 1) != end; ++it)
                {
                    debug_assert(it->end == (it + 1)->start);
                }
            {
                const auto d = Direction::WAIT;
                for (auto [it, end] = get_intervals(n, static_cast<Direction>(d)); it != end && (it + 1) != end; ++it)
                {
                    debug_assert(it->end <= (it + 1)->start);
                }
            }
        }
#endif
    }
}

}
