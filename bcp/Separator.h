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

#ifndef MAPF_SEPARATOR_H
#define MAPF_SEPARATOR_H

#include "Includes.h"
#include "Coordinates.h"
#include "scip/scip.h"

class TwoAgentRobustCut
{
#ifdef DEBUG
    String name_;
#endif
    SCIP_ROW* row_;
    Agent a1_;
    Agent a2_ : 31;
    bool is_same_time_ : 1;
    Int a2_begin_;
    Int a2_end_;
    union
    {
        EdgeTime* ets_;
        struct
        {
            Edge* es_;
            Time t_;
        };
    };
  public:

    // Constructors
    TwoAgentRobustCut(
        SCIP* scip,
        const Agent a1,
        const Agent a2,
        const Time t,
        const Int nb_a1_edges,
        const Int nb_a2_edges
#ifdef DEBUG
        , String&& name
#endif
    ) :
#ifdef DEBUG
        name_(std::move(name)),
#endif
        row_(nullptr),
        a1_(a1),
        a2_(a2),
        is_same_time_(true),
        a2_begin_(nb_a1_edges),
        a2_end_(nb_a1_edges + nb_a2_edges),
        t_(t)
    {
        scip_assert(SCIPallocBlockMemoryArray(scip, &es_, a2_end_));
    }
    TwoAgentRobustCut(
        SCIP* scip,
        const Agent a1,
        const Agent a2,
        const Int nb_a1_edgetimes,
        const Int nb_a2_edgetimes
#ifdef DEBUG
        , String&& name
#endif
    ) :
#ifdef DEBUG
        name_(std::move(name)),
#endif
        row_(nullptr),
        a1_(a1),
        a2_(a2),
        is_same_time_(false),
        a2_begin_(nb_a1_edgetimes),
        a2_end_(nb_a1_edgetimes + nb_a2_edgetimes)
    {
        scip_assert(SCIPallocBlockMemoryArray(scip, &ets_, a2_end_));
    }

    // Getters
    inline auto a1() const { return a1_; }
    inline auto a2() const { return a2_; }
    inline auto row() const { return row_; }
    inline auto is_same_time() const { return is_same_time_; }
    inline auto t() const { debug_assert(is_same_time()); return t_; }
#ifdef DEBUG
    inline const auto& name() const { return name_; }
#endif

    // Setters
    inline void set_row(SCIP_ROW* row) { row_ = row; }

    // Get edge-times
    Pair<const EdgeTime*, const EdgeTime*> edge_times_a1() const
    {
        debug_assert(!is_same_time());
        return {&ets_[0], &ets_[a2_begin_]};
    }
    Pair<const EdgeTime*, const EdgeTime*> edge_times_a2() const
    {
        debug_assert(!is_same_time());
        return {&ets_[a2_begin_], &ets_[a2_end_]};
    }
    Pair<EdgeTime*, EdgeTime*> edge_times_a1()
    {
        debug_assert(!is_same_time());
        return {&ets_[0], &ets_[a2_begin_]};
    }
    Pair<EdgeTime*, EdgeTime*> edge_times_a2()
    {
        debug_assert(!is_same_time());
        return {&ets_[a2_begin_], &ets_[a2_end_]};
    }
    Pair<const EdgeTime*, const EdgeTime*> edge_times(const Agent a) const
    {
        debug_assert(!is_same_time());
        debug_assert(a == a1_ || a == a2_);
        return a == a1_ ? edge_times_a1() : edge_times_a2();
    }
    Pair<EdgeTime*, EdgeTime*> edge_times(const Agent a)
    {
        debug_assert(!is_same_time());
        debug_assert(a == a1_ || a == a2_);
        return a == a1_ ? edge_times_a1() : edge_times_a2();
    }
    const EdgeTime& edge_times_a1(const Int idx) const
    {
        debug_assert(!is_same_time());
        return ets_[idx];
    }
    const EdgeTime& edge_times_a2(const Int idx) const
    {
        debug_assert(!is_same_time());
        return ets_[a2_begin_ + idx];
    }
    EdgeTime& edge_times_a1(const Int idx)
    {
        debug_assert(!is_same_time());
        return ets_[idx];
    }
    EdgeTime& edge_times_a2(const Int idx)
    {
        debug_assert(!is_same_time());
        return ets_[a2_begin_ + idx];
    }

    // Get edges
    Pair<const Edge*, const Edge*> edges_a1() const
    {
        debug_assert(is_same_time());
        return {&es_[0], &es_[a2_begin_]};
    }
    Pair<const Edge*, const Edge*> edges_a2() const
    {
        debug_assert(is_same_time());
        return {&es_[a2_begin_], &es_[a2_end_]};
    }
    Pair<Edge*, Edge*> edges_a1()
    {
        debug_assert(is_same_time());
        return {&es_[0], &es_[a2_begin_]};
    }
    Pair<Edge*, Edge*> edges_a2()
    {
        debug_assert(is_same_time());
        return {&es_[a2_begin_], &es_[a2_end_]};
    }
    Pair<const Edge*, const Edge*> edges(const Agent a) const
    {
        debug_assert(is_same_time());
        debug_assert(a == a1_ || a == a2_);
        return a == a1_ ? edges_a1() : edges_a2();
    }
    Pair<Edge*, Edge*> edges(const Agent a)
    {
        debug_assert(is_same_time());
        debug_assert(a == a1_ || a == a2_);
        return a == a1_ ? edges_a1() : edges_a2();
    }
    const Edge& edges_a1(const Int idx) const
    {
        debug_assert(is_same_time());
        return es_[idx];
    }
    const Edge& edges_a2(const Int idx) const
    {
        debug_assert(is_same_time());
        return es_[a2_begin_ + idx];
    }
    Edge& edges_a1(const Int idx)
    {
        debug_assert(is_same_time());
        return es_[idx];
    }
    Edge& edges_a2(const Int idx)
    {
        debug_assert(is_same_time());
        return es_[a2_begin_ + idx];
    }
};

#endif
