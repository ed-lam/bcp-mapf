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

#if defined(USE_RECTANGLE_KNAPSACK_CONFLICTS) || defined(USE_RECTANGLE_CLIQUE_CONFLICTS)

#ifndef MAPF_SEPARATOR_RECTANGLECONFLICTS_H
#define MAPF_SEPARATOR_RECTANGLECONFLICTS_H

#include "Includes.h"
#include "Coordinates.h"
#include "ProblemData.h"

struct RectangleConflict
{
    Vector<EdgeTime> edges;    // Edges of the rectangle
    Agent a1;                  // Agent 1 in the conflict
    Agent a2;                  // Agent 2 in the conflict
    uint16_t out1_begin_;      // Index of the first out edge for agent 1
    uint16_t in2_begin_;       // Index of the first in edge for agent 2
    uint16_t out2_begin_;      // Index of the first out edge for agent 2

    inline auto empty() const { return edges.empty(); }

    inline auto in1_begin() const { return edges.begin(); }
    inline auto in1_end() const { return edges.begin() + out1_begin_; }
    inline auto out1_begin() const { return edges.begin() + out1_begin_; }
    inline auto out1_end() const { return edges.begin() + in2_begin_; }
    inline auto in2_begin() const { return edges.begin() + in2_begin_; }
    inline auto in2_end() const { return edges.begin() + out2_begin_; }
    inline auto out2_begin() const { return edges.begin() + out2_begin_; }
    inline auto out2_end() const { return edges.end(); }

    using ETIteratorPair = Pair<Vector<EdgeTime>::const_iterator,
                                Vector<EdgeTime>::const_iterator>;
    using ETIteratorTriple = Tuple<Vector<EdgeTime>::const_iterator,
                                   Vector<EdgeTime>::const_iterator,
                                   Vector<EdgeTime>::const_iterator>;

    inline auto agent1_edges() const { return ETIteratorPair{in1_begin(), out1_end()}; }
    inline auto agent2_edges() const { return ETIteratorPair{in2_begin(), out2_end()}; }
    inline auto agent_edges(const Agent a) const
    {
        debug_assert(a == a1 || a == a2);
        return a == a1 ? agent1_edges() : agent2_edges();
    }
    inline auto agent_in_out_edges(const Agent a) const
    {
        debug_assert(a == a1 || a == a2);
        return a == a1 ?
               ETIteratorTriple{in1_begin(), in1_end(), out1_end()} :
               ETIteratorTriple{in2_begin(), in2_end(), out2_end()};
    }
};

void compute_rectangle(
    const Map& map,                // Map
    const Time start_t,            // Time before entry
    const Time end_t,              // Time after exit
    const Position start_x1,       // Start coordinate of agent 1
    const Position start_y1,       // Start coordinate of agent 1
    const Position start_x2,       // Start coordinate of agent 2
    const Position start_y2,       // Start coordinate of agent 2
    const Position end_x1,         // End coordinate of agent 1
    const Position end_y1,         // End coordinate of agent 1
    const Position end_x2,         // End coordinate of agent 2
    const Position end_y2,         // End coordinate of agent 2
    RectangleConflict& conflict    // Output rectangle conflict
);

#endif

#endif
