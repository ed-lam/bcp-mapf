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

#pragma once

#include "problem/includes.h"
#include "types/map_types.h"
#include "types/memory_pool.h"
#include "problem/map.h"
#include "types/priority_queue.h"
#include "types/hash_map.h"

class Heuristic
{
    // Label for heuristic
    struct Label
    {
#ifdef DEBUG
        size_t label_id;
        const Label* parent;
#endif
        Node n;
        Time g;
    };
#ifdef DEBUG
    static_assert(sizeof(Label) == 2*8 + 2*4);
#else
    static_assert(sizeof(Label) == 2*4);
#endif

    // Comparison of labels
    struct LabelComparison
    {
        static inline bool lt(const Label* const lhs, const Label* const rhs)
        {
            return lhs->g <  rhs->g;
        }
        static inline bool le(const Label* const lhs, const Label* const rhs)
        {
            return lhs->g <= rhs->g;
        }
        static inline bool eq(const Label* const lhs, const Label* const rhs)
        {
            return lhs->g == rhs->g;
        }
    };

    // Priority queue holding labels
    using PriorityQueueSizeType = Int32;
    class HeuristicPriorityQueue : public PriorityQueue<Label*, LabelComparison, PriorityQueueSizeType>
    {
      public:
        // Modify the handle in the label pointing to its position in the priority queue
        void update_index(Label*, const PriorityQueueSizeType) {}

        // Check the validity of an index
        Bool check_index(Label* const&, const PriorityQueueSizeType) const { return true; }
    };

    // Instance
    const Map& map_;

    // Lower bounds
    HashMap<Node, Vector<Time>> h_;
    Time max_path_length_;

    // Solver data structures
    MemoryPool label_pool_;
    HeuristicPriorityQueue open_;
    Vector<bool> visited_;
#ifdef DEBUG
    size_t nb_labels_;
#endif

  public:
    // Constructors
    Heuristic() = delete;
    Heuristic(const Map& map);
    Heuristic(const Heuristic&) = delete;
    Heuristic(Heuristic&&) = delete;
    Heuristic& operator=(const Heuristic&) = delete;
    Heuristic& operator=(Heuristic&&) = delete;
    ~Heuristic() = default;

    // Getters
    inline auto max_path_length() const { return max_path_length_; }

    // Get the lower bound from every node to a goal node
    const Vector<Time>& get_h(const Node goal);

  private:
    // Check if a node has already been visited
    bool dominated(const Node n);

    // Generate labels
    void generate_start(const Node start);
    void generate(const Label* const current, const Node n);
    void generate_neighbours(const Label* const current);

    // Compute lower bound from every node to a goal node
    void search(const Node goal, Vector<Time>& h);
};

