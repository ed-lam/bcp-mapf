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

#ifndef TRUFFLEHOG_HEURISTIC_H
#define TRUFFLEHOG_HEURISTIC_H

#include "Includes.h"
#include "Coordinates.h"
#include "LabelPool.h"
#include "Map.h"
#include "PriorityQueue.h"

namespace TruffleHog
{

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
        IntCost g;
    };
#ifdef DEBUG
    static_assert(sizeof(Label) == 2*8 + 2*4);
#else
    static_assert(sizeof(Label) == 2*4);
#endif

    // Comparison of labels
    struct LabelCompare
    {
        inline bool operator()(const Label* const a, const Label* const b) const
        {
            return a->g < b->g;
        }
    };

    // Priority queue holding labels
    class HeuristicPriorityQueue : public PriorityQueue<Label, LabelCompare>
    {
      public:
        // Inherit constructors.
        using PriorityQueue::PriorityQueue;

      protected:
        // Modify the handle in the label pointing to its position in the priority queue
        inline void update_pqueue_index(Label*, const Int) {}

        // Checks.
#ifdef DEBUG
        Cost get_f(const Label* label) const { return label->g; }
        void check_pqueue_index() const {}
#endif
    };

    // Instance
    const Map& map_;

    // Lower bounds
    HashTable<Node, Vector<IntCost>> h_;
    Time max_path_length_;

    // Solver data structures
    LabelPool label_pool_;
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
    const Vector<IntCost>& get_h(const Node goal);

  private:
    // Check if a node has already been visited
    bool dominated(const Node n);

    // Generate labels
    void generate_start(const Node start);
    void generate(const Label* const current, const Node n);
    void generate_neighbours(const Label* const current);

    // Compute lower bound from every node to a goal node
    void search(const Node goal, Vector<IntCost>& h);
};

}

#endif
