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
        const Label* parent;
        IntCost g;
        Node n;
#ifdef DEBUG
        Int label_id;
#endif
    };
#ifdef DEBUG
    static_assert(sizeof(Label) == 8 + 4 + 4 + 8);
#else
    static_assert(sizeof(Label) == 8 + 4 + 4);
#endif

    // Comparison of labels in heuristic
    struct LabelCompare
    {
        inline bool operator()(const Label* const a, const Label* const b)
        {
            return a->g < b->g;
        }
    };

    // Instance
    const Map& map_;

    // Global memory pool
    LabelPool& label_pool_;

    // Runs
    Vector<Node> goals_;
    Vector<Vector<IntCost>> h_;
    Time max_path_length_;

    // Temporary storage for each run
    PriorityQueue<Label, LabelCompare, true> open_;
    Vector<bool> visited_;

    // Label counter
#ifdef DEBUG
    Int nb_labels_;
#endif

  public:
    // Constructors
    Heuristic() = delete;
    Heuristic(const Map& map, LabelPool& label_pool);
    Heuristic(const Heuristic&) = delete;
    Heuristic(Heuristic&&) = delete;
    Heuristic& operator=(const Heuristic&) = delete;
    Heuristic& operator=(Heuristic&&) = delete;
    ~Heuristic() = default;

    // Getters
    inline auto max_path_length() const { return max_path_length_; }

    // Compute heuristic costs to a goal node
    const Vector<IntCost>& compute_h(const Node goal);

  private:
    // Check if a node has already been visited
    bool dominated(const Node n);

    // Generate labels
    void generate_start(const Node start);
    void generate(const Label* const current, const Node n);
    void generate_neighbours(const Label* const current);

    // Find h of each node to a goal
    void search(const Node goal, Vector<IntCost>& h);
};

}

#endif
