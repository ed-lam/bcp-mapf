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

//#define PRINT_DEBUG

#include "Heuristic.h"

namespace TruffleHog
{

Heuristic::Heuristic(const Map& map, LabelPool& label_pool)
    : map_(map),
      label_pool_(label_pool),
      goals_(),
      h_(),
      max_path_length_(-1),
      open_(),
      visited_(map_.size())
#ifdef DEBUG
    , nb_labels_(0)
#endif
{
    goals_.reserve(1000);
    h_.reserve(1000);
}

bool Heuristic::dominated(const Node n)
{
    if (!visited_[n])
    {
        // Not yet visited.
        visited_[n] = true;
        return false;
    }
    else
    {
        // Already visited.
        return true;
    }
}

void Heuristic::generate_start(const Node start)
{
    // Create label.
    auto new_label = reinterpret_cast<Label*>(label_pool_.get_label_buffer());
    new_label->parent = nullptr;
    new_label->g = 0;
    new_label->n = start;
#ifdef DEBUG
    new_label->label_id = nb_labels_++;
#endif

    // Store the label.
    dominated(start);
    label_pool_.take_label();
    open_.push(new_label);

    // Print.
    debugln("   Generating start label {} (n {}, position ({},{}), g {})",
            new_label->label_id,
            new_label->n,
            map_.get_x(new_label->n),
            map_.get_y(new_label->n),
            new_label->g);
}

void Heuristic::generate(const Label* const current, const Node n)
{
    if (!dominated(n))
    {
        // Create label.
        auto new_label = reinterpret_cast<Label*>(label_pool_.get_label_buffer());
        new_label->parent = current;
        new_label->g = current->g + 1;
        new_label->n = n;
#ifdef DEBUG
        new_label->label_id = nb_labels_++;
#endif

        // Store the label.
        label_pool_.take_label();
        open_.push(new_label);

        // Print.
        debugln("   Generating label {} (n {}, position ({},{}), g {})",
                new_label->label_id,
                new_label->n,
                map_.get_x(new_label->n),
                map_.get_y(new_label->n),
                new_label->g);
    }
    else
    {
        debugln("   Not generating label (n {}, position ({},{}), g {})",
                n,
                map_.get_x(n),
                map_.get_y(n),
                current->g + 1);
    }
}

void Heuristic::generate_neighbours(const Label* const current)
{
    // Expand in four directions.
    const auto current_n = current->n;
    if (const auto new_n = map_.get_north(current_n); map_[new_n])
    {
        generate(current, new_n);
    }
    if (const auto new_n = map_.get_south(current_n); map_[new_n])
    {
        generate(current, new_n);
    }
    if (const auto new_n = map_.get_east(current_n); map_[new_n])
    {
        generate(current, new_n);
    }
    if (const auto new_n = map_.get_west(current_n); map_[new_n])
    {
        generate(current, new_n);
    }
}

void Heuristic::search(const Node goal, Vector<IntCost>& h)
{
    // Reset.
    label_pool_.reset(sizeof(Label));
    open_.clear();
    std::fill(visited_.begin(), visited_.end(), false);

    // Create label at the start node.
    generate_start(goal);

    // Main loop.
    debug_assert(static_cast<Int>(h.size()) == map_.size());
    while (!open_.empty())
    {
        // Get a label from priority queue.
        const auto current = open_.top();
        open_.pop();

        // Print.
        debugln("Expanding label {} (n {}, position ({},{}), g {})",
                current->label_id,
                current->n,
                map_.get_x(current->n),
                map_.get_y(current->n),
                current->g);

        // Store h.
        h[current->n] = current->g;

        // Generate neighbours.
        generate_neighbours(current);
    }
    debugln("=======================================");
}

const Vector<IntCost>& Heuristic::compute_h(const Node goal)
{
    auto it = std::find(goals_.begin(), goals_.end(), goal);
    if (it == goals_.end())
    {
        // Print.
        debugln("Computing h to goal node {}, position ({},{})",
                goal, map_.get_x(goal), map_.get_y(goal));

        // Add a new goal since no h-values exist for this goal yet.
        goals_.emplace_back(goal);

        // Compute the h-values for this goal.
        auto& h = h_.emplace_back(map_.size(), -1);
        search(goal, h);

        // Get estimate of longest path length.
        {
            auto it = std::max_element(h.begin(), h.end());
            debug_assert(it != h.end());
            const auto new_max_path_length = 2 * *it;
            if (new_max_path_length > max_path_length_)
            {
                max_path_length_ = new_max_path_length;
                debugln("Max path length: {}", max_path_length_);
            }
        }

        // Done.
        return h;
    }
    else
    {
        // Print.
        debugln("Retrieving pre-computed h to goal node {}, position ({},{})",
                goal, map_.get_x(goal), map_.get_y(goal));

        // Retrieve existing h-values.
        const auto idx = it - goals_.begin();
        return h_[idx];
    }
}

}
