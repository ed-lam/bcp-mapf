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

#include "pricing/distance_heuristic.h"

#define MAX_PATH_LENGTH_FACTOR 2

Heuristic::Heuristic(const Map& map) :
    map_(map),
    h_(),
    max_path_length_(-1),
    label_pool_(),
    open_(),
    visited_(map_.size())
#ifdef DEBUG
  , nb_labels_(0)
#endif
{
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
    auto new_label = reinterpret_cast<Label*>(label_pool_.get_buffer<true, false>());
#ifdef DEBUG
    new_label->label_id = nb_labels_++;
    new_label->parent = nullptr;
#endif
    new_label->n = start;
    new_label->g = 0;

    // Store the label.
    dominated(start);
    open_.push(new_label);

    // Print.
    debugln("    Generating start label {} (n {}, xy ({},{}), g {})",
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
        auto new_label = reinterpret_cast<Label*>(label_pool_.get_buffer<true, false>());
#ifdef DEBUG
        new_label->label_id = nb_labels_++;
        new_label->parent = current;
#endif
        new_label->n = n;
        new_label->g = current->g + 1;

        // Store the label.
        open_.push(new_label);

        // Print.
        debugln("    Generating label {} (n {}, xy ({},{}), g {})",
                new_label->label_id,
                new_label->n,
                map_.get_x(new_label->n),
                map_.get_y(new_label->n),
                new_label->g);
    }
    else
    {
        debugln("    Dominated label (n {}, xy ({},{}), g {})",
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
    if (const auto next_n = map_.get_north(current_n); map_[next_n])
    {
        generate(current, next_n);
    }
    if (const auto next_n = map_.get_south(current_n); map_[next_n])
    {
        generate(current, next_n);
    }
    if (const auto next_n = map_.get_east(current_n); map_[next_n])
    {
        generate(current, next_n);
    }
    if (const auto next_n = map_.get_west(current_n); map_[next_n])
    {
        generate(current, next_n);
    }
}

void Heuristic::search(const Node goal, Vector<IntCost>& h)
{
    // Reset.
    label_pool_.reset(sizeof(Label));
    open_.clear();
    std::fill(visited_.begin(), visited_.end(), false);

    // Solve.
    debug_assert(h.empty());
    h.resize(map_.size());
    generate_start(goal);
    while (!open_.empty())
    {
        // Get a label from priority queue.
        const auto current = open_.top();
        open_.pop();

        // Print.
        debugln("Expanding label {} (n {}, xy ({},{}), g {})",
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

const Vector<IntCost>& Heuristic::get_h(const Node goal)
{
    auto& h = h_[goal];
    if (h.empty())
    {
        // Compute the h values for this goal.
        search(goal, h);

        // Get estimate of longest path length.
        {
            auto it = std::max_element(h.begin(), h.end());
            debug_assert(it != h.end());
            const auto new_max_path_length = MAX_PATH_LENGTH_FACTOR * *it;
            max_path_length_ = std::max(new_max_path_length, max_path_length_);
        }
    }
    return h;
}
