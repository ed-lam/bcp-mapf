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

String make_rectangle_state_string(const std::byte* const state, const Int nb_rect_crossings)
{
    String str;
    if (nb_rect_crossings > 0)
    {
        str = ", rect ";
        for (Int idx = 0; idx < nb_rect_crossings; ++idx)
        {
            const Int count = get_bitset(state, 2 * idx) + get_bitset(state, 2 * idx + 1);
            str += std::to_string(count);
        }
    }
    return str;
}

#ifdef USE_RECTANGLE_CLIQUE_CONFLICTS
inline bool rectangle_dominated(const Int size,
                                const Int* resources1,
                                const Int* resources2)
{
    // Checks whether the first label dominates the second label in the rectangle
    // resources.
    for (Int idx = 0; idx < size; ++idx)
        if (!(resources1[idx] % 2 <= resources2[idx]))
        {
            return false;
        }

    return true;
}

bool AStar::dominated_with_resources(Label* const new_label)
{
    err("Not yet supported with new priority queue");

    // Get number of resources.
    const auto nb_goal_crossings = static_cast<Int>(goal_crossings_.size());
    const auto nb_rect_crossings = static_cast<Int>(rectangle_crossings_.size());

    // Check all labels in the Pareto frontier.
#ifdef DEBUG
    bool dominated_existing = false;
#endif
    auto& existing_labels = frontier_with_resources_[NodeTime{new_label->nt}];
    for (auto it = existing_labels.begin(); it != existing_labels.end();)
    {
        auto existing_label = *it;
        if (isLE(existing_label->f, new_label->f) &&
            rectangle_dominated(nb_rect_crossings,
                                &existing_label->resources_[nb_goal_crossings],
                                &new_label->resources_[nb_goal_crossings]))
        {
            // Existing label dominates new label.
            debug_assert(isLE(existing_label->g, new_label->g));
//            new_label->dominated = true;
            debug_assert(!dominated_existing);
            return true;
        }
        else if (isLE(new_label->f, existing_label->f) &&
                 rectangle_dominated(nb_rect_crossings,
                                     &new_label->resources_[nb_goal_crossings],
                                     &existing_label->resources_[nb_goal_crossings]))
        {
            // New label dominates existing label.
            debug_assert(isLE(new_label->g, existing_label->g));
//            existing_label->dominated = true;
            std::swap(*it, existing_labels.back());
            existing_labels.pop_back();
#ifdef DEBUG
            dominated_existing = true;
#endif
        }
        else
        {
            ++it;
        }
    }

    // Not dominated.
    existing_labels.push_back(new_label);
    return false;
}
#endif

template<bool is_farkas>
Pair<Vector<NodeTime>, Cost> AStar::solve(const NodeTime start,
                                          const Node goal,
                                          const Time goal_earliest,
                                          const Time goal_latest,
                                          const Cost max_cost)
{
#ifdef USE_RECTANGLE_CLIQUE_CONFLICTS
    const auto without_resources = rectangle_crossings_.empty();
    if (without_resources)
    {
        return solve_internal<true, is_farkas>(start, goal, goal_earliest, goal_latest, max_cost);
    }
    else
    {
        return solve_internal<false, is_farkas>(start, goal, goal_earliest, goal_latest, max_cost);
    }
#else
    return solve_internal<true, is_farkas>(start, goal, goal_earliest, goal_latest, max_cost);
#endif
}


solve function

#ifdef USE_RECTANGLE_CLIQUE_CONFLICTS
const auto nb_rect_crossings = static_cast<Int>(rectangle_crossings_.size());
#else
constexpr Int nb_rect_crossings = 0;
#endif











generate function


// Check all rectangle crossings.
#ifdef USE_RECTANGLE_CLIQUE_CONFLICTS
if constexpr (!without_resources)
    {
        for (Int idx = 0; idx < nb_rect_crossings; ++idx)
        {
            // Get the rectangle crossing.
            const auto& crossing = rectangle_crossings_[idx];
            const auto pos = nb_goal_crossings + idx;

            // Check that times are ordered.
#ifdef DEBUG
            for (auto et = &(crossing.edges[0]);
                 et != &(crossing.edges[crossing.mid - 1]);
                 ++et)
            {
                debug_assert(et->t + 1 == (et + 1)->t);
            }
            for (auto et = &(crossing.edges[crossing.mid]);
                 et != &(crossing.edges[crossing.end - 1]);
                 ++et)
            {
                debug_assert(et->t + 1 == (et + 1)->t);
            }
#endif

            // Check offsets.
#ifdef DEBUG
            if (crossing.edges[0].t <= current->t && current->t <= crossing.edges[crossing.mid - 1].t)
            {
                const auto idx = current->t - crossing.edges[0].t;
                debug_assert(current->t == crossing.edges[idx].t);
            }
            if (crossing.edges[crossing.mid].t <= current->t && current->t <= crossing.edges[crossing.end - 1].t)
            {
                const auto idx = crossing.mid + (current->t - crossing.edges[crossing.mid].t);
                debug_assert(current->t == crossing.edges[idx].t);
            }
#endif

            // Compute the resource extension function.
            if (new_label->resources_[pos] == 2)
            {
                // Nothing happens if already past the rectangle.
            }
            else if (current->t > crossing.edges[crossing.end - 1].t)
            {
                // Set to skip the rectangle if past its last timestep.
                new_label->resources_[pos] = 2;
            }
            else if (new_label->resources_[pos] == 0)
            {
                // Entry side to rectangle not yet crossed. Check for entry.
                const auto idx = current->t - crossing.edges[0].t;
                if (d == crossing.edges[0].e.d &&
                    crossing.edges[0].t <= current->t && current->t <= crossing.edges[crossing.mid - 1].t &&
                    current->n == crossing.edges[idx].e.n)
                {
                    // Check.
                    debug_assert(current->t == crossing.edges[idx].t);

                    // Mark as entered.
                    new_label->resources_[pos] = 1;
                }
            }
            else if (new_label->resources_[pos] == 1)
            {
                debug_assert(current->t >= crossing.edges[0].t);
                if (d == crossing.edges[crossing.mid].e.d)
                {
                    // Already entered into the rectangle. Check for exit.
                    const auto idx = crossing.mid + (current->t - crossing.edges[crossing.mid].t);
                    if (crossing.edges[crossing.mid].t <= current->t && current->t <= crossing.edges[crossing.end - 1].t &&
                        current->n == crossing.edges[idx].e.n)
                    {
                        // Check.
                        debug_assert(current->t == crossing.edges[idx].t);

                        // Incur the penalty.
                        new_label->g -= crossing.dual;
                        new_label->resources_[pos] = 2;
                    }
                }
                else if (d != crossing.other_dir)
                {
                    // Going in the wrong direction such that it will never go through the
                    // exit side at the correct timesteps.
                    new_label->resources_[pos] = 2;
                }
            }
        }
    }
#endif