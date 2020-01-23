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

#include "AStar.h"
#include <cstddef>

#define isLE(x,y) ((x)-(y) <= (1e-06))

namespace TruffleHog
{

#ifdef DEBUG
static bool verbose = false;
#endif

static inline bool get_bitset(const std::byte* const bitset, const Int i)
{
    const auto idx = i / CHAR_BIT;
    const auto mask = std::byte(0x01) << (i % CHAR_BIT);
    return (bitset[idx] & mask) != std::byte(0x0);
}

static inline void set_bitset(std::byte* const bitset, const Int i)
{
    const auto idx = i / CHAR_BIT;
    const auto mask = std::byte(0x01) << (i % CHAR_BIT);
    bitset[idx] |= mask;
}

//static inline void clear_bitset(std::byte* const bitset, const Int i)
//{
//    const auto idx = i / CHAR_BIT;
//    const auto mask = std::byte(0x01) << (i % CHAR_BIT);
//    bitset[idx] &= ~mask;
//}

#ifdef DEBUG
String make_goal_state_string(const std::byte* const state, const Int nb_goal_crossings)
{
    String str;
    if (nb_goal_crossings > 0)
    {
        str = ", goal ";
        for (Int idx = 0; idx < nb_goal_crossings; ++idx)
        {
            if (get_bitset(state, idx))
            {
                str.push_back('1');
            }
            else
            {
                str.push_back('0');
            }
        }
    }
    return str;
}
#endif

AStar::AStar(const Map& map)
    : map_(map),
      label_pool_(),
      heuristic_(map, label_pool_),
      edge_penalties_(),
      time_finish_penalties_(),
#ifdef USE_GOAL_CONFLICTS
      goal_crossings_(),
#endif
      open_(map.size()),
      frontier_without_resources_(),
      h_(nullptr),
      time_finish_h_()
#ifdef DEBUG
    , nb_labels_(0)
#endif
{
}

AStar::Label* AStar::dominated_without_resources(Label* const new_label)
{
    // Check.
//#ifdef DEBUG
//    for (auto it = frontier_without_resources_.begin();
//         it != frontier_without_resources_.end();
//         ++it)
//    {
//        auto label = it->second;
//        open_.check_label(label);
//    }
//#endif

    // Try to put in the new label.
    auto [it, success] = frontier_without_resources_.emplace(
        NodeTime{new_label->nt}, new_label);

    // Check for dominance if a label already exists.
    if (!success)
    {
        auto existing_label = it->second;
        debug_assert(existing_label->nt == new_label->nt);
        if (isLE(existing_label->f, new_label->f))
        {
            // Existing label dominates new label.
            debug_assert(isLE(existing_label->g, new_label->g));

            // Dominated.
            return nullptr;
        }
        else
        {
            // New label dominates existing label.
            debug_assert(it == frontier_without_resources_.find(new_label->nt));
            debug_assert(!isLE(existing_label->g, new_label->g));

            // If the existing label has already been popped, push the new label. Otherwise, replace
            // the existing label with the new label.
            if (existing_label->pqueue_index >= 0)
            {
                new_label->pqueue_index = existing_label->pqueue_index;
                memcpy(existing_label, new_label, label_pool_.label_size());
                open_.decrease_key(existing_label);
            }
            else
            {
                debug_assert(new_label->pqueue_index == -1);
                memcpy(existing_label, new_label, label_pool_.label_size());
                open_.push(existing_label);
            }
            return existing_label;
        }
    }
    else
    {
        // Store the label.
        label_pool_.take_label();
        open_.push(new_label);

        // Not dominated.
        return new_label;
    }
}

template <bool without_resources>
void AStar::generate_start(const NodeTime start)
{
    // Get number of resources.
#ifdef USE_GOAL_CONFLICTS
    const auto nb_goal_crossings = static_cast<Int>(goal_crossings_.size());
#else
    constexpr Int nb_goal_crossings = 0;
#endif

    // Create label.
    auto new_label = reinterpret_cast<Label*>(label_pool_.get_label_buffer());
    memset(new_label, 0, label_pool_.label_size());
    const auto h_to_goal = (*h_)[start.n];
    const auto h_to_finish = start.t + h_to_goal < static_cast<Int>(time_finish_h_.size()) ?
                             time_finish_h_[start.t + h_to_goal] :
                             0.0;
    const auto h = h_to_goal + h_to_finish;
    new_label->f = h;
    new_label->nt = start.nt;
    new_label->pqueue_index = -1;
#ifdef DEBUG
    new_label->label_id = nb_labels_++;
#endif

    // Store the label.
    debug_assert(open_.empty());
    static_assert(without_resources);
    if constexpr (without_resources)
    {
        debug_assert(frontier_without_resources_.empty());
        dominated_without_resources(new_label);
    }

    // Print.
#ifdef DEBUG
    if (verbose)
    {
        println("   Generating start label {} (n {}, t {}, nt {}, position ({},{}), g {}, h {}, f {}{})",
                new_label->label_id,
                decltype(new_label->n){new_label->n},
                decltype(new_label->t){new_label->t},
                decltype(new_label->nt){new_label->nt},
                map_.get_x(new_label->n),
                map_.get_y(new_label->n),
                new_label->g,
                h,
                new_label->f,
                make_goal_state_string(&new_label->state_[0], nb_goal_crossings));
    }
#endif
}

void AStar::generate_end(Label* const current, const Cost max_cost)
{
    // Compute node-time.
    const NodeTime nt(-1, current->t);

    // Create label.
    auto new_label = reinterpret_cast<Label*>(label_pool_.get_label_buffer());
    memcpy(new_label, current, label_pool_.label_size());
    new_label->parent = current;
    new_label->g = current->g +
                   (current->t < static_cast<Time>(time_finish_penalties_.size()) ?
                    time_finish_penalties_[current->t] :
                    0.0);
    new_label->f = new_label->g;
    new_label->nt = nt.nt;
    new_label->pqueue_index = -1;
#ifdef DEBUG
    new_label->label_id = nb_labels_++;
#endif

    // Check if cost-infeasible.
    if (new_label->g > max_cost)
    {
        // Print.
#ifdef DEBUG
        if (verbose)
        {
            println("   Cost-infeasible goal label {} (n {}, t {}, nt {}, position ({},{}), g {})",
                    new_label->label_id,
                    decltype(current->n){current->n},
                    decltype(current->t){current->t},
                    decltype(current->nt){current->nt},
                    map_.get_x(current->n),
                    map_.get_y(current->n),
                    new_label->g);
        }
#endif

        // Done.
        return;
    }

    // Print.
#ifdef DEBUG
    if (verbose)
    {
        // Get number of resources.
#ifdef USE_GOAL_CONFLICTS
        const auto nb_goal_crossings = static_cast<Int>(goal_crossings_.size());
#else
        constexpr Int nb_goal_crossings = 0;
#endif

        // Print.
        println("   Generating goal label {} {} (t {}, g {}{})",
                new_label->label_id,
                fmt::ptr(new_label),
                decltype(new_label->t){new_label->t},
                new_label->g,
                make_goal_state_string(&new_label->state_[0], nb_goal_crossings));
    }
#endif

    // Store the label.
    label_pool_.take_label();
    open_.push(new_label);
}

template <bool without_resources>
void AStar::generate(Label* const current,
                     const Node node,
                     const Cost cost,
                     const Time goal_latest,
                     const Cost max_cost)
{
    // Compute node-time.
    const auto new_t = current->t + 1;
    const NodeTime nt(node, new_t);

    // Check if time-infeasible.
    const auto h_to_goal = (*h_)[nt.n];
    if (new_t + h_to_goal > goal_latest)
    {
        // Print.
#ifdef DEBUG
        if (verbose)
        {
            println("   Time-infeasible label (n {}, t {}, nt {}, position ({},{}))",
                    nt.n,
                    nt.t,
                    nt.nt,
                    map_.get_x(nt.n),
                    map_.get_y(nt.n));
        }
#endif

        // Done.
        return;
    }

    // Create label.
    auto new_label = reinterpret_cast<Label*>(label_pool_.get_label_buffer());
    memcpy(new_label, current, label_pool_.label_size());
    new_label->parent = current;
    new_label->g = current->g + cost;
    new_label->nt = nt.nt;
    new_label->reserved = reservation_table().is_reserved(nt);
//    new_label->dominated = false;
    new_label->pqueue_index = -1;
#ifdef DEBUG
    new_label->label_id = nb_labels_++;
#endif

    // Get number of resources.
#ifdef USE_GOAL_CONFLICTS
    const auto nb_goal_crossings = static_cast<Int>(goal_crossings_.size());
#else
    constexpr Int nb_goal_crossings = 0;
#endif

    // Check all goal crossings.
#ifdef USE_GOAL_CONFLICTS
    for (Int idx = 0; idx < nb_goal_crossings; ++idx)
    {
        const auto& crossing = goal_crossings_[idx];
        const auto used = get_bitset(new_label->state_, idx);

        if (!used && nt.n == crossing.nt.n && nt.t >= crossing.nt.t)
        {
            // Incur the penalty.
            new_label->g -= crossing.dual;
            set_bitset(new_label->state_, idx);
        }
    }
#endif

    // Compute f.
    const auto h_to_finish = new_t + h_to_goal < static_cast<Int>(time_finish_h_.size()) ?
                             time_finish_h_[new_t + h_to_goal] :
                             0.0;
    const auto h = h_to_goal + h_to_finish;
    new_label->f = new_label->g + h;

    // Check if cost-infeasible.
    if (new_label->f > max_cost)
    {
        // Print.
#ifdef DEBUG
        if (verbose)
        {
            println("   Cost-infeasible label {} (n {}, t {}, nt {}, position ({},{}), g {}, h {}, f {}{})",
                    new_label->label_id,
                    decltype(new_label->n){new_label->n},
                    decltype(new_label->t){new_label->t},
                    decltype(new_label->nt){new_label->nt},
                    map_.get_x(new_label->n),
                    map_.get_y(new_label->n),
                    new_label->g,
                    h,
                    new_label->f,
                    make_goal_state_string(&new_label->state_[0], nb_goal_crossings));
        }
#endif

        // Done.
        return;
    }

    // Store the label if not dominated.
#ifdef DEBUG
    auto new_label_copy = new_label;
#endif
    new_label = dominated_without_resources(new_label);

    // Print.
#ifdef DEBUG
    if (verbose)
    {
        if (new_label)
        {
            println("   Generating label {} {} (n {}, t {}, nt {}, position ({},{}), g {}, h {}, f {}{})",
                    new_label->label_id,
                    fmt::ptr(new_label),
                    decltype(new_label->n){new_label->n},
                    decltype(new_label->t){new_label->t},
                    decltype(new_label->nt){new_label->nt},
                    map_.get_x(new_label->n),
                    map_.get_y(new_label->n),
                    new_label->g,
                    h,
                    new_label->f,
                    make_goal_state_string(&new_label->state_[0], nb_goal_crossings));
        }
        else
        {
            println("   Dominated label {} (n {}, t {}, nt {}, position ({},{}), g {}, h {}, f {}{})",
                    new_label_copy->label_id,
                    decltype(new_label_copy->n){new_label_copy->n},
                    decltype(new_label_copy->t){new_label_copy->t},
                    decltype(new_label_copy->nt){new_label_copy->nt},
                    map_.get_x(new_label_copy->n),
                    map_.get_y(new_label_copy->n),
                    new_label_copy->g,
                    h,
                    new_label_copy->f,
                    make_goal_state_string(&new_label_copy->state_[0], nb_goal_crossings));
        }
    }
#endif
}

template<bool without_resources, IntCost default_cost>
void AStar::generate_neighbours(Label* const current,
                                const Node goal,
                                const Time goal_earliest,
                                const Time goal_latest,
                                const Cost max_cost)
{
    // Print.
#ifdef DEBUG
    if (verbose)
    {
#ifdef USE_GOAL_CONFLICTS
        const auto nb_goal_crossings = static_cast<Int>(goal_crossings_.size());
#else
        constexpr Int nb_goal_crossings = 0;
#endif
        println("Expanding label {} {} (n {}, t {}, nt {}, position ({},{}), g {}, h {}, f {}{})",
                current->label_id,
                fmt::ptr(current),
                decltype(current->n){current->n},
                decltype(current->t){current->t},
                decltype(current->nt){current->nt},
                map_.get_x(current->n),
                map_.get_y(current->n),
                current->g,
                current->f - current->g,
                current->f,
                make_goal_state_string(&current->state_[0], nb_goal_crossings));
    }
#endif

    // Get edge costs.
    const auto edge_costs = edge_penalties_.get_edge_costs<default_cost>(current->nt);

    // Expand in five directions.
    const auto current_n = current->n;
    debug_assert(edge_costs.north >= 0 || std::isnan(edge_costs.north));
    debug_assert(edge_costs.south >= 0 || std::isnan(edge_costs.south));
    debug_assert(edge_costs.east >= 0 || std::isnan(edge_costs.east));
    debug_assert(edge_costs.west >= 0 || std::isnan(edge_costs.west));
    debug_assert(edge_costs.wait >= 0 || std::isnan(edge_costs.wait));
    if (const auto next_n = map_.get_north(current_n);
        map_[next_n] && !std::isnan(edge_costs.north))
    {
        generate<without_resources>(current,
                                    next_n,
                                    edge_costs.north,
                                    goal_latest,
                                    max_cost);
    }
    if (const auto next_n = map_.get_south(current_n);
        map_[next_n] && !std::isnan(edge_costs.south))
    {
        generate<without_resources>(current,
                                    next_n,
                                    edge_costs.south,
                                    goal_latest,
                                    max_cost);
    }
    if (const auto next_n = map_.get_east(current_n);
        map_[next_n] && !std::isnan(edge_costs.east))
    {
        generate<without_resources>(current,
                                    next_n,
                                    edge_costs.east,
                                    goal_latest,
                                    max_cost);
    }
    if (const auto next_n = map_.get_west(current_n);
        map_[next_n] && !std::isnan(edge_costs.west))
    {
        generate<without_resources>(current,
                                    next_n,
                                    edge_costs.west,
                                    goal_latest,
                                    max_cost);
    }
    if (const auto next_n = map_.get_wait(current_n);
        map_[next_n] && !std::isnan(edge_costs.wait))
    {
        generate<without_resources>(current,
                                    next_n,
                                    edge_costs.wait,
                                    goal_latest,
                                    max_cost);
    }

    // Expand to the end dummy node.
    if (current_n == goal && current->t >= goal_earliest)
    {
        generate_end(current, max_cost);
    }
}
template
void AStar::generate_neighbours<true, 0>(Label* const current,
                                         const Node goal,
                                         const Time goal_earliest,
                                         const Time goal_latest,
                                         const Cost max_cost);
template
void AStar::generate_neighbours<true, 1>(Label* const current,
                                         const Node goal,
                                         const Time goal_earliest,
                                         const Time goal_latest,
                                         const Cost max_cost);

template<bool is_farkas>
Pair<Vector<NodeTime>, Cost> AStar::solve(const NodeTime start,
                                          const Node goal,
                                          const Time goal_earliest,
                                          const Time goal_latest,
                                          const Cost max_cost)
{
    return solve_internal<true, is_farkas>(start, goal, goal_earliest, goal_latest, max_cost);
}
template Pair<Vector<NodeTime>, Cost> AStar::solve<false>(const NodeTime start,
                                                          const Node goal,
                                                          const Time goal_earliest,
                                                          const Time goal_latest,
                                                          const Cost max_cost);
template Pair<Vector<NodeTime>, Cost> AStar::solve<true>(const NodeTime start,
                                                         const Node goal,
                                                         const Time goal_earliest,
                                                         const Time goal_latest,
                                                         const Cost max_cost);

template<bool without_resources, bool is_farkas>
Pair<Vector<NodeTime>, Cost> AStar::solve_internal(const NodeTime start,
                                                   const Node goal,
                                                   const Time goal_earliest,
                                                   const Time goal_latest,
                                                   const Cost max_cost)
{
    // Create output.
    Pair<Vector<NodeTime>, Cost> output;
    auto& path = output.first;
    auto& path_cost = output.second;

    // Get h values to the goal node. Compute them if necessary.
    debug_assert(heuristic_.max_path_length() >= 1);
    h_ = &heuristic_.compute_h(goal);

    // Calculate the default edge cost.
    constexpr IntCost default_cost = is_farkas ? 0 : 1;

    // Remove vertices with default outgoing costs.
//    edge_duals_.clean_up(default_cost);

    // Get number of resources.
#ifdef USE_GOAL_CONFLICTS
    const auto nb_goal_crossings = static_cast<Int>(goal_crossings_.size());
#else
    constexpr Int nb_goal_crossings = 0;
#endif

    // Print.
    if constexpr (without_resources)
    {
        debugln("Solving from ({},{}) at time {} to ({},{}) between times {} and {} "
                "without resources",
                map_.get_x(start.n),
                map_.get_y(start.n),
                start.t,
                map_.get_x(goal),
                map_.get_y(goal),
                goal_earliest,
                goal_latest);
    }
    else
    {
        debugln("Solving from ({},{}) at time {} to ({},{}) between times {} and {}",
                map_.get_x(start.n),
                map_.get_y(start.n),
                start.t,
                map_.get_x(goal),
                map_.get_y(goal),
                goal_earliest,
                goal_latest);
    }

    // Reset.
    const auto nb_states = nb_goal_crossings;
    label_pool_.reset(sizeof(Label) + (nb_states / CHAR_BIT) + (nb_states % CHAR_BIT != 0));
    open_.clear();
    static_assert(without_resources);
    if constexpr (without_resources)
    {
        frontier_without_resources_.clear();
    }

    // Compute h-value to reach the end dummy node.
    time_finish_h_.resize(time_finish_penalties_.size());
    std::copy(time_finish_penalties_.begin(), time_finish_penalties_.end(), time_finish_h_.begin());
    for (Time t = 0; t < static_cast<Time>(time_finish_h_.size()); ++t)
    {
        time_finish_h_[t] = time_finish_penalties_.size() - t;
        for (Time i = t; i < static_cast<Time>(time_finish_penalties_.size()); ++i)
            if (i - t + time_finish_penalties_[i] < time_finish_h_[t])
            {
                time_finish_h_[t] = i - t + time_finish_penalties_[i];
            }
    }

    // Create label at the start node-time.
    generate_start<without_resources>(start);

    // Main loop.
    while (!open_.empty())
    {
        // Get a label from priority queue.
        const auto current = open_.top();
        open_.pop();

        // Expand the neighbours of the current label or exit if the goal is reached.
        if (current->n != -1) [[likely]]
        {
            // Generate neighbours.
            generate_neighbours<without_resources, default_cost>(current,
                                                                 goal,
                                                                 goal_earliest,
                                                                 goal_latest,
                                                                 max_cost);
        }
        else
        {
            // Get the label of the actual goal cell.
            auto parent = current->parent;

            // Store the path cost.
            path_cost = current->g;

            // Store the path.
            for (auto l = parent; l; l = l->parent)
            {
                path.push_back(l->nt);
            }
            std::reverse(path.begin(), path.end());

            // Print.
#ifdef DEBUG
            if (verbose)
            {
                println("Reached goal at label {} {} (n {}, t {}, nt {}, position ({},{}), g {}, h {}, f {}{})",
                        current->label_id,
                        fmt::ptr(current),
                        decltype(parent->n){parent->n},
                        decltype(parent->t){parent->t},
                        decltype(parent->nt){parent->nt},
                        map_.get_x(parent->n),
                        map_.get_y(parent->n),
                        current->g,
                        current->f - current->g,
                        current->f,
                        make_goal_state_string(&current->state_[0], nb_goal_crossings));

                fmt::print("Found path with cost {}: ", path_cost);
                for (const auto nt : path)
                {
                    fmt::print("({},{}) ", map_.get_x(nt.n), map_.get_y(nt.n));
                }
                println("");
            }
#endif

            // Check.
            debug_assert(path_cost <= max_cost);
            debug_assert(goal_earliest <= current->t && current->t <= goal_latest);

            // Finish.
            break;
        }
    }

    // Retrieve path.
#ifdef DEBUG
    if (verbose)
    {
        println("=======================================");
    }
#endif

    // Return.
    return output;
}
template Pair<Vector<NodeTime>, Cost> AStar::solve_internal<true, false>(const NodeTime start,
                                                                         const Node goal,
                                                                         const Time goal_earliest,
                                                                         const Time goal_latest,
                                                                         const Cost max_cost);
template Pair<Vector<NodeTime>, Cost> AStar::solve_internal<true, true>(const NodeTime start,
                                                                        const Node goal,
                                                                        const Time goal_earliest,
                                                                        const Time goal_latest,
                                                                        const Cost max_cost);

#ifdef DEBUG

void AStar::set_verbose(const bool on)
{
    verbose = on;
}

void AStar::print_crossings()
{
#ifdef USE_GOAL_CONFLICTS
    if (!goal_crossings_.empty())
    {
        println("Goal crossings:");
        for (const auto& crossing : goal_crossings_)
        {
            println("   nt: {}, n: {}, t: {}, dual: {}",
                    crossing.nt.nt, crossing.nt.n, crossing.nt.t, crossing.dual);
        }
    }
#endif
}
#endif

}
