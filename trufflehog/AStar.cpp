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

#define EPS (1e-6)
#define isEQ(x, y) (std::abs((x)-(y)) <= (EPS))
#define isLT(x, y) ((x)-(y) < -(EPS))
#define isLE(x, y) ((x)-(y) <= (EPS))
#define isGT(x, y) ((x)-(y) > (EPS))
#define isGE(x, y) ((x)-(y) >= -(EPS))

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
            str.push_back('0' + static_cast<char>(get_bitset(state, idx)));
        }
    }
    return str;
}
#endif

bool AStar::Data::can_be_better(const Data& previous_data)
{
    if (cost_offset < previous_data.cost_offset ||
        waypoints != previous_data.waypoints ||
        latest_goal_time != previous_data.latest_goal_time ||
        earliest_goal_time != previous_data.earliest_goal_time)
    {
        return true;
    }

    for (const auto& [nt, previous_edge_costs] : previous_data.edge_penalties)
        if (previous_edge_costs.used)
        {
            const auto it = edge_penalties.find(nt);
            if (it == edge_penalties.end())
            {
                return true;
            }
            const auto& current_edge_costs = it->second;
            if (current_edge_costs.north < previous_edge_costs.north ||
                current_edge_costs.south < previous_edge_costs.south ||
                current_edge_costs.east < previous_edge_costs.east ||
                current_edge_costs.west < previous_edge_costs.west ||
                current_edge_costs.wait < previous_edge_costs.wait)
            {
                return true;
            }
        }

    debug_assert(latest_visit_time.size() == previous_data.latest_visit_time.size());
    for (Node n = 0; n < static_cast<Node>(latest_visit_time.size()); ++n)
        if (latest_visit_time[n] > previous_data.latest_visit_time[n])
        {
            return true;
        }

    if (finish_time_penalties.size() != previous_data.finish_time_penalties.size())
    {
        return true;
    }
    for (Time t = 0; t < static_cast<Time>(finish_time_penalties.size()); ++t)
        if (finish_time_penalties[t] < previous_data.finish_time_penalties[t])
        {
            return true;
        }

#ifdef USE_GOAL_CONFLICTS
    if (goal_penalties.size() != previous_data.goal_penalties.size())
    {
        return true;
    }
    for (Int idx = 0; idx < static_cast<Int>(goal_penalties.size()); ++idx)
        if (goal_penalties[idx].nt != previous_data.goal_penalties[idx].nt ||
            goal_penalties[idx].cost < previous_data.goal_penalties[idx].cost)
        {
            return true;
        }
#endif

    return false;
}

AStar::AStar(const Map& map) :
    map_(map),

    data_(),

    h_node_to_waypoint_(nullptr),
    h_waypoint_to_goal_(),
    heuristic_(map),
    label_pool_(),
#ifdef USE_RESERVATION_TABLE
    open_(map.size()),
#else
    open_(),
#endif
    frontier_without_resources_(),
    frontier_with_resources_(),
#ifdef DEBUG
    nb_labels_(0),
#endif

    sipp_intervals_(map_.size())
{
}

template<bool has_resources>
void AStar::generate_start()
{
    // Get data.
    const auto& [start,
                 waypoints,
                 goal,
                 earliest_goal_time,
                 latest_goal_time,
                 cost_offset,
                 latest_visit_time,
                 edge_penalties,
                 finish_time_penalties
#ifdef USE_GOAL_CONFLICTS
               , goal_penalties
#endif
    ] = data_;
    constexpr auto start_time = 0;
    const auto waypoint_time = waypoints[0].t;

    // Create label.
    auto new_label = reinterpret_cast<Label*>(label_pool_.get_label_buffer());
    memset(new_label, 0, label_pool_.label_size());
#ifdef DEBUG
    new_label->label_id = nb_labels_++;
#endif
    const auto h_node_to_waypoint = std::max((*h_node_to_waypoint_)[start], waypoint_time - start_time);
    const auto h_waypoint_to_goal = h_waypoint_to_goal_[0];
    const auto h_goal_to_finish = finish_time_penalties.get_h(start_time + h_node_to_waypoint + h_waypoint_to_goal);
    const auto h = std::max(h_node_to_waypoint, waypoint_time - start_time) + h_waypoint_to_goal + h_goal_to_finish;
    debug_assert(h_node_to_waypoint >= 0);
    debug_assert(h_waypoint_to_goal >= 0);
    debug_assert(h_goal_to_finish >= 0);
    new_label->g = cost_offset;
    new_label->f = cost_offset + h;
    new_label->nt = NodeTime{start, start_time}.nt;

    // Store the label.
    debug_assert(open_.empty());
    debug_assert(frontier_without_resources_.empty());
    debug_assert(frontier_with_resources_.empty());
    dominated<has_resources>(new_label);

    // Print.
#ifdef DEBUG
    if (verbose)
    {
#ifdef USE_GOAL_CONFLICTS
        const auto nb_goal_penalties = goal_penalties.size();
#else
        const auto nb_goal_penalties = 0;
#endif
        println("    Generating start label {} {} (n {}, t {}, nt {}, xy ({},{}), g {}, h {}, f {}{})",
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
                make_goal_state_string(&new_label->state_[0], nb_goal_penalties));
    }
#endif
}

template<bool has_resources, bool is_sipp>
void AStar::generate(Label* const current,
                     const Node next_n,
                     const Time next_t,
                     const Cost cost,
                     const Waypoint w,
                     const Time waypoint_time)
{
    // Get data.
    const auto& [start,
                 waypoints,
                 goal,
                 earliest_goal_time,
                 latest_goal_time,
                 cost_offset,
                 latest_visit_time,
                 edge_penalties,
                 finish_time_penalties
#ifdef USE_GOAL_CONFLICTS
               , goal_penalties
#endif
    ] = data_;

    // Compute the vertex (node-time) of the new label.
    const NodeTime next_nt{next_n, next_t};

    // Check if time-infeasible.
    const auto h_node_to_waypoint = std::max((*h_node_to_waypoint_)[next_nt.n], waypoint_time - next_t);
    const auto h_waypoint_to_goal = h_waypoint_to_goal_[w];
    debug_assert(h_node_to_waypoint >= 0);
    debug_assert(h_waypoint_to_goal >= 0);
    if (next_t + h_node_to_waypoint > waypoint_time || next_t + h_node_to_waypoint + h_waypoint_to_goal > latest_goal_time)
    {
        // Print.
#ifdef DEBUG
        if (verbose)
        {
            println("    Time-infeasible label (n {}, t {}, nt {}, xy ({},{}))",
                    next_nt.n,
                    next_nt.t,
                    next_nt.nt,
                    map_.get_x(next_nt.n),
                    map_.get_y(next_nt.n));
        }
#endif

        // Done.
        return;
    }

    // Create label.
    auto next_label = reinterpret_cast<Label*>(label_pool_.get_label_buffer());
    memcpy(next_label, current, label_pool_.label_size());
#ifdef DEBUG
    next_label->label_id = nb_labels_++;
#endif
    next_label->parent = current;
    next_label->g = current->g + cost;
    next_label->nt = next_nt.nt;
#ifdef USE_RESERVATION_TABLE
    next_label->reserves += reservation_table().is_reserved(next_nt);
#endif

    // Check all goal crossings.
#ifdef USE_GOAL_CONFLICTS
    for (Int idx = 0; idx < goal_penalties.size(); ++idx)
    {
        const auto [goal_nt, goal_cost] = goal_penalties[idx];

        const auto crossed = get_bitset(next_label->state_, idx);
        if (!crossed && ((next_n == goal_nt.n && next_t >= goal_nt.t) ||
                         (is_sipp && current->n == goal_nt.n && next_t - 1 >= goal_nt.t)))
        {
            // Incur the penalty.
            next_label->g += goal_cost;
            set_bitset(next_label->state_, idx);
        }
    }
#endif

    // Compute f.
    const auto h_goal_to_finish = finish_time_penalties.get_h(next_t + h_node_to_waypoint + h_waypoint_to_goal);
    const auto h = std::max(h_node_to_waypoint, waypoint_time - next_t) + h_waypoint_to_goal + h_goal_to_finish;
    next_label->f = next_label->g + h;
    debug_assert(isGE(next_label->g, current->g + 1));
    debug_assert(isGE(next_label->f, current->f));

    // Check if cost-infeasible.
    if (isGE(next_label->f, 0))
    {
        // Print.
#ifdef DEBUG
        if (verbose)
        {
#ifdef USE_GOAL_CONFLICTS
            const auto nb_goal_penalties = goal_penalties.size();
#else
            const auto nb_goal_penalties = 0;
#endif
            println("    Cost-infeasible label {} {} (n {}, t {}, nt {}, xy ({},{}), g {}, h {}, f {}{})",
                    next_label->label_id,
                    fmt::ptr(next_label),
                    decltype(next_label->n){next_label->n},
                    decltype(next_label->t){next_label->t},
                    decltype(next_label->nt){next_label->nt},
                    map_.get_x(next_label->n),
                    map_.get_y(next_label->n),
                    next_label->g,
                    h,
                    next_label->f,
                    make_goal_state_string(&next_label->state_[0], nb_goal_penalties));
        }
#endif

        // Done.
        return;
    }

    // Store the label if not dominated.
#ifdef DEBUG
    auto next_label_copy = next_label;
#endif
    next_label = dominated<has_resources>(next_label);

    // Print.
#ifdef DEBUG
    if (verbose)
    {
        if (next_label)
        {
#ifdef USE_GOAL_CONFLICTS
            const auto nb_goal_penalties = goal_penalties.size();
#else
            const auto nb_goal_penalties = 0;
#endif
            println("    Generating label {} {} (n {}, t {}, nt {}, xy ({},{}), g {}, h {}, f {}{})",
                    next_label_copy->label_id,
                    fmt::ptr(next_label_copy),
                    decltype(next_label_copy->n){next_label_copy->n},
                    decltype(next_label_copy->t){next_label_copy->t},
                    decltype(next_label_copy->nt){next_label_copy->nt},
                    map_.get_x(next_label_copy->n),
                    map_.get_y(next_label_copy->n),
                    next_label_copy->g,
                    h,
                    next_label_copy->f,
                    make_goal_state_string(&next_label_copy->state_[0], nb_goal_penalties));
        }
        else
        {
#ifdef USE_GOAL_CONFLICTS
            const auto nb_goal_penalties = goal_penalties.size();
#else
            const auto nb_goal_penalties = 0;
#endif
            println("    Dominated label {} {} (n {}, t {}, nt {}, xy ({},{}), g {}, h {}, f {}{})",
                    next_label_copy->label_id,
                    fmt::ptr(next_label_copy),
                    decltype(next_label_copy->n){next_label_copy->n},
                    decltype(next_label_copy->t){next_label_copy->t},
                    decltype(next_label_copy->nt){next_label_copy->nt},
                    map_.get_x(next_label_copy->n),
                    map_.get_y(next_label_copy->n),
                    next_label_copy->g,
                    h,
                    next_label_copy->f,
                    make_goal_state_string(&next_label_copy->state_[0], nb_goal_penalties));
        }
    }
#endif
}

template<bool has_resources, bool is_sipp>
void AStar::generate_last_segment(Label* const current, const Node next_n, const Time next_t, const Cost cost)
{
    // Get data.
    const auto& [start,
                 waypoints,
                 goal,
                 earliest_goal_time,
                 latest_goal_time,
                 cost_offset,
                 latest_visit_time,
                 edge_penalties,
                 finish_time_penalties
#ifdef USE_GOAL_CONFLICTS
               , goal_penalties
#endif
    ] = data_;

    // Compute the vertex (node-time) of the new label.
    const NodeTime next_nt{next_n, next_t};

    // Check if time-infeasible.
    const auto h_node_to_waypoint = std::max((*h_node_to_waypoint_)[next_nt.n], earliest_goal_time - next_t);
    debug_assert(h_node_to_waypoint >= 0);
    if (next_t + h_node_to_waypoint > latest_goal_time)
    {
        // Print.
#ifdef DEBUG
        if (verbose)
        {
            println("    Time-infeasible label (n {}, t {}, nt {}, xy ({},{}))",
                    next_nt.n,
                    next_nt.t,
                    next_nt.nt,
                    map_.get_x(next_nt.n),
                    map_.get_y(next_nt.n));
        }
#endif

        // Done.
        return;
    }

    // Create label.
    auto next_label = reinterpret_cast<Label*>(label_pool_.get_label_buffer());
    memcpy(next_label, current, label_pool_.label_size());
#ifdef DEBUG
    next_label->label_id = nb_labels_++;
#endif
    next_label->parent = current;
    next_label->g = current->g + cost;
    next_label->nt = next_nt.nt;
#ifdef USE_RESERVATION_TABLE
    next_label->reserves += reservation_table().is_reserved(next_nt);
#endif

    // Check all goal crossings.
#ifdef USE_GOAL_CONFLICTS
    for (Int idx = 0; idx < goal_penalties.size(); ++idx)
    {
        const auto [goal_nt, goal_cost] = goal_penalties[idx];

        const auto crossed = get_bitset(next_label->state_, idx);
        if (!crossed && ((next_n == goal_nt.n && next_t >= goal_nt.t) ||
                         (is_sipp && current->n == goal_nt.n && next_t - 1 >= goal_nt.t)))
        {
            // Incur the penalty.
            next_label->g += goal_cost;
            set_bitset(next_label->state_, idx);
        }
    }
#endif

    // Compute f.
    const auto h_goal_to_finish = finish_time_penalties.get_h(next_t + h_node_to_waypoint);
    const auto h = std::max(h_node_to_waypoint, earliest_goal_time - next_t) + h_goal_to_finish;
    next_label->f = next_label->g + h;
    debug_assert(isGE(next_label->g, current->g + 1));
    debug_assert(isGE(next_label->f, current->f));

    // Check if cost-infeasible.
    if (isGE(next_label->f, 0))
    {
        // Print.
#ifdef DEBUG
        if (verbose)
        {
#ifdef USE_GOAL_CONFLICTS
            const auto nb_goal_penalties = goal_penalties.size();
#else
            const auto nb_goal_penalties = 0;
#endif
            println("    Cost-infeasible label {} {} (n {}, t {}, nt {}, xy ({},{}), g {}, h {}, f {}{})",
                    next_label->label_id,
                    fmt::ptr(next_label),
                    decltype(next_label->n){next_label->n},
                    decltype(next_label->t){next_label->t},
                    decltype(next_label->nt){next_label->nt},
                    map_.get_x(next_label->n),
                    map_.get_y(next_label->n),
                    next_label->g,
                    h,
                    next_label->f,
                    make_goal_state_string(&next_label->state_[0], nb_goal_penalties));
        }
#endif

        // Done.
        return;
    }

    // Store the label if not dominated.
#ifdef DEBUG
    auto next_label_copy = next_label;
#endif
    next_label = dominated<has_resources>(next_label);

    // Print.
#ifdef DEBUG
    if (verbose)
    {
        if (next_label)
        {
#ifdef USE_GOAL_CONFLICTS
            const auto nb_goal_penalties = goal_penalties.size();
#else
            const auto nb_goal_penalties = 0;
#endif
            println("    Generating label {} {} (n {}, t {}, nt {}, xy ({},{}), g {}, h {}, f {}{})",
                    next_label_copy->label_id,
                    fmt::ptr(next_label_copy),
                    decltype(next_label_copy->n){next_label_copy->n},
                    decltype(next_label_copy->t){next_label_copy->t},
                    decltype(next_label_copy->nt){next_label_copy->nt},
                    map_.get_x(next_label_copy->n),
                    map_.get_y(next_label_copy->n),
                    next_label_copy->g,
                    h,
                    next_label_copy->f,
                    make_goal_state_string(&next_label_copy->state_[0], nb_goal_penalties));
        }
        else
        {
#ifdef USE_GOAL_CONFLICTS
            const auto nb_goal_penalties = goal_penalties.size();
#else
            const auto nb_goal_penalties = 0;
#endif
            println("    Dominated label {} {} (n {}, t {}, nt {}, xy ({},{}), g {}, h {}, f {}{})",
                    next_label_copy->label_id,
                    fmt::ptr(next_label_copy),
                    decltype(next_label_copy->n){next_label_copy->n},
                    decltype(next_label_copy->t){next_label_copy->t},
                    decltype(next_label_copy->nt){next_label_copy->nt},
                    map_.get_x(next_label_copy->n),
                    map_.get_y(next_label_copy->n),
                    next_label_copy->g,
                    h,
                    next_label_copy->f,
                    make_goal_state_string(&next_label_copy->state_[0], nb_goal_penalties));
        }
    }
#endif
}

template<bool has_resources, IntCost default_cost>
void AStar::generate_neighbours(Label* const current, const Waypoint w, const Time waypoint_time)
{
    constexpr bool is_sipp = false;

    // Get data.
    auto& [start,
           waypoints,
           goal,
           earliest_goal_time,
           latest_goal_time,
           cost_offset,
           latest_visit_time,
           edge_penalties,
           finish_time_penalties
#ifdef USE_GOAL_CONFLICTS
         , goal_penalties
#endif
    ] = data_;

    // Print.
#ifdef DEBUG
    if (verbose)
    {
#ifdef USE_GOAL_CONFLICTS
        const auto nb_goal_penalties = goal_penalties.size();
#else
        const auto nb_goal_penalties = 0;
#endif
        println("Expanding label {} {} (n {}, t {}, nt {}, xy ({},{}), g {}, h {}, f {}{})",
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
                make_goal_state_string(&current->state_[0], nb_goal_penalties));
    }
#endif

    // Expand in five directions.
    const auto edge_costs = edge_penalties.get_edge_costs<default_cost>(current->nt);
    const auto current_n = current->n;
    const auto next_t = current->t + 1;
    if (const auto next_n = map_.get_north(current_n);
        latest_visit_time[next_n] >= next_t && edge_costs.north < std::numeric_limits<Cost>::infinity())
    {
        generate<has_resources, is_sipp>(current, next_n, next_t, edge_costs.north, w, waypoint_time);
    }
    if (const auto next_n = map_.get_south(current_n);
        latest_visit_time[next_n] >= next_t && edge_costs.south < std::numeric_limits<Cost>::infinity())
    {
        generate<has_resources, is_sipp>(current, next_n, next_t, edge_costs.south, w, waypoint_time);
    }
    if (const auto next_n = map_.get_east(current_n);
        latest_visit_time[next_n] >= next_t && edge_costs.east < std::numeric_limits<Cost>::infinity())
    {
        generate<has_resources, is_sipp>(current, next_n, next_t, edge_costs.east, w, waypoint_time);
    }
    if (const auto next_n = map_.get_west(current_n);
        latest_visit_time[next_n] >= next_t && edge_costs.west < std::numeric_limits<Cost>::infinity())
    {
        generate<has_resources, is_sipp>(current, next_n, next_t, edge_costs.west, w, waypoint_time);
    }
    if (const auto next_n = map_.get_wait(current_n);
        latest_visit_time[next_n] >= next_t && edge_costs.wait < std::numeric_limits<Cost>::infinity())
    {
        generate<has_resources, is_sipp>(current, next_n, next_t, edge_costs.wait, w, waypoint_time);
    }
}
template
void AStar::generate_neighbours<false, 0>(Label* const current, const Waypoint w, const Time waypoint_time);
template
void AStar::generate_neighbours<false, 1>(Label* const current, const Waypoint w, const Time waypoint_time);
template
void AStar::generate_neighbours<true, 0>(Label* const current, const Waypoint w, const Time waypoint_time);
template
void AStar::generate_neighbours<true, 1>(Label* const current, const Waypoint w, const Time waypoint_time);

template<bool has_resources, IntCost default_cost>
void AStar::generate_neighbours_last_segment(Label* const current)
{
    constexpr bool is_sipp = false;

    // Get data.
    auto& [start,
           waypoints,
           goal,
           earliest_goal_time,
           latest_goal_time,
           cost_offset,
           latest_visit_time,
           edge_penalties,
           finish_time_penalties
#ifdef USE_GOAL_CONFLICTS
         , goal_penalties
#endif
    ] = data_;

    // Print.
#ifdef DEBUG
    if (verbose)
    {
#ifdef USE_GOAL_CONFLICTS
        const auto nb_goal_penalties = goal_penalties.size();
#else
        const auto nb_goal_penalties = 0;
#endif
        println("Expanding label {} {} (n {}, t {}, nt {}, xy ({},{}), g {}, h {}, f {}{})",
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
                make_goal_state_string(&current->state_[0], nb_goal_penalties));
    }
#endif

    // Expand in five directions.
    const auto edge_costs = edge_penalties.get_edge_costs<default_cost>(current->nt);
    const auto current_n = current->n;
    const auto next_t = current->t + 1;
    if (const auto next_n = map_.get_north(current_n);
        latest_visit_time[next_n] >= next_t && edge_costs.north < std::numeric_limits<Cost>::infinity())
    {
        generate_last_segment<has_resources, is_sipp>(current, next_n, next_t, edge_costs.north);
    }
    if (const auto next_n = map_.get_south(current_n);
        latest_visit_time[next_n] >= next_t && edge_costs.south < std::numeric_limits<Cost>::infinity())
    {
        generate_last_segment<has_resources, is_sipp>(current, next_n, next_t, edge_costs.south);
    }
    if (const auto next_n = map_.get_east(current_n);
        latest_visit_time[next_n] >= next_t && edge_costs.east < std::numeric_limits<Cost>::infinity())
    {
        generate_last_segment<has_resources, is_sipp>(current, next_n, next_t, edge_costs.east);
    }
    if (const auto next_n = map_.get_west(current_n);
        latest_visit_time[next_n] >= next_t && edge_costs.west < std::numeric_limits<Cost>::infinity())
    {
        generate_last_segment<has_resources, is_sipp>(current, next_n, next_t, edge_costs.west);
    }
    if (const auto next_n = map_.get_wait(current_n);
        latest_visit_time[next_n] >= next_t && edge_costs.wait < std::numeric_limits<Cost>::infinity())
    {
        generate_last_segment<has_resources, is_sipp>(current, next_n, next_t, edge_costs.wait);
    }
}
template
void AStar::generate_neighbours_last_segment<false, 0>(Label* const current);
template
void AStar::generate_neighbours_last_segment<false, 1>(Label* const current);
template
void AStar::generate_neighbours_last_segment<true, 0>(Label* const current);
template
void AStar::generate_neighbours_last_segment<true, 1>(Label* const current);

template<bool has_resources, IntCost default_cost>
void AStar::generate_neighbours_sipp(Label* const current, const Waypoint w, const Time waypoint_time)
{
    constexpr bool is_sipp = true;

    // Get data.
    auto& [start,
           waypoints,
           goal,
           earliest_goal_time,
           latest_goal_time,
           cost_offset,
           latest_visit_time,
           edge_penalties,
           finish_time_penalties
#ifdef USE_GOAL_CONFLICTS
         , goal_penalties
#endif
    ] = data_;

    // Get constant.
    constexpr auto inf_cost = std::numeric_limits<Cost>::infinity();
    constexpr auto max_time = std::numeric_limits<Time>::max();

    // Print.
#ifdef DEBUG
    if (verbose)
    {
#ifdef USE_GOAL_CONFLICTS
        const auto nb_goal_penalties = goal_penalties.size();
#else
        const auto nb_goal_penalties = 0;
#endif
        println("Expanding label {} {} (n {}, t {}, nt {}, xy ({},{}), g {}, h {}, f {}{})",
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
                make_goal_state_string(&current->state_[0], nb_goal_penalties));
    }
#endif

    // Get current label.
    const auto n = current->n;
    const auto t = current->t;

    // Expand to after the next wait at the current node.
    Time wait_start = max_time;
    Time wait_end = max_time;
    Cost wait_penalty = 0;
    for (const auto& interval : sipp_intervals_[n][Direction::WAIT])
        if (interval.end > t)
        {
            // Store the wait interval.
            wait_start = std::max(interval.start, t);
            wait_end = interval.end;
            wait_penalty = interval.penalty;

            // Expand in the wait direction.
            const auto next_n = n;
            const auto next_t = wait_end;
            const Cost cost = (default_cost)                * std::max(wait_start - t, 0) +
                              (default_cost + wait_penalty) * (wait_end - wait_start);
            debug_assert(next_t == t + std::max(wait_start - t, 0) + (wait_end - wait_start));
            if (cost < inf_cost && latest_visit_time[n] >= next_t - 1 && latest_visit_time[next_n] >= next_t)
            {
                generate<has_resources, is_sipp>(current, next_n, next_t, cost, w, waypoint_time);
            }

            // Only expand to the first of the upcoming wait intervals.
            break;
        }

    // Expand in the move directions.
    for (Int d = 0; d < 4; ++d)
    {
        // Get the destination node.
        const auto next_n = map_.get_destination(Edge{n, static_cast<Direction>(d)});

        // Get the outgoing intervals at the current node and the wait intervals at the destination.
        const auto& intervals_nd = sipp_intervals_[n][d];
        const auto& dest_wait_intervals = sipp_intervals_[next_n][Direction::WAIT];

        // Expand departing at the first timestep of the outgoing intervals and expand arriving after the wait intervals
        // at the destination.
        debug_assert(!intervals_nd.empty());
        auto interval = intervals_nd.begin();
        for (; interval != intervals_nd.end() && interval->end <= t; ++interval);
        debug_assert(interval != intervals_nd.end() && interval->end > t);
        auto wait_interval = dest_wait_intervals.begin();
        for (; wait_interval != dest_wait_intervals.end() && wait_interval->end <= t + 1; ++wait_interval);
        debug_assert(wait_interval == dest_wait_intervals.end() || wait_interval->end > t + 1);
        for (; interval != intervals_nd.end() && interval->start < wait_end; ++interval)
        {
            // Move to the destination at the start of the interval.
            {
                const auto next_t = std::max(interval->start, t) + 1;
                Cost cost;
                if (interval->start <= wait_start)
                {
                    cost = (default_cost)                     * std::max(interval->start - t, 0) +
                           (default_cost + interval->penalty) * 1;
                    debug_assert(next_t == t + std::max(interval->start - t, 0) + 1);
                }
                else
                {
                    cost = (default_cost)                     * std::max(wait_start - t, 0) +
                           (default_cost + wait_penalty)      * (interval->start - wait_start) +
                           (default_cost + interval->penalty) * 1;
                    debug_assert(next_t == t + std::max(wait_start - t, 0) + (interval->start - wait_start) + 1);
                }
                if (cost < inf_cost && latest_visit_time[n] >= next_t - 1 && latest_visit_time[next_n] >= next_t)
                {
                    generate<has_resources, is_sipp>(current, next_n, next_t, cost, w, waypoint_time);
                }
            }

            // Move to the destination (within the interval) to arrive after a wait interval at the destination.
            for (const auto latest_wait_end = std::min(interval->end, wait_end);
                 wait_interval != dest_wait_intervals.end() && wait_interval->end <= latest_wait_end;
                 ++wait_interval)
            {
                const auto depart = wait_interval->end - 1;
                debug_assert(depart < interval->end);
                if (depart > interval->start)
                {
                    const auto next_t = wait_interval->end;
                    Cost cost;
                    if (depart <= wait_start)
                    {
                        cost = (default_cost)                     * std::max(depart - t, 0) +
                               (default_cost + interval->penalty) * 1;
                        debug_assert(next_t == t + std::max(depart - t, 0) + 1);
                    }
                    else
                    {
                        cost = (default_cost)                     * std::max(wait_start - t, 0) +
                               (default_cost + wait_penalty)      * (depart - wait_start) +
                               (default_cost + interval->penalty) * 1;
                        debug_assert(next_t == t + std::max(wait_start - t, 0) + (depart - wait_start) + 1);
                    }
                    if (cost < inf_cost && latest_visit_time[n] >= next_t - 1 && latest_visit_time[next_n] >= next_t)
                    {
                        generate<has_resources, is_sipp>(current, next_n, next_t, cost, w, waypoint_time);
                    }
                }
            }
        }
    }
}
template
void AStar::generate_neighbours_sipp<false, 0>(Label* const current, const Waypoint w, const Time waypoint_time);
template
void AStar::generate_neighbours_sipp<false, 1>(Label* const current, const Waypoint w, const Time waypoint_time);
template
void AStar::generate_neighbours_sipp<true, 0>(Label* const current, const Waypoint w, const Time waypoint_time);
template
void AStar::generate_neighbours_sipp<true, 1>(Label* const current, const Waypoint w, const Time waypoint_time);

template<bool has_resources, IntCost default_cost>
void AStar::generate_neighbours_last_segment_sipp(Label* const current)
{
    constexpr bool is_sipp = true;

    // Get data.
    auto& [start,
           waypoints,
           goal,
           earliest_goal_time,
           latest_goal_time,
           cost_offset,
           latest_visit_time,
           edge_penalties,
           finish_time_penalties
#ifdef USE_GOAL_CONFLICTS
         , goal_penalties
#endif
    ] = data_;

    // Get constant.
    constexpr auto inf_cost = std::numeric_limits<Cost>::infinity();
    constexpr auto max_time = std::numeric_limits<Time>::max();

    // Print.
#ifdef DEBUG
    if (verbose)
    {
#ifdef USE_GOAL_CONFLICTS
        const auto nb_goal_penalties = goal_penalties.size();
#else
        const auto nb_goal_penalties = 0;
#endif
        println("Expanding label {} {} (n {}, t {}, nt {}, xy ({},{}), g {}, h {}, f {}{})",
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
                make_goal_state_string(&current->state_[0], nb_goal_penalties));
    }
#endif

    // Get current label.
    const auto n = current->n;
    const auto t = current->t;

    // Expand to after the next wait at the current node.
    Time wait_start = max_time;
    Time wait_end = max_time;
    Cost wait_penalty = 0;
    for (const auto& interval : sipp_intervals_[n][Direction::WAIT])
        if (interval.end > t)
        {
            // Store the wait interval.
            wait_start = std::max(interval.start, t);
            wait_end = interval.end;
            wait_penalty = interval.penalty;

            // Expand in the wait direction.
            const auto next_n = n;
            const auto next_t = wait_end;
            const Cost cost = (default_cost)                * std::max(wait_start - t, 0) +
                              (default_cost + wait_penalty) * (wait_end - wait_start);
            debug_assert(next_t == t + std::max(wait_start - t, 0) + (wait_end - wait_start));
            if (cost < inf_cost && latest_visit_time[n] >= next_t - 1 && latest_visit_time[next_n] >= next_t)
            {
                generate_last_segment<has_resources, is_sipp>(current, next_n, next_t, cost);
            }

            // Only expand to the first of the upcoming wait intervals.
            break;
        }

    // Expand in the move directions.
    for (Int d = 0; d < 4; ++d)
    {
        // Get the destination node.
        const auto next_n = map_.get_destination(Edge{n, static_cast<Direction>(d)});

        // Get the outgoing intervals at the current node and the wait intervals at the destination.
        const auto& intervals_nd = sipp_intervals_[n][d];
        const auto& dest_wait_intervals = sipp_intervals_[next_n][Direction::WAIT];

        // Expand departing at the first timestep of the outgoing intervals and expand arriving after the wait intervals
        // at the destination.
        debug_assert(!intervals_nd.empty());
        auto interval = intervals_nd.begin();
        for (; interval != intervals_nd.end() && interval->end <= t; ++interval);
        debug_assert(interval != intervals_nd.end() && interval->end > t);
        auto wait_interval = dest_wait_intervals.begin();
        for (; wait_interval != dest_wait_intervals.end() && wait_interval->end <= t + 1; ++wait_interval);
        debug_assert(wait_interval == dest_wait_intervals.end() || wait_interval->end > t + 1);
        for (; interval != intervals_nd.end() && interval->start < wait_end; ++interval)
        {
            // Move to the destination at the start of the interval.
            {
                const auto next_t = std::max(interval->start, t) + 1;
                Cost cost;
                if (interval->start <= wait_start)
                {
                    cost = (default_cost)                     * std::max(interval->start - t, 0) +
                           (default_cost + interval->penalty) * 1;
                    debug_assert(next_t == t + std::max(interval->start - t, 0) + 1);
                }
                else
                {
                    cost = (default_cost)                     * std::max(wait_start - t, 0) +
                           (default_cost + wait_penalty)      * (interval->start - wait_start) +
                           (default_cost + interval->penalty) * 1;
                    debug_assert(next_t == t + std::max(wait_start - t, 0) + (interval->start - wait_start) + 1);
                }
                if (cost < inf_cost && latest_visit_time[n] >= next_t - 1 && latest_visit_time[next_n] >= next_t)
                {
                    generate_last_segment<has_resources, is_sipp>(current, next_n, next_t, cost);
                }
            }

            // Move to the destination (within the interval) to arrive after a wait interval at the destination.
            for (const auto latest_wait_end = std::min(interval->end, wait_end);
                 wait_interval != dest_wait_intervals.end() && wait_interval->end <= latest_wait_end;
                 ++wait_interval)
            {
                const auto depart = wait_interval->end - 1;
                debug_assert(depart < interval->end);
                if (depart > interval->start)
                {
                    const auto next_t = wait_interval->end;
                    Cost cost;
                    if (depart <= wait_start)
                    {
                        cost = (default_cost)                     * std::max(depart - t, 0) +
                               (default_cost + interval->penalty) * 1;
                        debug_assert(next_t == t + std::max(depart - t, 0) + 1);
                    }
                    else
                    {
                        cost = (default_cost)                     * std::max(wait_start - t, 0) +
                               (default_cost + wait_penalty)      * (depart - wait_start) +
                               (default_cost + interval->penalty) * 1;
                        debug_assert(next_t == t + std::max(wait_start - t, 0) + (depart - wait_start) + 1);
                    }
                    if (cost < inf_cost && latest_visit_time[n] >= next_t - 1 && latest_visit_time[next_n] >= next_t)
                    {
                        generate_last_segment<has_resources, is_sipp>(current, next_n, next_t, cost);
                    }
                }
            }
        }
    }
}
template
void AStar::generate_neighbours_last_segment_sipp<false, 0>(Label* const current);
template
void AStar::generate_neighbours_last_segment_sipp<false, 1>(Label* const current);
template
void AStar::generate_neighbours_last_segment_sipp<true, 0>(Label* const current);
template
void AStar::generate_neighbours_last_segment_sipp<true, 1>(Label* const current);

void AStar::generate_end(Label* const current)
{
    // Get data.
    const auto& [start,
                 waypoints,
                 goal,
                 earliest_goal_time,
                 latest_goal_time,
                 cost_offset,
                 latest_visit_time,
                 edge_penalties,
                 finish_time_penalties
#ifdef USE_GOAL_CONFLICTS
               , goal_penalties
#endif
    ] = data_;

    // Create label.
    auto new_label = reinterpret_cast<Label*>(label_pool_.get_label_buffer());
    memcpy(new_label, current, label_pool_.label_size());
#ifdef DEBUG
    new_label->label_id = nb_labels_++;
#endif
    new_label->parent = current;
    debug_assert((*h_node_to_waypoint_)[current->n] == 0);
    new_label->g += finish_time_penalties.get_penalty(current->t);
    new_label->f = new_label->g;
    new_label->n = -1;

    // Check if cost-infeasible.
    if (isGE(new_label->f, 0))
    {
        // Print.
#ifdef DEBUG
        if (verbose)
        {
#ifdef USE_GOAL_CONFLICTS
            const auto nb_goal_penalties = goal_penalties.size();
#else
            const auto nb_goal_penalties = 0;
#endif
            println("    Cost-infeasible end label {} {} (t {}, g {}{})",
                    new_label->label_id,
                    fmt::ptr(new_label),
                    decltype(new_label->t){new_label->t},
                    new_label->g,
                    make_goal_state_string(&new_label->state_[0], nb_goal_penalties));
        }
#endif

        // Done.
        return;
    }

    // Store the label.
    label_pool_.commit_latest_label();
    open_.push(new_label);

    // Print.
#ifdef DEBUG
    if (verbose)
    {
#ifdef USE_GOAL_CONFLICTS
        const auto nb_goal_penalties = goal_penalties.size();
#else
        const auto nb_goal_penalties = 0;
#endif
        println("    Generating end label {} {} (t {}, g {}{})",
                new_label->label_id,
                fmt::ptr(new_label),
                decltype(new_label->t){new_label->t},
                new_label->g,
                make_goal_state_string(&new_label->state_[0], nb_goal_penalties));
    }
#endif
}

template<>
AStar::Label* AStar::dominated<false>(Label* const new_label)
{
    // Check.
//#ifdef DEBUG
//    for (auto it = frontier_without_resources_.begin(); it != frontier_without_resources_.end(); ++it)
//    {
//        auto label = it->second;
//        open_.check_label(label);
//    }
//#endif

    // Check.
    debug_assert(data_.goal_penalties.empty());

    // Try to put in the new label.
    auto [it, success] = frontier_without_resources_.try_emplace(NodeTime{new_label->nt}, new_label);

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

            // Replace the existing label with the new label.
            release_assert(existing_label->pqueue_index >= 0,
                           "New label replacing an existing label that is not in the priority queue");
            // if (existing_label->pqueue_index >= 0)
            {
                open_.update_pqueue_index(new_label, existing_label->pqueue_index);
                memcpy(existing_label, new_label, label_pool_.label_size());
                open_.decrease_key(existing_label);
            }
            // else
            // {
            //     debug_assert(existing_label->pqueue_index == -1);
            //     memcpy(existing_label, new_label, label_pool_.label_size());
            //     open_.push(existing_label);
            // }
            return existing_label;
        }
    }
    else
    {
        // Store the label.
        label_pool_.commit_latest_label();
        open_.push(new_label);

        // Not dominated.
        return new_label;
    }
}

template<>
AStar::Label* AStar::dominated<true>(Label* const new_label)
{
#ifdef USE_GOAL_CONFLICTS
    // Get goal crossings.
    const auto& goal_penalties = data_.goal_penalties;
    const auto nb_goal_penalties = goal_penalties.size();
    // debug_assert(nb_goal_penalties > 0);

    // Print.
    // println("    Checking dominance for new label {} {} (n {}, t {}, nt {}, xy ({},{}), g {}, h {}, f {}{})",
    //         new_label->label_id,
    //         fmt::ptr(new_label),
    //         decltype(new_label->n){new_label->n},
    //         decltype(new_label->t){new_label->t},
    //         decltype(new_label->nt){new_label->nt},
    //         map_.get_x(new_label->n),
    //         map_.get_y(new_label->n),
    //         new_label->g,
    //         new_label->f - new_label->g,
    //         new_label->f,
    //         make_goal_state_string(&new_label->state_[0], nb_goal_penalties));

    // Check dominance.
    auto& existing_labels = frontier_with_resources_[new_label->nt];
    Label* store_in_existing_label = nullptr;
#ifdef DEBUG
    bool dominates = false;
#endif
    debug_assert(nb_goal_penalties > 0 || existing_labels.size() <= 1);
    for (size_t idx = 0; idx < existing_labels.size();)
    {
        // Check if the new label is dominated by the existing label.
        auto& existing_label = existing_labels[idx];
        {
            // Calculate the maximum cost of the existing label if it incurred the same penalties as the new label.
            auto existing_label_potential_cost = existing_label->f;
            for (Int idx = 0; idx < nb_goal_penalties; ++idx)
                if (get_bitset(new_label->state_, idx) > get_bitset(existing_label->state_, idx))
                {
                    existing_label_potential_cost += goal_penalties[idx].cost;
                }
            // If the existing label still costs less than or equal to the new label, even after incurring these
            // penalties, then the new label is dominated.
            if (isLE(existing_label_potential_cost, new_label->f))
            {
                debug_assert(!dominates);
                return nullptr;
            }
        }

        // Check if the existing label is dominated by the new label.
        {
            auto new_label_potential_cost = new_label->f;
            for (Int idx = 0; idx < nb_goal_penalties; ++idx)
                if (get_bitset(existing_label->state_, idx) > get_bitset(new_label->state_, idx))
                {
                    new_label_potential_cost += goal_penalties[idx].cost;
                }
            if (isLE(new_label_potential_cost, existing_label->f))
            {
                // If the existing label is not yet expanded, use its memory to store the new label.
                debug_assert(nb_goal_penalties > 0 || existing_label->pqueue_index >= 0);
                if (existing_label->pqueue_index >= 0)
                {
                    if (store_in_existing_label)
                    {
                        debug_assert(store_in_existing_label->pqueue_index >= 0);
                        open_.erase(store_in_existing_label->pqueue_index);
                    }
                    store_in_existing_label = existing_label;
                }

                // Delete the existing label from future dominance checks.
                existing_label = existing_labels.back();
                existing_labels.pop_back(); // pop_back() invalidates end()-1 iterator so must use pointer in loop
#ifdef DEBUG
                dominates = true;
#endif
            }
            else
            {
                ++idx;
            }
        }
    }

    // The new label is not dominated. Store the label.
    if (store_in_existing_label)
    {
        // Replace the existing label with the new label.
        debug_assert(isLE(new_label->f, store_in_existing_label->f));
        debug_assert(store_in_existing_label->pqueue_index >= 0);
        open_.update_pqueue_index(new_label, store_in_existing_label->pqueue_index);
        memcpy(store_in_existing_label, new_label, label_pool_.label_size());
        open_.decrease_key(store_in_existing_label);

        existing_labels.push_back(store_in_existing_label);
        return store_in_existing_label;
    }
    else
    {
        label_pool_.commit_latest_label();
        open_.push(new_label);
        debug_assert(new_label->pqueue_index >= 0);

        existing_labels.push_back(new_label);
        return new_label;
    }
#else
    err("Cannot call this dominance checking function without goal conflict constraints");
#endif
}

void AStar::preprocess_input()
{
    // Get data.
    auto& [start,
           waypoints,
           goal,
           earliest_goal_time,
           latest_goal_time,
           cost_offset,
           latest_visit_time,
           edge_penalties,
           finish_time_penalties
#ifdef USE_GOAL_CONFLICTS
         , goal_penalties
#endif
    ] = data_;

    // Append the goal as a waypoint.
    debug_assert(earliest_goal_time <= latest_goal_time);
    waypoints.push_back(NodeTime{goal, earliest_goal_time});
}

template<bool is_farkas>
Pair<Vector<NodeTime>, Cost> AStar::solve()
{
    constexpr bool is_sipp = false;

#ifdef USE_GOAL_CONFLICTS
    if (data_.goal_penalties.empty())
    {
        return solve<is_farkas, false, is_sipp>();
    }
    else
    {
        return solve<is_farkas, true, is_sipp>();
    }
#else
    return solve<is_farkas, false, is_sipp>();
#endif
}
template Pair<Vector<NodeTime>, Cost> AStar::solve<false>();
template Pair<Vector<NodeTime>, Cost> AStar::solve<true>();

template<bool is_farkas>
Pair<Vector<NodeTime>, Cost> AStar::solve_sipp()
{
    constexpr bool is_sipp = true;

#ifdef USE_GOAL_CONFLICTS
    if (data_.goal_penalties.empty())
    {
        return solve<is_farkas, false, is_sipp>();
    }
    else
    {
        return solve<is_farkas, true, is_sipp>();
    }
#else
    return solve<is_farkas, false, is_sipp>();
#endif
}
template Pair<Vector<NodeTime>, Cost> AStar::solve_sipp<false>();
template Pair<Vector<NodeTime>, Cost> AStar::solve_sipp<true>();

template<bool is_farkas, bool has_resources, bool is_sipp>
Pair<Vector<NodeTime>, Cost> AStar::solve()
{
    // Get data.
    const auto& [start,
                 waypoints,
                 goal,
                 earliest_goal_time,
                 latest_goal_time,
                 cost_offset,
                 latest_visit_time,
                 edge_penalties,
                 finish_time_penalties
#ifdef USE_GOAL_CONFLICTS
               , goal_penalties
#endif
    ] = data_;

    // Print.
#ifdef PRINT_DEBUG
    {
        String str;
        for (auto it = waypoints.begin(); it != waypoints.end() - 1; ++it)
        {
            if (it == waypoints.begin())
            {
                str += " via " ;
            }
            else
            {
                str += ", ";
            }
            str += fmt::format("(({},{}),{})", map_.get_x(it->n), map_.get_y(it->n), it->t);
        }
        println("Solving from ({},{}){} to ({},{}) between times {} and {}",
                map_.get_x(start),
                map_.get_y(start),
                str,
                map_.get_x(goal),
                map_.get_y(goal),
                earliest_goal_time,
                latest_goal_time);
    }
#endif

    // Create output.
    Pair<Vector<NodeTime>, Cost> output;
    auto& path = output.first;
    auto& path_cost = output.second;

    // Prepare costs.
//     data_.edge_penalties.before_solve();
//     data_.finish_time_penalties.before_solve();
// #ifdef USE_GOAL_CONFLICTS
//     data_.goal_penalties.before_solve();
// #endif

    // Get number of resources.
#ifdef USE_GOAL_CONFLICTS
    const auto nb_goal_crossings = goal_penalties.size();
#else
    constexpr Int nb_goal_crossings = 0;
#endif

    // Print goal crossings.
#ifdef PRINT_DEBUG
#ifdef USE_GOAL_CONFLICTS
    for (Int idx = 0; idx < nb_goal_crossings; ++idx)
    {
        const auto [goal_nt, goal_cost] = goal_penalties[idx];
        println("Goal crossing ({},{}) at or after time {} incurs {:.6f}",
                map_.get_x(goal_nt.n), map_.get_y(goal_nt.n), goal_nt.t, goal_cost);
    }
#endif
#endif

    // Reset.
    const auto nb_states = nb_goal_crossings;
    label_pool_.reset(sizeof(Label) + (nb_states + CHAR_BIT - 1) / CHAR_BIT);
    open_.clear();
    frontier_without_resources_.clear();
    frontier_with_resources_.clear();

    // Compute minimum time between each waypoint.
    h_waypoint_to_goal_.resize(waypoints.size());
    h_waypoint_to_goal_.back() = 0;
    for (Waypoint w = waypoints.size() - 2; w >= 0; --w)
    {
        const auto h = heuristic_.get_h(waypoints[w + 1].n)[waypoints[w].n];
        const auto t_diff = waypoints[w + 1].t - waypoints[w].t;
        if (w != static_cast<Waypoint>(waypoints.size() - 2) && t_diff < h)
        {
            return output;
        }
        h_waypoint_to_goal_[w] = std::max(h, t_diff) + h_waypoint_to_goal_[w + 1];
    }

    // Create the first label.
    Waypoint w = 0;
    h_node_to_waypoint_ = &heuristic_.get_h(waypoints[w].n);
    generate_start<has_resources>();

    // Create SIPP intervals.
    if constexpr (is_sipp)
    {
        // Clear previous intervals.
        for (Node n = 0; n < map_.size(); ++n)
            for (Int d = 0; d < 5; ++d)
                sipp_intervals_[n][d].clear();

        // Add intervals from edge costs.
        for (const auto& [nt, edge_penalty] : edge_penalties)
            for (Int d = 0; d < 5; ++d)
                if (edge_penalty.d[d] != 0)
                {
                    sipp_intervals_[nt.n][d].push_back({nt.t, nt.t + 1, edge_penalty.d[d]});
                }

        // Add extra intervals to correctly expand to the waypoints (which includes the goal).
        for (const auto nt : waypoints)
        {
            auto& wait_intervals = sipp_intervals_[nt.n][Direction::WAIT];
            if (const Time t = nt.t;
                t > 0 &&
                std::find_if(wait_intervals.begin(),
                             wait_intervals.end(),
                             [t](const SIPPInterval& interval) { return interval.end == t; }) ==
                wait_intervals.end())
            {
                wait_intervals.push_back({t - 1, t, 0});
            }
        }

        // Add extra intervals to correctly expand to the dummy end node.
        {
            auto& wait_intervals = sipp_intervals_[goal][Direction::WAIT];
            for (Time t = 1; t < static_cast<Time>(finish_time_penalties.size()); ++t)
                if (finish_time_penalties[t] != finish_time_penalties[t - 1] &&
                    std::find_if(wait_intervals.begin(),
                                 wait_intervals.end(),
                                 [t](const SIPPInterval& interval) { return interval.end == t; }) ==
                    wait_intervals.end())
                {
                    wait_intervals.push_back({t - 1, t, 0});
                }
            if (const Time t = finish_time_penalties.size();
                t > 0 &&
                std::find_if(wait_intervals.begin(),
                             wait_intervals.end(),
                             [t](const SIPPInterval& interval) { return interval.end == t; }) ==
                wait_intervals.end())
            {
                wait_intervals.push_back({t - 1, t, 0});
            }
        }

        // Process intervals at each node.
        for (Node n = 0; n < map_.size(); ++n)
        {
            // Process move intervals.
            auto& intervals_n = sipp_intervals_[n];
            for (Int d = 0; d < Direction::WAIT; ++d)
            {
                // Sort intervals.
                auto& intervals_nd = intervals_n[d];
                std::sort(intervals_nd.begin(),
                          intervals_nd.end(),
                          [](const SIPPInterval& a, const SIPPInterval& b) { return a.start < b.start; });

                // Merge consecutive intervals.
                for (Int idx = 0; idx < static_cast<Int>(intervals_nd.size()) - 1;)
                {
                    debug_assert(intervals_nd[idx].end <= intervals_nd[idx + 1].start);

                    if (intervals_nd[idx].end == intervals_nd[idx + 1].start &&
                        intervals_nd[idx].penalty == intervals_nd[idx + 1].penalty)
                    {
                        intervals_nd[idx].end = intervals_nd[idx + 1].end;
                        intervals_nd.erase(intervals_nd.begin() + idx + 1);
                    }
                    else
                    {
                        ++idx;
                    }
                }

                // Insert dummy intervals spanning the entire time horizon.
                if (intervals_nd.empty())
                {
                    intervals_nd.push_back({0, std::numeric_limits<Time>::max(), 0});
                }
                else
                {
                    // Insert an interval starting at the beginning of the planning period.
                    if (intervals_nd.front().start > 0)
                    {
                        intervals_nd.insert(intervals_nd.begin(), {0, intervals_nd.front().start, 0});
                    }

                    // Insert an interval end at the end of the planning period.
                    debug_assert(intervals_nd.back().end < std::numeric_limits<Time>::max());
                    intervals_nd.push_back({intervals_nd.back().end, std::numeric_limits<Time>::max(), 0});

                    // Insert dummy intervals between existing intervals.
                    for (Int idx = 1; idx < static_cast<Int>(intervals_nd.size()); ++idx)
                        if (intervals_nd[idx - 1].end != intervals_nd[idx].start)
                        {
                            intervals_nd.insert(intervals_nd.begin() + idx,
                                                {intervals_nd[idx - 1].end, intervals_nd[idx].start, 0});
                        }
                }

                // Print.
#ifdef PRINT_DEBUG
                if (intervals_nd.size() > 1)
                {
                    println("({},{}) ({},{}) {} intervals:",
                            map_.get_x(n),
                            map_.get_y(n),
                            map_.get_destination_xy(Edge{n, Direction(d)}).first,
                            map_.get_destination_xy(Edge{n, Direction(d)}).second,
                            static_cast<Direction>(d));
                    for (const auto& interval : intervals_nd)
                    {
                        println("    start {}, end {}, penalty {:.6f}", interval.start, interval.end, interval.penalty);
                    }
                }
#endif
            }

            // Process wait intervals.
            {
                // Sort intervals.
                constexpr auto d = Direction::WAIT;
                auto& intervals_nd = intervals_n[d];
                std::sort(intervals_nd.begin(),
                          intervals_nd.end(),
                          [](const SIPPInterval& a, const SIPPInterval& b) { return a.start < b.start; });

                // Check.
#ifdef DEBUG
                for (Int idx = 1; idx < static_cast<Int>(intervals_nd.size()); ++idx)
                {
                    debug_assert(intervals_nd[idx - 1].end <= intervals_nd[idx].start);
                }
#endif

                // Print.
#ifdef PRINT_DEBUG
                if (intervals_nd.size() > 1)
                {
                    println("({},{}) ({},{}) {} intervals:",
                            map_.get_x(n),
                            map_.get_y(n),
                            map_.get_destination_xy(Edge{n, Direction(d)}).first,
                            map_.get_destination_xy(Edge{n, Direction(d)}).second,
                            static_cast<Direction>(d));
                    for (const auto& interval : intervals_nd)
                    {
                        println("    start {}, end {}, penalty {:.6f}", interval.start, interval.end, interval.penalty);
                    }
                }
#endif
            }
        }
    }

    // Solve up to but not including the last waypoint (goal).
    constexpr IntCost default_cost = is_farkas ? 0 : 1;
    if (waypoints.size() > 1)
    {
        while (!open_.empty())
        {
            // Get a label from priority queue.
            const auto current = open_.top();
            open_.pop();

            // Advance to the next waypoint.
            debug_assert(current->t <= waypoints[w].t);
            if (current->nt == waypoints[w])
            {
                // Print.
#ifdef DEBUG
                if (verbose)
                {
                    // Store the path.
                    Vector<NodeTime> path;
                    for (auto l = current; l; l = l->parent)
                    {
                        path.push_back(l->nt);
                    }
                    std::reverse(path.begin(), path.end());

                    // Print.
#ifdef USE_GOAL_CONFLICTS
                    const auto nb_goal_penalties = goal_penalties.size();
#else
                    const auto nb_goal_penalties = 0;
#endif
                    fmt::print("Reached waypoint at label {} {} (n {}, t {}, nt {}, xy ({},{}), g {}, h {}, f {}{}) "
                               "with path",
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
                               make_goal_state_string(&current->state_[0], nb_goal_penalties));
                    for (const auto nt: path)
                    {
                        fmt::print(" ({},{})", map_.get_x(nt.n), map_.get_y(nt.n));
                    }
                    println("");
                }
#endif

                // Advance to the next waypoint.
                ++w;
                h_node_to_waypoint_ = &heuristic_.get_h(waypoints[w].n);

                // Clear priority queue.
                open_.clear();

                // Stop if reached the last waypoint (goal).
                if (w == static_cast<Waypoint>(waypoints.size() - 1))
                {
                    open_.push(current);
                    break;
                }
            }

            // Generate neighbours.
            if constexpr (is_sipp)
            {
                generate_neighbours_sipp<has_resources, default_cost>(current, w, waypoints[w].t);
            }
            else
            {
                generate_neighbours<has_resources, default_cost>(current, w, waypoints[w].t);
            }
        }
    }

    // Solve the last segment.
    while (!open_.empty())
    {
        // Get a label from priority queue.
        const auto current = open_.top();
        open_.pop();

        // Expand the neighbours of the current label or exit if the goal is reached.
        debug_assert(current->t <= latest_goal_time);
        if (current->n >= 0)
        {
            // Generate neighbours.
            if constexpr (is_sipp)
            {
                generate_neighbours_last_segment_sipp<has_resources, default_cost>(current);
            }
            else
            {
                generate_neighbours_last_segment<has_resources, default_cost>(current);
            }

            // Generate to the end.
            if (current->n == goal && current->t >= earliest_goal_time)
            {
                generate_end(current);
            }
        }
        else
        {
            // Store the path cost.
            path_cost = current->g;

            // Store the path.
            debug_assert(path.empty());
            if constexpr (is_sipp)
            {
                auto prev = NodeTime{current->parent->nt};
                for (auto l = current->parent; l; l = l->parent)
                {
                    for (Time t = prev.t; t > l->t + 1;)
                    {
                        --t;
                        path.push_back(NodeTime{l->n, t});
                    }
                    path.push_back(l->nt);
                    prev = NodeTime{l->nt};
                }
            }
            else
            {
                for (auto l = current->parent; l; l = l->parent)
                {
                    path.push_back(l->nt);
                }
            }
            std::reverse(path.begin(), path.end());

            // Check.
#ifdef DEBUG
            for (auto l = current->parent; l; l = l->parent)
            {
                debug_assert(path[l->t].nt == l->nt);
            }
            for (Time t = 0; t < static_cast<Time>(path.size()); ++t)
            {
                debug_assert(path[t].t == t);
            }
            for (Time t = 0; t < static_cast<Time>(path.size()) - 1; ++t)
            {
                const auto [x1, y1] = map_.get_xy(path[t].n);
                const auto [x2, y2] = map_.get_xy(path[t + 1].n);
                debug_assert(std::abs(x2 - x1) + std::abs(y2 - y2) <= 1);
            }
#endif

            // Print.
#ifdef DEBUG
            if (verbose)
            {
                println("Reached end at label {} {} (t {}, g {}{})",
                        current->label_id,
                        fmt::ptr(current),
                        decltype(current->t){current->t},
                        current->g,
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
            debug_assert(isLT(path_cost, 0));
            debug_assert(earliest_goal_time <= current->t && current->t <= latest_goal_time);

            // Finish.
            break;
        }
    }

    // Done.
#ifdef DEBUG
    if (verbose)
    {
        println("=======================================");
    }
#endif
    return output;
}

// TODO: move back into solve()
void AStar::before_solve()
{
    // Prepare costs.
    data_.edge_penalties.before_solve();
    data_.finish_time_penalties.before_solve();
#ifdef USE_GOAL_CONFLICTS
    data_.goal_penalties.before_solve();
#endif
}

#ifdef DEBUG
Pair<Vector<NodeTime>, Cost> AStar::calculate_cost(const Vector<Node>& input_path)
{
    static Int iter = 0;
#ifdef DEBUG
    if (verbose)
    {
        println("=======================================");
        println("START DEBUG ITER {}", iter);
    }
#endif
    iter++;

    constexpr bool has_resources = true;
    constexpr bool is_farkas = false;
    constexpr bool is_sipp = false;

    // ------------------------------------

    // Get data.
    auto& [start,
           waypoints,
           goal,
           earliest_goal_time,
           latest_goal_time,
           cost_offset,
           latest_visit_time,
           edge_penalties,
           finish_time_penalties
#ifdef USE_GOAL_CONFLICTS
         , goal_penalties
#endif
    ] = data_;

    // Print.
#ifdef PRINT_DEBUG
    {
        String str;
        for (auto it = waypoints.begin(); it != waypoints.end() - 1; ++it)
        {
            if (it == waypoints.begin())
            {
                str += " via " ;
            }
            else
            {
                str += ", ";
            }
            str += fmt::format("(({},{}),{})", map_.get_x(it->n), map_.get_y(it->n), it->t);
        }
        println("Solving from ({},{}){} to ({},{}) between times {} and {}",
                map_.get_x(start),
                map_.get_y(start),
                str,
                map_.get_x(goal),
                map_.get_y(goal),
                earliest_goal_time,
                latest_goal_time);
    }
#endif

    // Create output.
    Pair<Vector<NodeTime>, Cost> output;
    auto& path = output.first;
    auto& path_cost = output.second;

    // Prepare costs.
    data_.edge_penalties.before_solve();
    data_.finish_time_penalties.before_solve();
#ifdef USE_GOAL_CONFLICTS
    data_.goal_penalties.before_solve();
#endif

    // Get number of resources.
#ifdef USE_GOAL_CONFLICTS
    const auto nb_goal_crossings = goal_penalties.size();
#else
    constexpr Int nb_goal_crossings = 0;
#endif

    // Reset.
    const auto nb_states = nb_goal_crossings;
    label_pool_.reset(sizeof(Label) + (nb_states + CHAR_BIT - 1) / CHAR_BIT);
    open_.clear();
    frontier_without_resources_.clear();
    frontier_with_resources_.clear();

    // Compute minimum time between each waypoint.
    h_waypoint_to_goal_.resize(waypoints.size());
    h_waypoint_to_goal_.back() = 0;
    for (Waypoint w = waypoints.size() - 2; w >= 0; --w)
    {
        const auto h = heuristic_.get_h(waypoints[w + 1].n)[waypoints[w].n];
        const auto t_diff = waypoints[w + 1].t - waypoints[w].t;
        if (w != static_cast<Waypoint>(waypoints.size() - 2) && t_diff < h)
        {
            return output;
        }
        h_waypoint_to_goal_[w] = std::max(h, t_diff) + h_waypoint_to_goal_[w + 1];
    }

    // Create the first label.
    Waypoint w = 0;
    h_node_to_waypoint_ = &heuristic_.get_h(waypoints[w].n);
    generate_start<has_resources>();

    // Solve up to but not including the last waypoint (goal).
    Int idx = 1;
    constexpr IntCost default_cost = is_farkas ? 0 : 1;
    if (waypoints.size() > 1)
    {
        while (!open_.empty())
        {
            // Get a label from priority queue.
            const auto current = open_.top();
            open_.pop();

            // Advance to the next waypoint.
            debug_assert(current->t <= waypoints[w].t);
            if (current->nt == waypoints[w])
            {
                // Print.
#ifdef DEBUG
                if (verbose)
                {
                    // Store the path.
                    Vector<NodeTime> path;
                    for (auto l = current; l; l = l->parent)
                    {
                        path.push_back(l->nt);
                    }
                    std::reverse(path.begin(), path.end());

                    // Print.
#ifdef USE_GOAL_CONFLICTS
                    const auto nb_goal_penalties = goal_penalties.size();
#else
                    const auto nb_goal_penalties = 0;
#endif
                    fmt::print("Reached waypoint at label {} {} (n {}, t {}, nt {}, xy ({},{}), g {}, h {}, f {}{}) "
                               "with path",
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
                               make_goal_state_string(&current->state_[0], nb_goal_penalties));
                    for (const auto nt: path)
                    {
                        fmt::print(" ({},{})", map_.get_x(nt.n), map_.get_y(nt.n));
                    }
                    println("");
                }
#endif

                // Advance to the next waypoint.
                ++w;
                h_node_to_waypoint_ = &heuristic_.get_h(waypoints[w].n);

                // Clear priority queue.
                open_.clear();

                // Stop if reached the last waypoint (goal).
                if (w == static_cast<Waypoint>(waypoints.size() - 1))
                {
                    open_.push(current);
                    break;
                }
            }

            const auto waypoint_time = waypoints[w].t;

            // Expand in five directions.
            const auto edge_costs = edge_penalties.get_edge_costs<default_cost>(current->nt);
            const auto current_n = current->n;
            const auto next_t = current->t + 1;
            if (const auto next_n = map_.get_north(current_n);
                idx < static_cast<Int>(input_path.size()) && next_n == input_path[idx] &&
                latest_visit_time[next_n] >= next_t && edge_costs.north < std::numeric_limits<Cost>::infinity())
            {
                generate<has_resources, is_sipp>(current, next_n, next_t, edge_costs.north, w, waypoint_time);
            }
            if (const auto next_n = map_.get_south(current_n);
                idx < static_cast<Int>(input_path.size()) && next_n == input_path[idx] &&
                latest_visit_time[next_n] >= next_t && edge_costs.south < std::numeric_limits<Cost>::infinity())
            {
                generate<has_resources, is_sipp>(current, next_n, next_t, edge_costs.south, w, waypoint_time);
            }
            if (const auto next_n = map_.get_east(current_n);
                idx < static_cast<Int>(input_path.size()) && next_n == input_path[idx] &&
                latest_visit_time[next_n] >= next_t && edge_costs.east < std::numeric_limits<Cost>::infinity())
            {
                generate<has_resources, is_sipp>(current, next_n, next_t, edge_costs.east, w, waypoint_time);
            }
            if (const auto next_n = map_.get_west(current_n);
                idx < static_cast<Int>(input_path.size()) && next_n == input_path[idx] &&
                latest_visit_time[next_n] >= next_t && edge_costs.west < std::numeric_limits<Cost>::infinity())
            {
                generate<has_resources, is_sipp>(current, next_n, next_t, edge_costs.west, w, waypoint_time);
            }
            if (const auto next_n = map_.get_wait(current_n);
                idx < static_cast<Int>(input_path.size()) && next_n == input_path[idx] &&
                latest_visit_time[next_n] >= next_t && edge_costs.wait < std::numeric_limits<Cost>::infinity())
            {
                generate<has_resources, is_sipp>(current, next_n, next_t, edge_costs.wait, w, waypoint_time);
            }

            // Advance to the next node.
            ++idx;
        }
    }

    // Solve the last segment.
    while (!open_.empty())
    {
        // Get a label from priority queue.
        const auto current = open_.top();
        open_.pop();

        // Expand the neighbours of the current label or exit if the goal is reached.
        debug_assert(current->t <= latest_goal_time);
        if (current->n >= 0)
        {
            // Expand in five directions.
            const auto edge_costs = edge_penalties.get_edge_costs<default_cost>(current->nt);
            const auto current_n = current->n;
            const auto next_t = current->t + 1;
            if (const auto next_n = map_.get_north(current_n);
                idx < static_cast<Int>(input_path.size()) && next_n == input_path[idx] &&
                latest_visit_time[next_n] >= current->t + 1 && edge_costs.north < std::numeric_limits<Cost>::infinity())
            {
                generate_last_segment<has_resources, is_sipp>(current, next_n, next_t, edge_costs.north);
            }
            if (const auto next_n = map_.get_south(current_n);
                idx < static_cast<Int>(input_path.size()) && next_n == input_path[idx] &&
                latest_visit_time[next_n] >= current->t + 1 && edge_costs.south < std::numeric_limits<Cost>::infinity())
            {
                generate_last_segment<has_resources, is_sipp>(current, next_n, next_t, edge_costs.south);
            }
            if (const auto next_n = map_.get_east(current_n);
                idx < static_cast<Int>(input_path.size()) && next_n == input_path[idx] &&
                latest_visit_time[next_n] >= current->t + 1 && edge_costs.east < std::numeric_limits<Cost>::infinity())
            {
                generate_last_segment<has_resources, is_sipp>(current, next_n, next_t, edge_costs.east);
            }
            if (const auto next_n = map_.get_west(current_n);
                idx < static_cast<Int>(input_path.size()) && next_n == input_path[idx] &&
                latest_visit_time[next_n] >= current->t + 1 && edge_costs.west < std::numeric_limits<Cost>::infinity())
            {
                generate_last_segment<has_resources, is_sipp>(current, next_n, next_t, edge_costs.west);
            }
            if (const auto next_n = map_.get_wait(current_n);
                idx < static_cast<Int>(input_path.size()) && next_n == input_path[idx] &&
                latest_visit_time[next_n] >= current->t + 1 && edge_costs.wait < std::numeric_limits<Cost>::infinity())
            {
                generate_last_segment<has_resources, is_sipp>(current, next_n, next_t, edge_costs.wait);
            }

            // Generate to the end.
            if (current->n == goal && current->t >= earliest_goal_time)
            {
                generate_end(current);
            }
        }
        else
        {
            // Store the path cost.
            path_cost = current->g;

            // Store the path.
            debug_assert(path.empty());
            for (auto l = current->parent; l; l = l->parent)
            {
                path.push_back(l->nt);
            }
            std::reverse(path.begin(), path.end());

            // Print.
#ifdef DEBUG
            if (verbose)
            {
                println("Reached end at label {} {} (t {}, g {}{})",
                        current->label_id,
                        fmt::ptr(current),
                        decltype(current->t){current->t},
                        current->g,
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
            debug_assert(isLT(path_cost, 0));
            debug_assert(earliest_goal_time <= current->t && current->t <= latest_goal_time);

            // Finish.
            break;
        }

       // Advance to the next node.
       ++idx;
    }

    // Done.
#ifdef DEBUG
    if (verbose)
    {
        println("=======================================");
    }
#endif
    return output;
}

void AStar::set_verbose(const bool on)
{
    verbose = on;
}
#endif

}
