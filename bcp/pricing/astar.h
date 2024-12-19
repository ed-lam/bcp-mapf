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

#include "boost/container/small_vector.hpp"
#include "pricing/distance_heuristic.h"
#include "pricing/edgetime_penalties.h"
#include "pricing/finish_time_penalties.h"
#include "pricing/node_crossing_penalties.h"
#include "pricing/reservation_table.h"
#include "problem/includes.h"
#include "problem/map.h"
#include "types/map_types.h"
#include "types/memory_pool.h"
#include "types/priority_queue.h"

template <class T, Size N>
using SmallVector = boost::container::small_vector<T, N>;

class AStar
{
    // Label for main low-level search
    using PriorityQueueSizeType = Int32;
    struct Label
    {
#ifdef DEBUG
        size_t label_id;
#endif
        Label* parent;
        Cost g;
        Cost f;
        union
        {
            uint64_t nt;
            struct
            {
                Node n;
                Time t;
            };
        };
#ifdef USE_RESERVATION_TABLE
        Int reserves;
#endif
        PriorityQueueSizeType pqueue_index;
        std::byte state_[0];
    };
#ifdef DEBUG
    static_assert(sizeof(Label) == 5*8 + 2*4);
#else
    static_assert(sizeof(Label) == 4*8 + 2*4);
#endif

    // Comparison of labels
    struct LabelComparison
    {
        static inline bool lt(const Label* const lhs, const Label* const rhs)
        {
            // Prefer smallest f (shorter path) and break ties with fewest visits to reserved vertices
            // and then largest g (i.e., smallest h for the given f).
#ifdef USE_RESERVATION_TABLE
            return std::tie(lhs->f, lhs->reserves, rhs->g) < std::tie(rhs->f, rhs->reserves, lhs->g);
#else
            return std::tie(lhs->f, rhs->g) < std::tie(rhs->f, lhs->g);
#endif
        }
        static inline bool le(const Label* const lhs, const Label* const rhs)
        {
            // Prefer smallest f (shorter path) and break ties with fewest visits to reserved vertices
            // and then largest g (i.e., smallest h for the given f).
#ifdef USE_RESERVATION_TABLE
            return std::tie(lhs->f, lhs->reserves, rhs->g) <= std::tie(rhs->f, rhs->reserves, lhs->g);
#else
            return std::tie(lhs->f, rhs->g) <= std::tie(rhs->f, lhs->g);
#endif
        }
        static inline bool eq(const Label* const lhs, const Label* const rhs)
        {
            // Prefer smallest f (shorter path) and break ties with fewest visits to reserved vertices
            // and then largest g (i.e., smallest h for the given f).
#ifdef USE_RESERVATION_TABLE
            return std::tie(lhs->f, lhs->reserves, rhs->g) == std::tie(rhs->f, rhs->reserves, lhs->g);
#else
            return std::tie(lhs->f, rhs->g) == std::tie(rhs->f, lhs->g);
#endif
        }
    };

    // Priority queue holding labels
    class AStarPriorityQueue : public PriorityQueue<Label*, LabelComparison, PriorityQueueSizeType>
    {
      public:
        // Modify the handle in the label pointing to its position in the priority queue
        void update_index(Label* label, const PriorityQueueSizeType index)
        {
            label->pqueue_index = index;
        }

        // Check the validity of an index
        Bool check_index(Label* const& label, const PriorityQueueSizeType index) const
        {
            return (label->pqueue_index == index);
        }
    };

  public:
    struct Data
    {
        // Waypoints
        Node start;
        Vector<NodeTime> waypoints;
        Node goal;
        Time earliest_goal_time;
        Time latest_goal_time;

        // Costs
        Cost cost_offset;
        Vector<Time> latest_visit_time;
        EdgeTimePenalties edge_penalties;
        FinishTimePenalties finish_time_penalties;
#ifdef USE_GOAL_CONFLICTS
        NodeCrossingPenalties node_crossing_penalties;
#endif

        // Check if any cost is better
        bool can_be_better(const Data& previous_data);
    };

  private:
    // Instance
    const Map& map_;

    // Inputs for a run
    Data data_;

    // Solver data structures
    const Time* h_node_to_waypoint_;
    Vector<Time> h_waypoint_to_goal_;
    DistanceHeuristic& distance_heuristic_;
    MemoryPool label_pool_;
    AStarPriorityQueue open_;
    HashMap<NodeTime, Label*> frontier_without_resources_;
    HashMap<NodeTime, SmallVector<Label*, 4>> frontier_with_resources_;
#ifdef USE_RESERVATION_TABLE
    ReservationTable reservation_table_;
#endif
#ifdef DEBUG
    size_t nb_labels_;
#endif

  public:
    // Constructors
    AStar() = delete;
    AStar(const Map& map, DistanceHeuristic& distance_heuristic);
    AStar(const AStar&) = delete;
    AStar(AStar&&) = delete;
    AStar& operator=(const AStar&) = delete;
    AStar& operator=(AStar&&) = delete;
    ~AStar() = default;

    // Getters
#ifdef USE_RESERVATION_TABLE
    auto& reservation_table() { return reservation_table_; };
#endif
    auto& data() { return data_; }
    const auto& data() const { return data_; }

    // Solve
    void preprocess_input();
    void finalise();
    template<bool is_farkas>
    Pair<Vector<NodeTime>, Cost> solve();

    // Debug
#ifdef DEBUG
    Pair<Vector<NodeTime>, Cost> calculate_cost(const Vector<Node>& input_path);
    void set_verbose(const bool on = true);
#endif

  private:
    // Solve
    template<bool is_farkas, bool has_resources>
    Pair<Vector<NodeTime>, Cost> solve();

    // Create start label
    template<bool has_resources>
    void generate_start();

    // Create intermediate label
    template<bool has_resources>
    void generate_early_segment(Label* const current,
                                const Node next_n,
                                const Time next_t,
                                const Cost cost,
                                const Waypoint w,
                                const Time waypoint_time);
    template<bool has_resources>
    void generate_last_segment(Label* const current, const Node next_n, const Time next_t, const Cost cost);
    template<bool has_resources, bool is_last_segment, class... WaypointArgs>
    inline void generate(Label* const current,
                         const Node next_n,
                         const Time next_t,
                         const Cost cost,
                         WaypointArgs... waypoint_args)
    {
        if constexpr (is_last_segment)
        {
            generate_last_segment<has_resources>(current, next_n, next_t, cost, waypoint_args...);
        }
        else
        {
            generate_early_segment<has_resources>(current, next_n, next_t, cost, waypoint_args...);
        }
    }

    // Expand next - time-expanded A*
    template<Time default_cost, bool has_resources, bool is_last_segment, class... WaypointArgs>
    void generate_neighbours(Label* const current, WaypointArgs... waypoint_args);

    // Create end label
    void generate_end(Label* const current);

    // Check if a label is dominated by an existing label
    template<bool has_resources>
    AStar::Label* dominated(Label* const new_label);
};

