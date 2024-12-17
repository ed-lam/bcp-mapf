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

#ifndef TRUFFLEHOG_ASTAR_H
#define TRUFFLEHOG_ASTAR_H

#include "problem/includes.h"
#include "pricing/coordinates.h"
#include "problem/map.h"
#include "pricing/memory_pool.h"
#include "pricing/reservation_table.h"
#include "pricing/priority_queue.h"
#include "pricing/penalties.h"
#include "pricing/distance_heuristic.h"
#include "boost/container/small_vector.hpp"

template <class T, std::size_t N>
using SmallVector = boost::container::small_vector<T, N>;

namespace TruffleHog
{

class AStar
{
    // Label for main low-level search
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
        Int pqueue_index;
        std::byte state_[0];
    };
#ifdef DEBUG
    static_assert(sizeof(Label) == 5*8 + 2*4);
#else
    static_assert(sizeof(Label) == 4*8 + 2*4);
#endif

    // Comparison of labels
    struct LabelCompare
    {
#ifdef USE_RESERVATION_TABLE
        ReservationTable reservation_table_;

        LabelCompare(const Int map_size) : reservation_table_(map_size) {}
#endif

        inline bool operator()(const Label* const a, const Label* const b) const
        {
            // Prefer smallest f (shorter path) and break ties with fewest visits to reserved vertices
            // and then largest g (i.e., smallest h for the given f).
#ifdef USE_RESERVATION_TABLE
            return (a->f <  b->f) ||
                   (a->f == b->f && a->reserves <  b->reserves) ||
                   (a->f == b->f && a->reserves == b->reserves && a->g > b->g);
#else
            return (a->f <  b->f) ||
                   (a->f == b->f && a->g > b->g);
#endif
        }
    };

    // Priority queue holding labels
    class AStarPriorityQueue : public PriorityQueue<Label, LabelCompare>
    {
        friend class AStar;

      public:
        // Inherit constructors.
        using PriorityQueue::PriorityQueue;

        // Checks.
#ifdef DEBUG
        Cost get_f(const Label* label) const
        {
            return label->f;
        }
        void check_pqueue_index() const
        {
            for (Int pqueue_index = 0; pqueue_index < size_; ++pqueue_index)
            {
                debug_assert(elts_[pqueue_index]->pqueue_index == pqueue_index);
            }
        }
        void check_label(const Label* const label)
        {
            debug_assert(label &&
                         (-1 == label->pqueue_index ||
                          (label->pqueue_index < size_ && elts_[label->pqueue_index] == label)));
        }
#endif

      protected:
        // Modify the handle in the label pointing to its position in the priority queue
        inline void update_pqueue_index(Label* label, const Int pqueue_index)
        {
            label->pqueue_index = pqueue_index;
        }

        // Reprioritise an element up or down
        void decrease_key(Label* label)
        {
            debug_assert(contains(label));
            heapify_up(label->pqueue_index);
        }
        void increase_key(Label* label)
        {
            debug_assert(contains(label));
            heapify_down(label->pqueue_index);
        }

#ifdef DEBUG
        // Check if the priority queue contains label
        inline bool contains(Label* label) const
        {
            const auto index = label->pqueue_index;
            return index < size_ && label == elts_[index];
        }
#endif
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
        EdgePenalties edge_penalties;
        FinishTimePenalties finish_time_penalties;
#ifdef USE_GOAL_CONFLICTS
        GoalPenalties goal_penalties;
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
    const Vector<IntCost>* h_node_to_waypoint_;
    Vector<IntCost> h_waypoint_to_goal_;
    Heuristic heuristic_;
    LabelPool label_pool_;
    AStarPriorityQueue open_;
    HashTable<NodeTime, Label*> frontier_without_resources_;
    HashTable<NodeTime, SmallVector<Label*, 4>> frontier_with_resources_;
#ifdef DEBUG
    size_t nb_labels_;
#endif

  public:
    // Constructors
    AStar() = delete;
    AStar(const Map& map);
    AStar(const AStar&) = delete;
    AStar(AStar&&) = delete;
    AStar& operator=(const AStar&) = delete;
    AStar& operator=(AStar&&) = delete;
    ~AStar() = default;

    // Getters
    inline auto max_path_length() const { return heuristic_.max_path_length(); }
#ifdef USE_RESERVATION_TABLE
    auto& reservation_table() { return open_.cmp().reservation_table_; };
#endif
    auto& data() { return data_; }
    const auto& data() const { return data_; }

    // Solve
    inline void compute_h(const Node goal) { heuristic_.get_h(goal); }
    void preprocess_input();
    void before_solve();
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
    template<IntCost default_cost, bool has_resources, bool is_last_segment, class... WaypointArgs>
    void generate_neighbours(Label* const current, WaypointArgs... waypoint_args);

    // Create end label
    void generate_end(Label* const current);

    // Check if a label is dominated by an existing label
    template<bool has_resources>
    AStar::Label* dominated(Label* const new_label);
};

}

#endif
