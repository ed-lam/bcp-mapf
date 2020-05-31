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

#include "Includes.h"
#include "Coordinates.h"
#include "Map.h"
#include "LabelPool.h"
#include "ReservationTable.h"
#include "PriorityQueue.h"
#include "EdgePenalties.h"
#include "Crossings.h"
#include "Heuristic.h"
#include "AbstractPathfinder.h"
#include <unordered_map>
#include <functional>

namespace TruffleHog
{

class AStar : public AbstractPathfinder
{
    struct Label
    {
        Label* parent;
        Cost f;
        Cost g;
        union
        {
            struct
            {
                uint64_t nt : 63;
                bool reserved : 1;
            };
            struct
            {
                Node n : 32;
                Time t : 31;
            };
        };
        Int pqueue_index;
#ifdef DEBUG
        Int label_id;
#endif
        std::byte state_[0];
    };
#ifdef DEBUG
    static_assert(sizeof(Label) == 8 + 8 + 8 + 8 + 4 + 4);
#else
    static_assert(sizeof(Label) == 8 + 8 + 8 + 8 + 4 + 4);
#endif

    // Comparison of labels in heuristic
    struct LabelCompare
    {
        ReservationTable reservation_table_;

        LabelCompare(const Int map_size) : reservation_table_(map_size) {}

        inline bool operator()(const Label* const a, const Label* const b)
        {
            // Prefer smallest f (shorter path) and break ties with smallest reserved
            // status (not reserved) and then largest g (near the end).
            return (a->f <  b->f) ||
                   (a->f == b->f && a->reserved <  b->reserved) ||
                   (a->f == b->f && a->reserved == b->reserved && a->g >  b->g) ||
                   (a->f == b->f && a->reserved == b->reserved && a->g == b->g && static_cast<bool>(rand() % 2));
        }
    };

    // Instance
    const Map& map_;

    // Labels
    LabelPool label_pool_;

    // Heuristic
    Heuristic heuristic_;

    // Penalties
    EdgePenalties edge_penalties_;
    Vector<Cost> time_finish_penalties_;
#ifdef USE_GOAL_CONFLICTS
    Vector<GoalCrossing> goal_crossings_;
#endif

    // Temporary storage for each run
    PriorityQueue<Label, LabelCompare, false> open_;
    HashTable<NodeTime, Label*> frontier_without_resources_;
    const Vector<IntCost>* h_;
    Vector<Cost> time_finish_h_;

    // Label counter
#ifdef DEBUG
    Int nb_labels_;
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
    Time max_path_length() override { return heuristic_.max_path_length(); }
    ReservationTable& reservation_table() override { return open_.cmp().reservation_table_; };
    EdgePenalties& edge_penalties() override { return edge_penalties_; }
    Vector<Cost>& time_finish_penalties() override { return time_finish_penalties_; }
#ifdef USE_GOAL_CONFLICTS
    Vector<GoalCrossing>& goal_crossings() override { return goal_crossings_; }
#endif

    // Solve
    void compute_h(const Node goal) override { heuristic_.compute_h(goal); }

    Pair<Vector<NodeTime>, Cost> solve(NodeTime start,
                                       Node goal,
                                       Time goal_earliest = 0,
                                       Time goal_latest = std::numeric_limits<Time>::max(),
                                       Cost max_cost = std::numeric_limits<Cost>::infinity()) override;

    // Debug
#ifdef DEBUG
    template<bool without_resources>
    Cost calculate_cost(const Vector<Pair<Position, Position>>& path);
    void print_crossings();
#endif

  private:
    // Check if a label is dominated by an existing label
    AStar::Label* dominated_without_resources(Label* new_label);

    // Solve
    template <bool without_resources>
    void generate_start(NodeTime start);
    void generate_end(Label* current, Cost max_cost);
    template <bool without_resources>
    void generate(Label* current,
                  Node node,
                  Cost cost,
                  Time goal_latest,
                  Cost max_cost);
    template<bool without_resources, IntCost default_cost>
    void generate_neighbours(Label* current,
                             Node goal,
                             Time goal_earliest,
                             Time goal_latest,
                             Cost max_cost);
    template<bool without_resources, IntCost default_cost>
    void generate_goal_neighbours(const Label* const current);

    template<bool without_resources>
    Pair<Vector<NodeTime>, Cost> solve_internal(NodeTime start,
                                                Node goal,
                                                Time goal_earliest = 0,
                                                Time goal_latest = std::numeric_limits<Time>::max(),
                                                Cost max_cost = std::numeric_limits<Cost>::infinity());
};

}

#endif
