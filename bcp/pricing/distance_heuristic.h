#pragma once

#include "problem/map.h"
#include "types/hash_map.h"
#include "types/map_types.h"
#include "types/memory_pool.h"
#include "types/priority_queue.h"

class DistanceHeuristic
{
    // Item in the priority queue
    struct Label
    {
        Time g;
        Node n;
    };

    // Comparison of labels
    struct LabelComparison
    {
        static inline bool lt(const Label lhs, const Label rhs)
        {
            return lhs.g <  rhs.g;
        }
        static inline bool le(const Label lhs, const Label rhs)
        {
            return lhs.g <= rhs.g;
        }
        static inline bool eq(const Label lhs, const Label rhs)
        {
            return lhs.g == rhs.g;
        }
    };

    // Priority queue data structure
    using PriorityQueueSizeType = Int32;
    class HeuristicPriorityQueue : public PriorityQueue<Label, LabelComparison, PriorityQueueSizeType>
    {
      public:
        // Modify the handle in the label pointing to its position in the priority queue
        void update_index(Label, const PriorityQueueSizeType) {}

        // Check the validity of an index
        Bool check_index(Label const&, const PriorityQueueSizeType) const { return true; }
    };
    // Instance
    const Map& map_;

    // Lower bounds
    HashMap<Node, Vector<Time>> h_;

    // Solver state
    HeuristicPriorityQueue open_;

  public:
    // Constructors and destructor
    DistanceHeuristic() = delete;
    DistanceHeuristic(const Map& map);
    DistanceHeuristic(const DistanceHeuristic&) noexcept = delete;
    DistanceHeuristic(DistanceHeuristic&&) noexcept = default;
    DistanceHeuristic& operator=(const DistanceHeuristic&) noexcept = delete;
    DistanceHeuristic& operator=(DistanceHeuristic&&) noexcept = delete;
    ~DistanceHeuristic() = default;

    // Get the lower bound from every node to a goal node
    const Time* get_h(const Node goal);
    Vector<Time> get_h_using_map(const Node goal, const Map& map);

  private:
    // Check if a node has already been visited
    Bool dominated(const Node n);

    // Generate labels
    void generate_start(const Node n, Time* h);
    void generate(const Node current, const Node next, Time* h);
    void generate_neighbours(const Node current, Time* h, const Map& map);

    // Compute lower bound from every node to a goal node
    void search(const Node goal, Time* h, const Map& map);
};
