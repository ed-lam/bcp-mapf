//
// Isha Dijcks
//

#ifndef BCP_MAPF_ABSTRACTPATHFINDER_H
#define BCP_MAPF_ABSTRACTPATHFINDER_H

#include "Coordinates.h"
#include "ReservationTable.h"
#include "EdgePenalties.h"
#include "Crossings.h"

namespace TruffleHog {
    class AbstractPathfinder {

        // Getters
        virtual Time max_path_length() = 0;
        virtual ReservationTable& reservation_table() = 0;
        virtual EdgePenalties& edge_penalties() = 0;
        virtual Vector<Cost>& time_finish_penalties() = 0;
#ifdef USE_GOAL_CONFLICTS
        virtual Vector<GoalCrossing>& goal_crossings() = 0;
#endif

        // Solve
        virtual void compute_h(Node goal) = 0;

        template<bool is_farkas>
        Pair<Vector<NodeTime>, Cost> solve(NodeTime start,
                                           Node goal,
                                           Time goal_earliest = 0,
                                           Time goal_latest = std::numeric_limits<Time>::max(),
                                           Cost max_cost = std::numeric_limits<Cost>::infinity());

        // Debug
#ifdef DEBUG
        template<bool without_resources>
        Cost calculate_cost(const Vector<Pair<Position, Position>>& path);
        void set_verbose(const bool on = true);
        void print_crossings();
#endif

    };
}

#endif //BCP_MAPF_ABSTRACTPATHFINDER_H
