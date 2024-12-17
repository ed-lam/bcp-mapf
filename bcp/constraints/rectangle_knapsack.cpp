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

#ifdef USE_RECTANGLE_KNAPSACK_CONFLICTS

// #define PRINT_DEBUG

#include "constraints/rectangle_knapsack.h"
#include "constraints/rectangle.h"
#include "problem/problem.h"
#include "problem/variable_data.h"

#define SEPA_NAME         "rectangle_knapsack"
#define SEPA_DESC         "Separator for rectangle knapsack conflicts"
#define SEPA_PRIORITY     1000     // priority of the constraint handler for separation
#define SEPA_FREQ         1        // frequency for separating cuts; zero means to separate only in the root node
#define SEPA_MAXBOUNDDIST 1.0
#define SEPA_USESSUBSCIP  FALSE    // does the separator use a secondary SCIP instance? */
#define SEPA_DELAY        FALSE    // should separation method be delayed, if other separators found cuts? */

template<class T>
inline T signum(const T val)
{
    return (T{0} < val) - (val < T{0});
}

// Data for rectangle knapsack cuts
struct RectangleKnapsackSepaData
{
    Vector<RectangleKnapsackCut> cuts;
};

inline void append_edge(
    const Map& map,            // Map
    const Time t,              // Time of the edge
    const Position x,          // Coordinate
    const Position y,          // Coordinate
    const Direction d,         // Direction
    Vector<EdgeTime>& edges    // Output edges of the rectangle
)
{
    edges.emplace_back(map.get_id(x, y), d, t);
#ifdef PRINT_DEBUG
    const auto et = edges.back();
    // debugln("        ({},{}) {} time {}", map.get_x(et.n), map.get_y(et.n), Direction{et.d}, et.t);
#endif
}

inline bool append_vertical_boundary(
    const Map& map,            // Map
    const Time t0,             // Time at reference location
    const Position x0,         // Coordinate of reference location
    const Position y0,         // Coordinate of reference location
    const Direction d,         // Direction
    const Position y1,         // Start coordinate of the vertical boundary
    const Position y2,         // End coordinate of the vertical boundary
    Vector<EdgeTime>& edges    // Output edges of the rectangle
)
{
    // Calculate the direction the agent is moving.
    const auto y_direction = signum(y2 - y1);
    if (y_direction == 0)
    {
        return false;
    }

    // Add the edges crossing into the boundary of the rectangle.
    for (Position y = y1; y != y2 + y_direction; y += y_direction)
    {
        const auto t = t0 + y_direction * (y - y0);
        if (0 <= t &&
            0 <= x0 && x0 < map.width() &&
            0 <= y && y < map.height())
        {
            append_edge(map, t, x0, y, d, edges);
        }
    }
    return true;
}

inline bool append_horizontal_boundary(
    const Map& map,            // Map
    const Time t0,             // Time at reference location
    const Position x0,         // Coordinate of reference location
    const Position y0,         // Coordinate of reference location
    const Direction d,         // Direction
    const Position x1,         // Start coordinate of the horizontal boundary
    const Position x2,         // End coordinate of the horizontal boundary
    Vector<EdgeTime>& edges    // Output edges of the rectangle
)
{
    // Calculate the direction the agent is moving.
    const auto x_direction = signum(x2 - x1);
    if (x_direction == 0)
    {
        return false;
    }

    // Add the edges crossing into the boundary of the rectangle.
    for (Position x = x1; x != x2 + x_direction; x += x_direction)
    {
        const auto t = t0 + x_direction * (x - x0);
        if (0 <= t &&
            0 <= x && x < map.width() &&
            0 <= y0 && y0 < map.height())
        {
            append_edge(map, t, x, y0, d, edges);
        }
    }
    return true;
}

SCIP_RETCODE rectangle_knapsack_conflicts_create_cut(
    SCIP* scip,                                 // SCIP
    SCIP_ProbData* probdata,                    // Problem data
    SCIP_SEPA* sepa,                            // Separator
    RectangleKnapsackSepaData& sepadata,        // Separator data
    const Agent a1,                             // Agent 1
    const Agent a2,                             // Agent 2
    const Int a1_out_edges_begin,               // First index of edges of departure boundary for agent 1
    const Int a2_in_edges_begin,                // First index of edges of arrival boundary for agent 2
    const Int a2_out_edges_begin,               // First index of edges of departure boundary for agent 2
    const Vector<EdgeTime>& rectangle_edges,    // Edges of the rectangle
#if defined(DEBUG) or defined(PRINT_DEBUG)
    const Position a1_start_x,                  // Coordinate of agent 1 used to derive the rectangle
    const Position a1_start_y,                  // Coordinate of agent 1 used to derive the rectangle
    const Time a1_start_t,                      // Time of agent 1 used to derive the rectangle
    const Position a1_end_x,                    // Coordinate of agent 1 used to derive the rectangle
    const Position a1_end_y,                    // Coordinate of agent 1 used to derive the rectangle
    const Time a1_end_t,                        // Time of agent 1 used to derive the rectangle
    const Position a2_start_x,                  // Coordinate of agent 2 used to derive the rectangle
    const Position a2_start_y,                  // Coordinate of agent 2 used to derive the rectangle
    const Time a2_start_t,                      // Time of agent 2 used to derive the rectangle
    const Position a2_end_x,                    // Coordinate of agent 2 used to derive the rectangle
    const Position a2_end_y,                    // Coordinate of agent 2 used to derive the rectangle
    const Time a2_end_t,                        // Time of agent 2 used to derive the rectangle
#endif
    SCIP_Result* result                         // Output result
)
{
    // Create constraint name.
#ifdef DEBUG
    auto name = fmt::format("rectangle_knapsack_conflict("
                            "{},(({},{}),{}),(({},{}),{})"
                            "{},(({},{}),{}),(({},{}),{})"
                            ")",
                            a1, a1_start_x, a1_start_y, a1_start_t, a1_end_x, a1_end_y, a1_end_t,
                            a2, a2_start_x, a2_start_y, a2_start_t, a2_end_x, a2_end_y, a2_end_t);
#endif

    // Create data for the cut.
    TwoAgentRobustCut cut(scip, a1, a2, a2_in_edges_begin, rectangle_edges.size() - a2_in_edges_begin
#ifdef DEBUG
                        , std::move(name)
#endif
    );
    std::copy(rectangle_edges.begin(), rectangle_edges.end(), &cut.a1_edge_time(0));

    // Store the cut.
    Int idx;
    SCIP_CALL(SCIPprobdataAddTwoAgentRobustCut(scip,
                                               probdata,
                                               sepa,
                                               std::move(cut),
                                               3,
                                               result,
                                               &idx));
    sepadata.cuts.push_back({idx, a1_out_edges_begin, a2_in_edges_begin, a2_out_edges_begin});

    // Done.
    return SCIP_OKAY;
}

bool find_rectangle_conflict(
    SCIP* scip,                                                      // SCIP
    const Map& map,                                                  // Map
    const HashTable<EdgeTime, SCIP_Real>& a1_positive_move_edges,    // Edge weights for each agent
    const HashTable<EdgeTime, SCIP_Real>& a2_positive_move_edges,    // Edge weights for each agent
    const Time conflict_time,                                        // Time of the conflict
    const Edge* a1_path,                                             // Path of agent 1
    const Edge* a2_path,                                             // Path of agent 1
    const Int min_path_length,                                       // Length of the shorter path
    Vector<EdgeTime>& rectangle_edges,                               // Edges of the rectangle
    Int& a1_out_edges_begin,                                         // First index of edges of departure boundary for agent 1
    Int& a2_in_edges_begin,                                          // First index of edges of arrival boundary for agent 2
    Int& a2_out_edges_begin                                          // First index of edges of departure boundary for agent 2
#if defined(DEBUG) or defined(PRINT_DEBUG)
  , const Agent a1,                                                  // Agent 1
    const Agent a2,                                                  // Agent 2
    Position& output_a1_start_x,                                     // Start coordinate of agent 1
    Position& output_a1_start_y,                                     // Start coordinate of agent 1
    Time& output_a1_start_t,                                         // Time of agent 1
    Position& output_a2_start_x,                                     // Start coordinate of agent 2
    Position& output_a2_start_y,                                     // Start coordinate of agent 2
    Time& output_a2_start_t,                                         // Time of agent 2
    Position& output_a1_end_x,                                       // End coordinate of agent 1
    Position& output_a1_end_y,                                       // End coordinate of agent 1
    Time& output_a1_end_t,                                           // Time of agent 1
    Position& output_a2_end_x,                                       // End coordinate of agent 2
    Position& output_a2_end_y,                                       // End coordinate of agent 2
    Time& output_a2_end_t,                                           // Time of agent 2
    SCIP_Real& output_lhs                                            // LHS
#endif
)
{
    // Print.
// #ifdef PRINT_DEBUG
//     static int iter = 0;
//     ++iter;
//     {
//         const NodeTime nt{a1_path[conflict_time].n, conflict_time};
//         debugln("Checking conflict at ({},{}) time {} for agents {} and {} in iter {}",
//                 map.get_x(nt.n), map.get_y(nt.n), nt.t, a1, a2, iter);
//         debugln("Agent {:3d}: {}", a1, format_path_spaced(SCIPgetProbData(scip), min_path_length, a1_path));
//         debugln("Agent {:3d}: {}", a2, format_path_spaced(SCIPgetProbData(scip), min_path_length, a2_path));
//     }
// #endif

    // Get the movement directions.
    Direction x_dir = Direction::INVALID;
    Direction y_dir = Direction::INVALID;
    for (Time t = conflict_time; t < min_path_length; ++t)
    {
        if (a1_path[t].d == Direction::EAST || a1_path[t].d == Direction::WEST)
        {
            x_dir = a1_path[t].d;
            break;
        }
        else if (a2_path[t].d == Direction::EAST || a2_path[t].d == Direction::WEST)
        {
            x_dir = a2_path[t].d;
            break;
        }
        else if (a1_path[t].d == Direction::WAIT || a2_path[t].d == Direction::WAIT)
        {
            return false;
        }
    }
    for (Time t = conflict_time; t < min_path_length; ++t)
    {
        if (a1_path[t].d == Direction::NORTH || a1_path[t].d == Direction::SOUTH)
        {
            y_dir = a1_path[t].d;
            break;
        }
        else if (a2_path[t].d == Direction::NORTH || a2_path[t].d == Direction::SOUTH)
        {
            y_dir = a2_path[t].d;
            break;
        }
        else if (a1_path[t].d == Direction::WAIT || a2_path[t].d == Direction::WAIT)
        {
            return false;
        }
    }
    if (x_dir == Direction::INVALID || y_dir == Direction::INVALID)
    {
        return false;
    }

    // Find the first time when the direction changes.
    Time start_t = conflict_time;
    Time end_t = conflict_time;
    for (;
         (start_t >= 0) &&
         (a1_path[start_t].d == x_dir || a1_path[start_t].d == y_dir) &&
         (a2_path[start_t].d == x_dir || a2_path[start_t].d == y_dir);
         --start_t);
    start_t++;
    for (;
         (end_t < min_path_length - 1) &&
         (a1_path[end_t].d == x_dir || a1_path[end_t].d == y_dir) &&
         (a2_path[end_t].d == x_dir || a2_path[end_t].d == y_dir);
         ++end_t);
    if (end_t <= start_t + 2)
    {
        return false;
    }
    debug_assert(0 <= start_t && end_t > start_t + 2 && end_t < min_path_length);

    // Cannot find a rectangle conflict if the start location of the two agents are the same.
    if (a1_path[start_t].n == a2_path[start_t].n)
    {
        return false;
    }

    // Check.
#ifdef DEBUG
    for (Time t = start_t; t < end_t; ++t)
    {
        debug_assert(a1_path[t].d != Direction::WAIT);
        debug_assert(a2_path[t].d != Direction::WAIT);
    }
#endif

    // Get the coordinates of those times.
    const auto [a1_start_x, a1_start_y] = map.get_xy(a1_path[start_t].n);
    const auto [a2_start_x, a2_start_y] = map.get_xy(a2_path[start_t].n);
    const auto [a1_end_x, a1_end_y] = map.get_xy(a1_path[end_t].n);
    const auto [a2_end_x, a2_end_y] = map.get_xy(a2_path[end_t].n);

    // Check that there is no wait inside the rectangle.
    debug_assert(std::abs(a1_end_x - a1_start_x) + std::abs(a1_end_y - a1_start_y) == end_t - start_t);
    debug_assert(std::abs(a2_end_x - a2_start_x) + std::abs(a2_end_y - a2_start_y) == end_t - start_t);

    // Calculate the direction of the two agents.
    Direction a1_dir;
    Direction a2_dir;
    if (a1_start_x <= a2_start_x && a1_start_y <= a2_start_y &&
        a1_end_x   >= a2_end_x   && a1_end_y   >= a2_end_y   &&
        a1_start_x <= a1_end_x   && a1_start_y >= a1_end_y   &&
        a2_start_x <= a2_end_x   && a2_start_y >= a2_end_y)
    {
        a1_dir = Direction::EAST;
        a2_dir = Direction::NORTH;
    }
    else if (a2_start_x <= a1_start_x && a2_start_y <= a1_start_y &&
             a2_end_x   >= a1_end_x   && a2_end_y   >= a1_end_y   &&
             a2_start_x <= a2_end_x   && a2_start_y >= a2_end_y   &&
             a1_start_x <= a1_end_x   && a1_start_y >= a1_end_y)
    {
        a1_dir = Direction::NORTH;
        a2_dir = Direction::EAST;
    }
    else if (a1_start_x >= a2_start_x && a1_start_y <= a2_start_y &&
             a1_end_x   <= a2_end_x   && a1_end_y   >= a2_end_y   &&
             a1_start_x >= a1_end_x   && a1_start_y >= a1_end_y   &&
             a2_start_x >= a2_end_x   && a2_start_y >= a2_end_y)
    {
        a1_dir = Direction::WEST;
        a2_dir = Direction::NORTH;
    }
    else if (a2_start_x >= a1_start_x && a2_start_y <= a1_start_y &&
             a2_end_x   <= a1_end_x   && a2_end_y   >= a1_end_y   &&
             a2_start_x >= a2_end_x   && a2_start_y >= a2_end_y   &&
             a1_start_x >= a1_end_x   && a1_start_y >= a1_end_y)
    {
        a1_dir = Direction::NORTH;
        a2_dir = Direction::WEST;
    }
    else if (a1_start_x <= a2_start_x && a1_start_y >= a2_start_y &&
             a1_end_x   >= a2_end_x   && a1_end_y   <= a2_end_y   &&
             a1_start_x <= a1_end_x   && a1_start_y <= a1_end_y   &&
             a2_start_x <= a2_end_x   && a2_start_y <= a2_end_y)
    {
        a1_dir = Direction::EAST;
        a2_dir = Direction::SOUTH;
    }
    else if (a2_start_x <= a1_start_x && a2_start_y >= a1_start_y &&
             a2_end_x   >= a1_end_x   && a2_end_y   <= a1_end_y &&
             a2_start_x <= a2_end_x   && a2_start_y <= a2_end_y &&
             a1_start_x <= a1_end_x   && a1_start_y <= a1_end_y)
    {
        a1_dir = Direction::SOUTH;
        a2_dir = Direction::EAST;
    }
    else if (a1_start_x >= a2_start_x && a1_start_y >= a2_start_y &&
             a1_end_x   <= a2_end_x   && a1_end_y   <= a2_end_y &&
             a1_start_x >= a1_end_x   && a1_start_y <= a1_end_y &&
             a2_start_x >= a2_end_x   && a2_start_y <= a2_end_y)
    {
        a1_dir = Direction::WEST;
        a2_dir = Direction::SOUTH;
    }
    else if (a2_start_x >= a1_start_x && a2_start_y >= a1_start_y &&
             a2_end_x   <= a1_end_x   && a2_end_y   <= a1_end_y   &&
             a2_start_x >= a2_end_x   && a2_start_y <= a2_end_y   &&
             a1_start_x >= a1_end_x   && a1_start_y <= a1_end_y)
    {
        a1_dir = Direction::SOUTH;
        a2_dir = Direction::WEST;
    }
    else
    {
        return false;
    }

    // Compute exit time.
    const auto exit_t = end_t - 1;

    // Append the edges of agent 1.
    rectangle_edges.clear();
    if (a1_dir == Direction::NORTH || a1_dir == Direction::SOUTH)
    {
        const auto x_bound = a2_start_x + (a2_dir == Direction::EAST ? 1 : -1);
        const auto y_bound = a1_end_y + (a1_dir == Direction::NORTH ? 1 : -1);
        // debugln("    Agent {} arrival boundary:", a1);
        if (!append_horizontal_boundary(map, start_t, a1_start_x, a1_start_y, a1_dir, x_bound, a2_end_x, rectangle_edges))
        {
            return false;
        }

        // debugln("    Agent {} departure boundary:", a1);
        a1_out_edges_begin = rectangle_edges.size();
        if (!append_horizontal_boundary(map, exit_t, a1_end_x, y_bound, a1_dir, x_bound, a2_end_x, rectangle_edges))
        {
            return false;
        }
    }
    else
    {
        debug_assert(a1_dir == Direction::EAST || a1_dir == Direction::WEST);

        const auto y_bound = a2_start_y + (a2_dir == Direction::NORTH ? -1 : 1);
        const auto x_bound = a1_end_x + (a1_dir == Direction::EAST ? -1 : 1);

        // debugln("    Agent {} arrival boundary:", a1);
        if (!append_vertical_boundary(map, start_t, a1_start_x, a1_start_y, a1_dir, y_bound, a2_end_y, rectangle_edges))
        {
            return false;
        }

        // debugln("    Agent {} departure boundary:", a1);
        a1_out_edges_begin = rectangle_edges.size();
        if (!append_vertical_boundary(map, exit_t, x_bound, a1_end_y, a1_dir, y_bound, a2_end_y, rectangle_edges))
        {
            return false;
        }
    }

    // Append the edges of agent 2.
    if (a2_dir == Direction::NORTH || a2_dir == Direction::SOUTH)
    {
        const auto x_bound = a1_start_x + (a1_dir == Direction::EAST ? 1 : -1);
        const auto y_bound = a2_end_y + (a2_dir == Direction::NORTH ? 1 : -1);

        // debugln("    Agent {} arrival boundary:", a2);
        a2_in_edges_begin = rectangle_edges.size();
        if (!append_horizontal_boundary(map, start_t, a2_start_x, a2_start_y, a2_dir, x_bound, a1_end_x, rectangle_edges))
        {
            return false;
        }

        // debugln("    Agent {} departure boundary:", a2);
        a2_out_edges_begin = rectangle_edges.size();
        if (!append_horizontal_boundary(map, exit_t, a2_end_x, y_bound, a2_dir, x_bound, a1_end_x, rectangle_edges))
        {
            return false;
        }
    }
    else
    {
        debug_assert(a2_dir == Direction::EAST || a2_dir == Direction::WEST);

        const auto y_bound = a1_start_y + (a1_dir == Direction::NORTH ? -1 : 1);
        const auto x_bound = a2_end_x + (a2_dir == Direction::EAST ? -1 : 1);

        // debugln("    Agent {} arrival boundary:", a2);
        a2_in_edges_begin = rectangle_edges.size();
        if (!append_vertical_boundary(map, start_t, a2_start_x, a2_start_y, a2_dir, y_bound, a1_end_y, rectangle_edges))
        {
            return false;
        }

        // debugln("    Agent {} departure boundary:", a2);
        a2_out_edges_begin = rectangle_edges.size();
        if (!append_vertical_boundary(map, exit_t, x_bound, a2_end_y, a2_dir, y_bound, a1_end_y, rectangle_edges))
        {
            return false;
        }
    }

    // Determine if the cut is violated.
    SCIP_Real lhs = 0.0;
    for (Int idx = 0; idx < a2_in_edges_begin; ++idx)
    {
        const auto& et = rectangle_edges[idx];
        if (auto it = a1_positive_move_edges.find(et); it != a1_positive_move_edges.end())
        {
            lhs += it->second;
        }
    }
    for (Int idx = a2_in_edges_begin; idx < static_cast<Int>(rectangle_edges.size()); ++idx)
    {
        const auto& et = rectangle_edges[idx];
        if (auto it = a2_positive_move_edges.find(et); it != a2_positive_move_edges.end())
        {
            lhs += it->second;
        }
    }
    debug_assert(SCIPisSumLE(scip, lhs, 4.0));
    // debugln("    LHS: {:.4f}", lhs);

    // Store outputs.
#if defined(DEBUG) or defined(PRINT_DEBUG)
    output_a1_start_x = a1_start_x;
    output_a1_start_y = a1_start_y;
    output_a1_start_t = start_t;
    output_a2_start_x = a2_start_x;
    output_a2_start_y = a2_start_y;
    output_a2_start_t = start_t;
    output_a1_end_x = a1_end_x;
    output_a1_end_y = a1_end_y;
    output_a1_end_t = end_t;
    output_a2_end_x = a2_end_x;
    output_a2_end_y = a2_end_y;
    output_a2_end_t = end_t;
    output_lhs = lhs;
#endif

    // Check if the cut is violated.
    return SCIPisSumGT(scip, lhs, 3.0 + CUT_VIOLATION);
}

// bool find_rectangle_conflict_new(
//     SCIP* scip,                                                      // SCIP
//     const Map& map,                                                  // Map
//     const HashTable<EdgeTime, SCIP_Real>& a1_positive_move_edges,    // Edge weights for each agent
//     const HashTable<EdgeTime, SCIP_Real>& a2_positive_move_edges,    // Edge weights for each agent
//     const Time conflict_time,                                        // Time of the conflict
//     const Edge* a1_path,                                             // Path of agent 1
//     const Edge* a2_path,                                             // Path of agent 1
//     const Int min_path_length,                                       // Length of the shorter path
//     Vector<EdgeTime>& rectangle_edges,                               // Edges of the rectangle
//     Int& a1_out_edges_begin,                                         // First index of edges of departure boundary for agent 1
//     Int& a2_in_edges_begin,                                          // First index of edges of arrival boundary for agent 2
//     Int& a2_out_edges_begin                                          // First index of edges of departure boundary for agent 2
// #if defined(DEBUG) or defined(PRINT_DEBUG)
//   , const Agent a1,                                                  // Agent 1
//     const Agent a2,                                                  // Agent 2
//     Position& output_a1_start_x,                                     // Start coordinate of agent 1
//     Position& output_a1_start_y,                                     // Start coordinate of agent 1
//     Time& output_a1_start_t,                                         // Time of agent 1
//     Position& output_a2_start_x,                                     // Start coordinate of agent 2
//     Position& output_a2_start_y,                                     // Start coordinate of agent 2
//     Time& output_a2_start_t,                                         // Time of agent 2
//     Position& output_a1_end_x,                                       // End coordinate of agent 1
//     Position& output_a1_end_y,                                       // End coordinate of agent 1
//     Time& output_a1_end_t,                                           // Time of agent 1
//     Position& output_a2_end_x,                                       // End coordinate of agent 2
//     Position& output_a2_end_y,                                       // End coordinate of agent 2
//     Time& output_a2_end_t,                                           // Time of agent 2
//     SCIP_Real& output_lhs                                            // LHS
// #endif
// )
// {
//     // Print.
// // #ifdef PRINT_DEBUG
// //     static int iter = 0;
// //     ++iter;
// //     {
// //         const NodeTime nt{a1_path[conflict_time].n, conflict_time};
// //         debugln("Checking conflict at ({},{}) time {} for agents {} and {} in iter {}",
// //                 map.get_x(nt.n), map.get_y(nt.n), nt.t, a1, a2, iter);
// //         debugln("Agent {:3d}: {}", a1, format_path_spaced(SCIPgetProbData(scip), min_path_length, a1_path));
// //         debugln("Agent {:3d}: {}", a2, format_path_spaced(SCIPgetProbData(scip), min_path_length, a2_path));
// //     }
// // #endif
//
//     // Get the movement directions.
//     Direction x_dir = Direction::INVALID;
//     Direction y_dir = Direction::INVALID;
//     for (Time t = conflict_time; t < min_path_length; ++t)
//     {
//         if (a1_path[t].d == Direction::EAST || a1_path[t].d == Direction::WEST)
//         {
//             x_dir = a1_path[t].d;
//             break;
//         }
//         else if (a2_path[t].d == Direction::EAST || a2_path[t].d == Direction::WEST)
//         {
//             x_dir = a2_path[t].d;
//             break;
//         }
//         else if (a1_path[t].d == Direction::WAIT || a2_path[t].d == Direction::WAIT)
//         {
//             return false;
//         }
//     }
//     for (Time t = conflict_time; t < min_path_length; ++t)
//     {
//         if (a1_path[t].d == Direction::NORTH || a1_path[t].d == Direction::SOUTH)
//         {
//             y_dir = a1_path[t].d;
//             break;
//         }
//         else if (a2_path[t].d == Direction::NORTH || a2_path[t].d == Direction::SOUTH)
//         {
//             y_dir = a2_path[t].d;
//             break;
//         }
//         else if (a1_path[t].d == Direction::WAIT || a2_path[t].d == Direction::WAIT)
//         {
//             return false;
//         }
//     }
//     if (x_dir == Direction::INVALID || y_dir == Direction::INVALID)
//     {
//         return false;
//     }
//
//     // Find the first time when the direction changes.
//     Time start_t = conflict_time;
//     Time end_t = conflict_time - 1;
//     for (;
//          (start_t > 0) &&
//          (a1_path[start_t - 1].d == x_dir || a1_path[start_t - 1].d == y_dir) &&
//          (a2_path[start_t - 1].d == x_dir || a2_path[start_t - 1].d == y_dir);
//          --start_t);
//     for (;
//          (end_t < min_path_length - 1) &&
//          (a1_path[end_t + 1].d == x_dir || a1_path[end_t + 1].d == y_dir) &&
//          (a2_path[end_t + 1].d == x_dir || a2_path[end_t + 1].d == y_dir);
//          ++end_t);
//     if (start_t >= end_t)
//     {
//         return false;
//     }
//
//     // Make the rectangle smaller until the agents enter and exit in orthogonal directions.
//     for (; start_t <= conflict_time; ++start_t)
//         if ((a1_path[start_t].d != a2_path[start_t].d) &&
//             (a1_path[start_t].d == x_dir || a1_path[start_t].d == y_dir) &&
//             (a2_path[start_t].d == x_dir || a2_path[start_t].d == y_dir))
//         {
//             goto FOUND_START_T_1;
//         }
//     return false;
//     FOUND_START_T_1:
//     const auto original_start_t = start_t;
//     const auto original_end_t = end_t;
//     for (; end_t >= std::max(conflict_time, start_t + 1); --end_t)
//         if (a1_path[end_t].d == a1_path[start_t].d && a2_path[end_t].d == a2_path[start_t].d)
//         {
//             goto FOUND_END_T;
//         }
//
//     // Make another attempt by swapping the directions of the two agents.
//     for (start_t = original_start_t; start_t <= conflict_time; ++start_t)
//          if (a1_path[start_t].d == a2_path[original_start_t].d && a2_path[start_t].d == a1_path[original_start_t].d)
//          {
//             goto FOUND_START_T_2;
//          }
//     return false;
//     FOUND_START_T_2:
//     for (end_t = original_end_t; end_t >= std::max(conflict_time, start_t + 1); --end_t)
//         if (a1_path[end_t].d == a1_path[start_t].d && a2_path[end_t].d == a2_path[start_t].d)
//         {
//             goto FOUND_END_T;
//         }
//     return false;
//     FOUND_END_T:
//
//     // Check.
// #ifdef DEBUG
//     for (Time t = start_t; t <= end_t; ++t)
//     {
//         debug_assert(a1_path[t].d != Direction::WAIT);
//         debug_assert(a2_path[t].d != Direction::WAIT);
//     }
// #endif
//
//     // Get the coordinates of those times.
//     const auto [a1_start_x, a1_start_y] = map.get_xy(a1_path[start_t].n);
//     const auto [a2_start_x, a2_start_y] = map.get_xy(a2_path[start_t].n);
//     const auto [a1_end_x, a1_end_y] = map.get_xy(a1_path[end_t].n);
//     const auto [a2_end_x, a2_end_y] = map.get_xy(a2_path[end_t].n);
//     if (a1_path[start_t].n == a2_path[start_t].n && a1_path[end_t].n == a2_path[end_t].n)
//     {
//         return false;
//     }
//
//     // Check that there is no wait inside the rectangle.
//     debug_assert(std::abs(a1_end_x - a1_start_x) + std::abs(a1_end_y - a1_start_y) == end_t - start_t);
//     debug_assert(std::abs(a2_end_x - a2_start_x) + std::abs(a2_end_y - a2_start_y) == end_t - start_t);
//
//     // Calculate the direction of the two agents.
//     Direction a1_dir;
//     Direction a2_dir;
//     if (a1_start_x <= a2_start_x && a1_start_y <= a2_start_y &&
//         a1_end_x   >= a2_end_x   && a1_end_y   >= a2_end_y   &&
//         a1_start_x <= a1_end_x   && a1_start_y >= a1_end_y   &&
//         a2_start_x <= a2_end_x   && a2_start_y >= a2_end_y)
//     {
//         a1_dir = Direction::EAST;
//         a2_dir = Direction::NORTH;
//     }
//     else if (a2_start_x <= a1_start_x && a2_start_y <= a1_start_y &&
//              a2_end_x   >= a1_end_x   && a2_end_y   >= a1_end_y   &&
//              a2_start_x <= a2_end_x   && a2_start_y >= a2_end_y   &&
//              a1_start_x <= a1_end_x   && a1_start_y >= a1_end_y)
//     {
//         a1_dir = Direction::NORTH;
//         a2_dir = Direction::EAST;
//     }
//     else if (a1_start_x >= a2_start_x && a1_start_y <= a2_start_y &&
//              a1_end_x   <= a2_end_x   && a1_end_y   >= a2_end_y   &&
//              a1_start_x >= a1_end_x   && a1_start_y >= a1_end_y   &&
//              a2_start_x >= a2_end_x   && a2_start_y >= a2_end_y)
//     {
//         a1_dir = Direction::WEST;
//         a2_dir = Direction::NORTH;
//     }
//     else if (a2_start_x >= a1_start_x && a2_start_y <= a1_start_y &&
//              a2_end_x   <= a1_end_x   && a2_end_y   >= a1_end_y   &&
//              a2_start_x >= a2_end_x   && a2_start_y >= a2_end_y   &&
//              a1_start_x >= a1_end_x   && a1_start_y >= a1_end_y)
//     {
//         a1_dir = Direction::NORTH;
//         a2_dir = Direction::WEST;
//     }
//     else if (a1_start_x <= a2_start_x && a1_start_y >= a2_start_y &&
//              a1_end_x   >= a2_end_x   && a1_end_y   <= a2_end_y   &&
//              a1_start_x <= a1_end_x   && a1_start_y <= a1_end_y   &&
//              a2_start_x <= a2_end_x   && a2_start_y <= a2_end_y)
//     {
//         a1_dir = Direction::EAST;
//         a2_dir = Direction::SOUTH;
//     }
//     else if (a2_start_x <= a1_start_x && a2_start_y >= a1_start_y &&
//              a2_end_x   >= a1_end_x   && a2_end_y   <= a1_end_y &&
//              a2_start_x <= a2_end_x   && a2_start_y <= a2_end_y &&
//              a1_start_x <= a1_end_x   && a1_start_y <= a1_end_y)
//     {
//         a1_dir = Direction::SOUTH;
//         a2_dir = Direction::EAST;
//     }
//     else if (a1_start_x >= a2_start_x && a1_start_y >= a2_start_y &&
//              a1_end_x   <= a2_end_x   && a1_end_y   <= a2_end_y &&
//              a1_start_x >= a1_end_x   && a1_start_y <= a1_end_y &&
//              a2_start_x >= a2_end_x   && a2_start_y <= a2_end_y)
//     {
//         a1_dir = Direction::WEST;
//         a2_dir = Direction::SOUTH;
//     }
//     else if (a2_start_x >= a1_start_x && a2_start_y >= a1_start_y &&
//              a2_end_x   <= a1_end_x   && a2_end_y   <= a1_end_y   &&
//              a2_start_x >= a2_end_x   && a2_start_y <= a2_end_y   &&
//              a1_start_x >= a1_end_x   && a1_start_y <= a1_end_y)
//     {
//         a1_dir = Direction::SOUTH;
//         a2_dir = Direction::WEST;
//     }
//     else
//     {
//         return false;
//     }
//     if (a1_dir != a1_path[start_t].d || a1_dir != a1_path[end_t].d ||
//         a2_dir != a2_path[start_t].d || a2_dir != a2_path[end_t].d)
//     {
//         return false;
//     }
//
//     // Print.
//     // debugln("Agent {}: (({},{}),{}) to (({},{}),{}) {}",
//     //         a1,
//     //         a1_start_x, a1_start_y, start_t,
//     //         a1_end_x, a1_end_y, end_t,
//     //         a1_dir);
//     // debugln("Agent {}: (({},{}),{}) to (({},{}),{}) {}",
//     //         a2,
//     //         a2_start_x, a2_start_y, start_t,
//     //         a2_end_x, a2_end_y, end_t,
//     //         a2_dir);
//
//     // Append the edges of agent 1.
//     rectangle_edges.clear();
//     SCIP_Real lhs = 0.0;
//     if (a1_dir == Direction::NORTH || a1_dir == Direction::SOUTH)
//     {
//         const auto a1_rect_start_x = a2_start_x;
//         const auto a1_rect_end_x = a2_end_x;
//         const auto a1_rect_dir_x = signum(a1_rect_end_x - a1_rect_start_x);
//         const auto a1_rect_start_t = start_t - std::abs(a1_start_x - a1_rect_start_x);
//         const auto a1_rect_y_diff = std::abs(a1_end_y - a1_start_y);
//         const auto a1_rect_end_t = a1_rect_start_t + a1_rect_y_diff;
//         // debugln("    Agent {} arrival boundary:", a1);
//         for (Position x = a1_rect_start_x; x != a1_rect_end_x + a1_rect_dir_x; x += a1_rect_dir_x)
//         {
//             const auto y = a1_start_y;
//             const auto t = a1_rect_start_t + std::abs(x - a1_rect_start_x);
//             if (t >= 0)
//             {
//                 auto& et = rectangle_edges.emplace_back();
//                 et.n = map.get_id(x, y);
//                 et.t = t;
//                 et.d = a1_dir;
//                 // debugln("        ({},{}) {} time {}", map.get_x(et.n), map.get_y(et.n), Direction{et.d}, et.t);
//
//                 if (auto it = a1_positive_move_edges.find(et); it != a1_positive_move_edges.end())
//                 {
//                     lhs += it->second;
//                 }
//             }
//         }
//         // debugln("    Agent {} departure boundary:", a1);
//         a1_out_edges_begin = rectangle_edges.size();
//         for (Position x = a1_rect_start_x; x != a1_rect_end_x + a1_rect_dir_x; x += a1_rect_dir_x)
//         {
//             const auto y = a1_end_y;
//             const auto t = a1_rect_end_t + std::abs(x - a1_rect_start_x);
//             if (t >= std::max(a1_rect_end_t, a1_rect_y_diff))
//             {
//                 auto& et = rectangle_edges.emplace_back();
//                 et.n = map.get_id(x, y);
//                 et.t = t;
//                 et.d = a1_dir;
//                 // debugln("        ({},{}) {} time {}", map.get_x(et.n), map.get_y(et.n), Direction{et.d}, et.t);
//
//                 if (auto it = a1_positive_move_edges.find(et); it != a1_positive_move_edges.end())
//                 {
//                     lhs += it->second;
//                 }
//             }
//         }
//     }
//     else
//     {
//         debug_assert(a1_dir == Direction::EAST || a1_dir == Direction::WEST);
//
//         const auto a1_rect_start_y = a2_start_y;
//         const auto a1_rect_end_y = a2_end_y;
//         const auto a1_rect_dir_y = signum(a1_rect_end_y - a1_rect_start_y);
//         const auto a1_rect_start_t = start_t - std::abs(a1_start_y - a1_rect_start_y);
//         const auto a1_rect_x_diff = std::abs(a1_end_x - a1_start_x);
//         const auto a1_rect_end_t = a1_rect_start_t + a1_rect_x_diff;
//         // debugln("    Agent {} arrival boundary:", a1);
//         for (Position y = a1_rect_start_y; y != a1_rect_end_y + a1_rect_dir_y; y += a1_rect_dir_y)
//         {
//             const auto x = a1_start_x;
//             const auto t = a1_rect_start_t + std::abs(y - a1_rect_start_y);
//             if (t >= 0)
//             {
//                 auto& et = rectangle_edges.emplace_back();
//                 et.n = map.get_id(x, y);
//                 et.t = t;
//                 et.d = a1_dir;
//                 // debugln("        ({},{}) {} time {}", map.get_x(et.n), map.get_y(et.n), Direction{et.d}, et.t);
//
//                 if (auto it = a1_positive_move_edges.find(et); it != a1_positive_move_edges.end())
//                 {
//                     lhs += it->second;
//                 }
//             }
//         }
//         // debugln("    Agent {} departure boundary:", a1);
//         a1_out_edges_begin = rectangle_edges.size();
//         for (Position y = a1_rect_start_y; y != a1_rect_end_y + a1_rect_dir_y; y += a1_rect_dir_y)
//         {
//             const auto x = a1_end_x;
//             const auto t = a1_rect_end_t + std::abs(y - a1_rect_start_y);
//             if (t >= std::max(a1_rect_end_t, a1_rect_x_diff))
//             {
//                 auto& et = rectangle_edges.emplace_back();
//                 et.n = map.get_id(x, y);
//                 et.t = t;
//                 et.d = a1_dir;
//                 // debugln("        ({},{}) {} time {}", map.get_x(et.n), map.get_y(et.n), Direction{et.d}, et.t);
//
//                 if (auto it = a1_positive_move_edges.find(et); it != a1_positive_move_edges.end())
//                 {
//                     lhs += it->second;
//                 }
//             }
//         }
//     }
//
//     // Append the edges of agent 2.
//     if (a2_dir == Direction::NORTH || a2_dir == Direction::SOUTH)
//     {
//         const auto a2_rect_start_x = a1_start_x;
//         const auto a2_rect_end_x = a1_end_x;
//         const auto a2_rect_dir_x = signum(a2_rect_end_x - a2_rect_start_x);
//         const auto a2_rect_start_t = start_t - std::abs(a2_start_x - a2_rect_start_x);
//         const auto a2_rect_y_diff = std::abs(a2_end_y - a2_start_y);
//         const auto a2_rect_end_t = a2_rect_start_t + a2_rect_y_diff;
//         // debugln("    Agent {} arrival boundary:", a2);
//         a2_in_edges_begin = rectangle_edges.size();
//         for (Position x = a2_rect_start_x; x != a2_rect_end_x + a2_rect_dir_x; x += a2_rect_dir_x)
//         {
//             const auto y = a2_start_y;
//             const auto t = a2_rect_start_t + std::abs(x - a2_rect_start_x);
//             if (t >= 0)
//             {
//                 auto& et = rectangle_edges.emplace_back();
//                 et.n = map.get_id(x, y);
//                 et.t = t;
//                 et.d = a2_dir;
//                 // debugln("        ({},{}) {} time {}", map.get_x(et.n), map.get_y(et.n), Direction{et.d}, et.t);
//
//                 if (auto it = a2_positive_move_edges.find(et); it != a2_positive_move_edges.end())
//                 {
//                     lhs += it->second;
//                 }
//             }
//         }
//         // debugln("    Agent {} departure boundary:", a2);
//         a2_out_edges_begin = rectangle_edges.size();
//         for (Position x = a2_rect_start_x; x != a2_rect_end_x + a2_rect_dir_x; x += a2_rect_dir_x)
//         {
//             const auto y = a2_end_y;
//             const auto t = a2_rect_end_t + std::abs(x - a2_rect_start_x);
//             if (t >= std::max(a2_rect_end_t, a2_rect_y_diff))
//             {
//                 auto& et = rectangle_edges.emplace_back();
//                 et.n = map.get_id(x, y);
//                 et.t = t;
//                 et.d = a2_dir;
//                 // debugln("        ({},{}) {} time {}", map.get_x(et.n), map.get_y(et.n), Direction{et.d}, et.t);
//
//                 if (auto it = a2_positive_move_edges.find(et); it != a2_positive_move_edges.end())
//                 {
//                     lhs += it->second;
//                 }
//             }
//         }
//     }
//     else
//     {
//         debug_assert(a2_dir == Direction::EAST || a2_dir == Direction::WEST);
//
//         const auto a2_rect_start_y = a1_start_y;
//         const auto a2_rect_end_y = a1_end_y;
//         const auto a2_rect_dir_y = signum(a2_rect_end_y - a2_rect_start_y);
//         const auto a2_rect_start_t = start_t - std::abs(a2_start_y - a2_rect_start_y);
//         const auto a2_rect_x_diff = std::abs(a2_end_x - a2_start_x);
//         const auto a2_rect_end_t = a2_rect_start_t + a2_rect_x_diff;
//         // debugln("    Agent {} arrival boundary:", a2);
//         a2_in_edges_begin = rectangle_edges.size();
//         for (Position y = a2_rect_start_y; y != a2_rect_end_y + a2_rect_dir_y; y += a2_rect_dir_y)
//         {
//             const auto x = a2_start_x;
//             const auto t = a2_rect_start_t + std::abs(y - a2_rect_start_y);
//             if (t >= 0)
//             {
//                 auto& et = rectangle_edges.emplace_back();
//                 et.n = map.get_id(x, y);
//                 et.t = t;
//                 et.d = a2_dir;
//                 // debugln("        ({},{}) {} time {}", map.get_x(et.n), map.get_y(et.n), Direction{et.d}, et.t);
//
//                 if (auto it = a2_positive_move_edges.find(et); it != a2_positive_move_edges.end())
//                 {
//                     lhs += it->second;
//                 }
//             }
//         }
//         // debugln("    Agent {} departure boundary:", a2);
//         a2_out_edges_begin = rectangle_edges.size();
//         for (Position y = a2_rect_start_y; y != a2_rect_end_y + a2_rect_dir_y; y += a2_rect_dir_y)
//         {
//             const auto x = a2_end_x;
//             const auto t = a2_rect_end_t + std::abs(y - a2_rect_start_y);
//             if (t >= std::max(a2_rect_end_t, a2_rect_x_diff))
//             {
//                 auto& et = rectangle_edges.emplace_back();
//                 et.n = map.get_id(x, y);
//                 et.t = t;
//                 et.d = a2_dir;
//                 // debugln("        ({},{}) {} time {}", map.get_x(et.n), map.get_y(et.n), Direction{et.d}, et.t);
//
//                 if (auto it = a2_positive_move_edges.find(et); it != a2_positive_move_edges.end())
//                 {
//                     lhs += it->second;
//                 }
//             }
//         }
//     }
//     debug_assert(SCIPisSumLE(scip, lhs, 4.0));
//     // debugln("    LHS: {:.4f}", lhs);
//
//     // Store outputs.
// #if defined(DEBUG) or defined(PRINT_DEBUG)
//     output_a1_start_x = a1_start_x;
//     output_a1_start_y = a1_start_y;
//     output_a1_start_t = start_t;
//     output_a2_start_x = a2_start_x;
//     output_a2_start_y = a2_start_y;
//     output_a2_start_t = start_t;
//     output_a1_end_x = a1_end_x;
//     output_a1_end_y = a1_end_y;
//     output_a1_end_t = end_t;
//     output_a2_end_x = a2_end_x;
//     output_a2_end_y = a2_end_y;
//     output_a2_end_t = end_t;
//     output_lhs = lhs;
// #endif
//
//     // Check if the cut is violated.
//     return SCIPisSumGT(scip, lhs, 3.0 + CUT_VIOLATION);
// }

// Separator
static
SCIP_RETCODE rectangle_knapsack_conflicts_separate(
    SCIP* scip,            // SCIP
    SCIP_SEPA* sepa,       // Separator
    SCIP_RESULT* result    // Pointer to store the result of the separation call
)
{
    // Print.
    debugln("Starting separator for rectangle knapsack conflicts on solution with obj {:.6f}:",
            SCIPgetSolOrigObj(scip, nullptr));

    // Get separator data.
    auto sepadata = reinterpret_cast<RectangleKnapsackSepaData*>(SCIPsepaGetData(sepa));
    debug_assert(sepadata);

    // Get problem data.
    auto probdata = SCIPgetProbData(scip);
    const auto N = SCIPprobdataGetN(probdata);
    const auto& map = SCIPprobdataGetMap(probdata);

    // Skip this separator if an earlier separator found cuts.
    auto& found_cuts = SCIPprobdataGetFoundCutsIndicator(probdata);
    if (found_cuts)
    {
        return SCIP_OKAY;
    }

    // Print paths.
// #ifdef PRINT_DEBUG
//     print_used_paths(scip);
// #endif

    // Get the edges used by each agent.
    const auto& positive_move_edges = SCIPprobdataGetPositiveMoveEdges(probdata);

    // Get variables.
    const auto& agent_vars = SCIPprobdataGetAgentVars(probdata);

    // Find conflicts.
    Vector<EdgeTime> rectangle_edges;
    Int a1_out_edges_begin;
    Int a2_in_edges_begin;
    Int a2_out_edges_begin;
#if defined(DEBUG) or defined(PRINT_DEBUG)
    Position a1_start_x;
    Position a1_start_y;
    Time a1_start_t;
    Position a2_start_x;
    Position a2_start_y;
    Time a2_start_t;
    Position a1_end_x;
    Position a1_end_y;
    Time a1_end_t;
    Position a2_end_x;
    Position a2_end_y;
    Time a2_end_t;
    SCIP_Real lhs;
#endif
    for (Agent a1 = 0; a1 < N - 1; ++a1)
        for (auto it1 = agent_vars[a1].crbegin(); it1 != agent_vars[a1].crend(); ++it1)
        {
            const auto& [a1_var, a1_var_val] = *it1;
            if (SCIPisPositive(scip, a1_var_val))
            {
                // Get the path of agent 1.
                debug_assert(a1_var);
                const auto a1_vardata = SCIPvarGetData(a1_var);
                const auto a1_path_length = SCIPvardataGetPathLength(a1_vardata);
                const auto a1_path = SCIPvardataGetPath(a1_vardata);

                // Loop through the second agent.
                for (Agent a2 = a1 + 1; a2 < N; ++a2)
                {
                    Int count = 0;
                    for (auto it2 = agent_vars[a2].crbegin(); it2 != agent_vars[a2].crend(); ++it2)
                    {
                        const auto& [a2_var, a2_var_val] = *it2;
                        if (SCIPisPositive(scip, a2_var_val))
                        {
                            // Get the path of agent 2.
                            debug_assert(a2_var);
                            const auto a2_vardata = SCIPvarGetData(a2_var);
                            const auto a2_path_length = SCIPvardataGetPathLength(a2_vardata);
                            const auto a2_path = SCIPvardataGetPath(a2_vardata);

                            // Find a vertex conflict and search outward to find a rectangle conflict.
                            const auto min_path_length = std::min(a1_path_length, a2_path_length);
                            for (Time conflict_time = 1; conflict_time < min_path_length - 1; ++conflict_time)
                                if (a1_path[conflict_time].n == a2_path[conflict_time].n &&
                                    find_rectangle_conflict(scip,
                                                                map,
                                                                positive_move_edges[a1],
                                                                positive_move_edges[a2],
                                                                conflict_time,
                                                                a1_path,
                                                                a2_path,
                                                                min_path_length,
                                                                rectangle_edges,
                                                                a1_out_edges_begin,
                                                                a2_in_edges_begin,
                                                                a2_out_edges_begin
#if defined(DEBUG) or defined(PRINT_DEBUG)
                                                              , a1,
                                                                a2,
                                                                a1_start_x, a1_start_y, a1_start_t,
                                                                a2_start_x, a2_start_y, a2_start_t,
                                                                a1_end_x, a1_end_y, a1_end_t,
                                                                a2_end_x, a2_end_y, a2_end_t,
                                                                lhs
#endif
                                                               ))
                                {
                                    // Print.
#ifdef PRINT_DEBUG
                                    {
                                        const auto& map = SCIPprobdataGetMap(probdata);
                                        String a1_in_str;
                                        String a1_out_str;
                                        String a2_in_str;
                                        String a2_out_str;
                                        {
                                            Int idx = 0;
                                            for (; idx < a1_out_edges_begin; ++idx)
                                            {
                                                const auto& [e, t] = rectangle_edges[idx].et;
                                                const auto [x1, y1] = map.get_xy(e.n);
                                                const auto [x2, y2] = map.get_destination_xy(e);
                                                a1_in_str += fmt::format("(({},{}),({},{}),{}) ", x1, y1, x2, y2, t);
                                            }
                                            a1_in_str.pop_back();
                                            for (; idx < a2_in_edges_begin; ++idx)
                                            {
                                                const auto& [e, t] = rectangle_edges[idx].et;
                                                const auto [x1, y1] = map.get_xy(e.n);
                                                const auto [x2, y2] = map.get_destination_xy(e);
                                                a1_out_str += fmt::format("(({},{}),({},{}),{}) ", x1, y1, x2, y2, t);
                                            }
                                            a1_out_str.pop_back();
                                            for (; idx < a2_out_edges_begin; ++idx)
                                            {
                                                const auto& [e, t] = rectangle_edges[idx].et;
                                                const auto [x1, y1] = map.get_xy(e.n);
                                                const auto [x2, y2] = map.get_destination_xy(e);
                                                a2_in_str += fmt::format("(({},{}),({},{}),{}) ", x1, y1, x2, y2, t);
                                            }
                                            a2_in_str.pop_back();
                                            for (; idx < static_cast<Int>(rectangle_edges.size()); ++idx)
                                            {
                                                const auto& [e, t] = rectangle_edges[idx].et;
                                                const auto [x1, y1] = map.get_xy(e.n);
                                                const auto [x2, y2] = map.get_destination_xy(e);
                                                a2_out_str += fmt::format("(({},{}),({},{}),{}) ", x1, y1, x2, y2, t);
                                            }
                                            a2_out_str.pop_back();
                                        }
                                        println("    Creating rectangle knapsack cut for agents {} and {} with "
                                                "value {} in branch-and-bound node {}:",
                                                a1,
                                                a2,
                                                lhs,
                                                SCIPnodeGetNumber(SCIPgetCurrentNode(scip)));
                                        println("      Agent {} in: {}", a1, a1_in_str);
                                        println("      Agent {} out: {}", a1, a1_out_str);
                                        println("      Agent {} in: {}", a2, a2_in_str);
                                        println("      Agent {} out: {}", a2, a2_out_str);
                                    }
#endif

                                    // Create cut.
                                    SCIP_CALL(rectangle_knapsack_conflicts_create_cut(scip,
                                                                                      probdata,
                                                                                      sepa,
                                                                                      *sepadata,
                                                                                      a1,
                                                                                      a2,
                                                                                      a1_out_edges_begin,
                                                                                      a2_in_edges_begin,
                                                                                      a2_out_edges_begin,
                                                                                      rectangle_edges,
#if defined(DEBUG) or defined(PRINT_DEBUG)
                                                                                      a1_start_x,
                                                                                      a1_start_y,
                                                                                      a1_start_t,
                                                                                      a1_end_x,
                                                                                      a1_end_y,
                                                                                      a1_end_t,
                                                                                      a2_start_x,
                                                                                      a2_start_y,
                                                                                      a2_start_t,
                                                                                      a2_end_x,
                                                                                      a2_end_y,
                                                                                      a2_end_t,
#endif
                                                                                      result));
                                    found_cuts = true;
                                    goto NEXT_AGENT_PAIR;
                                }

                            // Stop if checked enough paths.
                            ++count;
                            if (count >= 50)
                            {
                                goto NEXT_AGENT_PAIR;
                            }
                        }
                    }
                    NEXT_AGENT_PAIR:;
                }
            }
        }

    // Done.
    return SCIP_OKAY;
}

// Copy method for separator
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_SEPACOPY(sepaCopyRectangleKnapsackConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);

    // Include separator.
    SCIP_SEPA* sepa_copy;
    SCIP_CALL(SCIPincludeSepaRectangleKnapsackConflicts(scip, &sepa_copy));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Free separator data
static
SCIP_DECL_SEPAFREE(sepaFreeRectangleKnapsackConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);

    // Get separator data.
    auto sepadata = reinterpret_cast<RectangleKnapsackSepaData*>(SCIPsepaGetData(sepa));
    debug_assert(sepadata);

    // Free memory.
    sepadata->~RectangleKnapsackSepaData();
    SCIPfreeBlockMemory(scip, &sepadata);

    // Done.
    return SCIP_OKAY;
}

// Separation method for LP solutions
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_SEPAEXECLP(sepaExeclpRectangleKnapsackConflicts)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);
    debug_assert(strcmp(SCIPsepaGetName(sepa), SEPA_NAME) == 0);

    // Start.
    *result = SCIP_DIDNOTFIND;

    // Start separator.
    SCIP_CALL(rectangle_knapsack_conflicts_separate(scip, sepa, result));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Creates separator for rectangle knapsack conflicts constraints and include it in SCIP
SCIP_RETCODE SCIPincludeSepaRectangleKnapsackConflicts(
    SCIP* scip,         // SCIP
    SCIP_SEPA** sepa    // Output pointer to separator
)
{
    // Check.
    debug_assert(scip);
    debug_assert(sepa);

    // Create separator data.
    RectangleKnapsackSepaData* sepadata = nullptr;
    SCIP_CALL(SCIPallocBlockMemory(scip, &sepadata));
    debug_assert(sepadata);
    new(sepadata) RectangleKnapsackSepaData;
    sepadata->cuts.reserve(500);

    // Include separator.
    SCIP_CALL(SCIPincludeSepaBasic(scip,
                                   sepa,
                                   SEPA_NAME,
                                   SEPA_DESC,
                                   SEPA_PRIORITY,
                                   SEPA_FREQ,
                                   SEPA_MAXBOUNDDIST,
                                   SEPA_USESSUBSCIP,
                                   SEPA_DELAY,
                                   sepaExeclpRectangleKnapsackConflicts,
                                   nullptr,
                                   reinterpret_cast<SCIP_SEPADATA*>(sepadata)));
    debug_assert(*sepa);

    // Set callbacks.
    SCIP_CALL(SCIPsetSepaCopy(scip, *sepa, sepaCopyRectangleKnapsackConflicts));
    SCIP_CALL(SCIPsetSepaFree(scip, *sepa, sepaFreeRectangleKnapsackConflicts));

    // Done.
    return SCIP_OKAY;
}

// Get additional data about rectangle knapsack cuts
const Vector<RectangleKnapsackCut>& rectangle_knapsack_get_cuts(
    SCIP_ProbData* probdata    // Problem data
)
{
    auto sepa = SCIPprobdataGetRectangleKnapsackConflictsSepa(probdata);
    debug_assert(sepa);
    auto sepadata = reinterpret_cast<RectangleKnapsackSepaData*>(SCIPsepaGetData(sepa));
    debug_assert(sepadata);
    return sepadata->cuts;
}

#endif
