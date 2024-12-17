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

// #if defined(USE_RECTANGLE_KNAPSACK_CONFLICTS) || defined(USE_RECTANGLE_CLIQUE_CONFLICTS)
//
// // #define PRINT_DEBUG
//
// #include "Separator_RectangleConflicts.h"
// #include "ProblemData.h"
// #include "VariableData.h"
// #include <algorithm>
//
// template<class T>
// inline T signum(const T val)
// {
//     return (T(0) < val) - (val < T(0));
// }
//
// inline void append_edge(
//     const Map& map,            // Map
//     const Time t,              // Time of the edge
//     const Position x,          // Coordinate
//     const Position y,          // Coordinate
//     const Direction d,         // Direction
//     Vector<EdgeTime>& edges    // Output edges of the rectangle
// )
// {
//     edges.emplace_back(map.get_id(x, y), d, t);
//     //    debugln("         vertex ({},{}), time {}, direction {}",
//     //            x, y, t, d);
// }
//
// inline bool append_vertical_boundary(
//     const Map& map,            // Map
//     const Time t0,             // Time at reference location
//     const Position x0,         // Coordinate of reference location
//     const Position y0,         // Coordinate of reference location
//     const Direction d,         // Direction
//     const Position y1,         // Start coordinate of the vertical boundary
//     const Position y2,         // End coordinate of the vertical boundary
//     Vector<EdgeTime>& edges    // Output edges of the rectangle
// )
// {
//     // Calculate the direction the agent is moving.
//     const auto y_direction = signum(y2 - y1);
//     if (y_direction == 0)
//         return false;
//
//     // Add the edges crossing into the boundary of the rectangle.
//     //    debugln("      Edges of vertical boundary:");
//     for (Position y = y1; y != y2 + y_direction; y += y_direction)
//     {
//         const auto t = t0 + y_direction * (y - y0);
//         if (0 <= t &&
//             0 <= x0 && x0 < map.width() &&
//             0 <= y && y < map.height())
//         {
//             append_edge(map, t, x0, y, d, edges);
//         }
//     }
//     return true;
// }
//
// inline bool append_horizontal_boundary(
//     const Map& map,            // Map
//     const Time t0,             // Time at reference location
//     const Position x0,         // Coordinate of reference location
//     const Position y0,         // Coordinate of reference location
//     const Direction d,         // Direction
//     const Position x1,         // Start coordinate of the horizontal boundary
//     const Position x2,         // End coordinate of the horizontal boundary
//     Vector<EdgeTime>& edges    // Output edges of the rectangle
// )
// {
//     // Calculate the direction the agent is moving.
//     const auto x_direction = signum(x2 - x1);
//     if (x_direction == 0)
//         return false;
//
//     // Add the edges crossing into the boundary of the rectangle.
//     //    debugln("      Edges of horizontal boundary:");
//     for (Position x = x1; x != x2 + x_direction; x += x_direction)
//     {
//         const auto t = t0 + x_direction * (x - x0);
//         if (0 <= t &&
//             0 <= x && x < map.width() &&
//             0 <= y0 && y0 < map.height())
//         {
//             append_edge(map, t, x, y0, d, edges);
//         }
//     }
//     return true;
// }
//
// static
// Pair<Direction, Direction> compute_direction(
//     const Position start_x1,    // Start coordinate of agent 1
//     const Position start_y1,    // Start coordinate of agent 1
//     const Position start_x2,    // Start coordinate of agent 2
//     const Position start_y2,    // Start coordinate of agent 2
//     const Position end_x1,      // End coordinate of agent 1
//     const Position end_y1,      // End coordinate of agent 1
//     const Position end_x2,      // End coordinate of agent 2
//     const Position end_y2       // End coordinate of agent 2
// )
// {
//     // Calculate direction of the two agents.
//     Direction d1 = Direction::INVALID;
//     Direction d2 = Direction::INVALID;
//
//     // Going north-east
//     if (start_x1 <= start_x2 && start_y1 <= start_y2 &&
//         end_x1   >= end_x2   && end_y1   >= end_y2   &&
//         start_x1 <= end_x1   && start_y1 >= end_y1   &&
//         start_x2 <= end_x2   && start_y2 >= end_y2)
//     {
//         release_assert(d1 == Direction::INVALID);
//         d1 = Direction::EAST;
//         d2 = Direction::NORTH;
//     }
//     if (start_x2 <= start_x1 && start_y2 <= start_y1 &&
//         end_x2   >= end_x1   && end_y2   >= end_y1   &&
//         start_x2 <= end_x2   && start_y2 >= end_y2   &&
//         start_x1 <= end_x1   && start_y1 >= end_y1)
//     {
//         release_assert(d1 == Direction::INVALID);
//         d1 = Direction::NORTH;
//         d2 = Direction::EAST;
//     }
//
//     // Going north-west
//     if (start_x1 >= start_x2 && start_y1 <= start_y2 &&
//         end_x1   <= end_x2   && end_y1   >= end_y2   &&
//         start_x1 >= end_x1   && start_y1 >= end_y1   &&
//         start_x2 >= end_x2   && start_y2 >= end_y2)
//     {
//         release_assert(d1 == Direction::INVALID);
//         d1 = Direction::WEST;
//         d2 = Direction::NORTH;
//     }
//     if (start_x2 >= start_x1 && start_y2 <= start_y1 &&
//         end_x2   <= end_x1   && end_y2   >= end_y1   &&
//         start_x2 >= end_x2   && start_y2 >= end_y2   &&
//         start_x1 >= end_x1   && start_y1 >= end_y1)
//     {
//         release_assert(d1 == Direction::INVALID);
//         d1 = Direction::NORTH;
//         d2 = Direction::WEST;
//     }
//
//     // Going south-east
//     if (start_x1 <= start_x2 && start_y1 >= start_y2 &&
//         end_x1   >= end_x2   && end_y1   <= end_y2   &&
//         start_x1 <= end_x1   && start_y1 <= end_y1   &&
//         start_x2 <= end_x2   && start_y2 <= end_y2)
//     {
//         release_assert(d1 == Direction::INVALID);
//         d1 = Direction::EAST;
//         d2 = Direction::SOUTH;
//     }
//     if (start_x2 <= start_x1 && start_y2 >= start_y1 &&
//         end_x2   >= end_x1   && end_y2   <= end_y1 &&
//         start_x2 <= end_x2   && start_y2 <= end_y2 &&
//         start_x1 <= end_x1   && start_y1 <= end_y1)
//     {
//         release_assert(d1 == Direction::INVALID);
//         d1 = Direction::SOUTH;
//         d2 = Direction::EAST;
//     }
//
//     // Going south-west
//     if (start_x1 >= start_x2 && start_y1 >= start_y2 &&
//         end_x1   <= end_x2   && end_y1   <= end_y2 &&
//         start_x1 >= end_x1   && start_y1 <= end_y1 &&
//         start_x2 >= end_x2   && start_y2 <= end_y2)
//     {
//         release_assert(d1 == Direction::INVALID);
//         d1 = Direction::WEST;
//         d2 = Direction::SOUTH;
//     }
//     if (start_x2 >= start_x1 && start_y2 >= start_y1 &&
//         end_x2   <= end_x1   && end_y2   <= end_y1   &&
//         start_x2 >= end_x2   && start_y2 <= end_y2   &&
//         start_x1 >= end_x1   && start_y1 <= end_y1)
//     {
//         release_assert(d1 == Direction::INVALID);
//         d1 = Direction::SOUTH;
//         d2 = Direction::WEST;
//     }
//
//     // Not a rectangle conflict.
//     return {d1, d2};
// }
//
// void compute_rectangle(
//     const Map& map,                // Map
//     const Time start_t,            // Time before entry
//     const Time end_t,              // Time after exit
//     const Position start_x1,       // Start coordinate of agent 1
//     const Position start_y1,       // Start coordinate of agent 1
//     const Position start_x2,       // Start coordinate of agent 2
//     const Position start_y2,       // Start coordinate of agent 2
//     const Position end_x1,         // End coordinate of agent 1
//     const Position end_y1,         // End coordinate of agent 1
//     const Position end_x2,         // End coordinate of agent 2
//     const Position end_y2,         // End coordinate of agent 2
//     RectangleConflict& conflict    // Output rectangle conflict
// )
// {
//     // Check that there is no wait inside the rectangle.
//     release_assert(std::abs(end_x1 - start_x1) + std::abs(end_y1 - start_y1) == end_t - start_t);
//     release_assert(std::abs(end_x2 - start_x2) + std::abs(end_y2 - start_y2) == end_t - start_t);
//
//     // Compute the directions of the two agents.
//     const auto [d1, d2] = compute_direction(start_x1, start_y1, start_x2, start_y2,
//                                             end_x1, end_y1, end_x2, end_y2);
//     release_assert((d1 == Direction::INVALID && d2 == Direction::INVALID) ||
//                    (d1 != Direction::INVALID && d2 != Direction::INVALID));
//     if (d1 == Direction::INVALID)
//         return;
//
//     // Compute exit time.
//     const auto exit_t = end_t - 1;
//
//     // Append edges for agent 1.
//     bool success;
//     if (d1 == Direction::NORTH || d1 == Direction::SOUTH)
//     {
//         const auto x_bound = start_x2 + (d2 == Direction::EAST ? 1 : -1);
//         const auto y_bound = end_y1 + (d1 == Direction::NORTH ? 1 : -1);
//
//         success = append_horizontal_boundary(map, start_t, start_x1, start_y1, d1, x_bound, end_x2, conflict.edges);
//         if (!success)
//         {
//             conflict.edges.clear();
//             return;
//         }
//
//         conflict.out1_begin_ = conflict.edges.size();
//         success = append_horizontal_boundary(map, exit_t, end_x1, y_bound, d1, x_bound, end_x2, conflict.edges);
//         if (!success)
//         {
//             conflict.edges.clear();
//             return;
//         }
//     }
//     if (d1 == Direction::EAST || d1 == Direction::WEST)
//     {
//         const auto y_bound = start_y2 + (d2 == Direction::NORTH ? -1 : 1);
//         const auto x_bound = end_x1 + (d1 == Direction::EAST ? -1 : 1);
//
//         success = append_vertical_boundary(map, start_t, start_x1, start_y1, d1, y_bound, end_y2, conflict.edges);
//         if (!success)
//         {
//             conflict.edges.clear();
//             return;
//         }
//
//         conflict.out1_begin_ = conflict.edges.size();
//         success = append_vertical_boundary(map, exit_t, x_bound, end_y1, d1, y_bound, end_y2, conflict.edges);
//         if (!success)
//         {
//             conflict.edges.clear();
//             return;
//         }
//     }
//
//     // Append edges for agent 2.
//     if (d2 == Direction::NORTH || d2 == Direction::SOUTH)
//     {
//         const auto x_bound = start_x1 + (d1 == Direction::EAST ? 1 : -1);
//         const auto y_bound = end_y2 + (d2 == Direction::NORTH ? 1 : -1);
//
//         conflict.in2_begin_ = conflict.edges.size();
//         success = append_horizontal_boundary(map, start_t, start_x2, start_y2, d2, x_bound, end_x1, conflict.edges);
//         if (!success)
//         {
//             conflict.edges.clear();
//             return;
//         }
//
//         conflict.out2_begin_ = conflict.edges.size();
//         success = append_horizontal_boundary(map, exit_t, end_x2, y_bound, d2, x_bound, end_x1, conflict.edges);
//         if (!success)
//         {
//             conflict.edges.clear();
//             return;
//         }
//     }
//     if (d2 == Direction::EAST || d2 == Direction::WEST)
//     {
//         const auto y_bound = start_y1 + (d1 == Direction::NORTH ? -1 : 1);
//         const auto x_bound = end_x2 + (d2 == Direction::EAST ? -1 : 1);
//
//         conflict.in2_begin_ = conflict.edges.size();
//         success = append_vertical_boundary(map, start_t, start_x2, start_y2, d2, y_bound, end_y1, conflict.edges);
//         if (!success)
//         {
//             conflict.edges.clear();
//             return;
//         }
//
//         conflict.out2_begin_ = conflict.edges.size();
//         success = append_vertical_boundary(map, exit_t, x_bound, end_y2, d2, y_bound, end_y1, conflict.edges);
//         if (!success)
//         {
//             conflict.edges.clear();
//             return;
//         }
//     }
// }
//
// #endif
