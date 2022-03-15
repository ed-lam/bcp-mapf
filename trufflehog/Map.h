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

#ifndef TRUFFLEHOG_MAP_H
#define TRUFFLEHOG_MAP_H

#include "Includes.h"
#include "Coordinates.h"

namespace TruffleHog
{

class Map
{
    Vector<bool> passable_;  // Row-major matrix
    Vector<Time> latest_visit_time_;
    Position width_ = 0;
    Position height_ = 0;

  public:
    // Constructors
    Map() = default;
    ~Map() = default;
    Map(const Map&) = default;
    Map(Map&&) = default;
    Map& operator=(const Map&) = default;
    Map& operator=(Map&&) = default;

    // Getters
    inline Node size() const
    {
        return passable_.size();
    }
    inline bool empty() const
    {
        return !size();
    }
    inline Position width() const
    {
        return width_;
    }
    inline Position height() const
    {
        return height_;
    }
    inline bool operator[](const Node n) const
    {
        debug_assert(n < size());
        return passable_[n];
    }
    inline const Vector<Time>& latest_visit_time() const { return latest_visit_time_; }
    inline Node get_id(const Position x, const Position y) const
    {
        return y * width_ + x;
    }
    inline Position get_x(const Node n) const
    {
        return n % width_;
    }
    inline Position get_y(const Node n) const
    {
        return n / width_;
    }
    inline Pair<Position, Position> get_xy(const Node n) const
    {
        return {get_x(n), get_y(n)};
    }
    inline Node get_north(const Node n) const
    {
        return n - width_;
    }
    inline Node get_south(const Node n) const
    {
        return n + width_;
    }
    inline Node get_east(const Node n) const
    {
        return n + 1;
    }
    inline Node get_west(const Node n) const
    {
        return n - 1;
    }
    inline Node get_wait(const Node n) const
    {
        return n;
    }
    inline Direction get_direction(const Node n1, const Node n2) const
    {
        // Check.
        debug_assert(n2 == get_north(n1) ||
                     n2 == get_south(n1) ||
                     n2 == get_east(n1) ||
                     n2 == get_west(n1) ||
                     n2 == get_wait(n1));

        // Return direction.
        if (n2 == get_north(n1)) { return Direction::NORTH; }
        else if (n2 == get_south(n1)) { return Direction::SOUTH; }
        else if (n2 == get_east(n1)) { return Direction::EAST; }
        else if (n2 == get_west(n1)) { return Direction::WEST; }
        else { return Direction::WAIT; }
    }
    inline Node get_destination(const Node n, const Direction d) const
    {
        debug_assert(d == Direction::NORTH ||
                     d == Direction::SOUTH ||
                     d == Direction::EAST ||
                     d == Direction::WEST ||
                     d == Direction::WAIT);
        return n +
               width_ * (static_cast<Node>(d == Direction::SOUTH) - static_cast<Node>(d == Direction::NORTH)) +
                        (static_cast<Node>(d == Direction::EAST)  - static_cast<Node>(d == Direction::WEST));
    }
    inline Node get_destination(const Edge e) const
    {
        return get_destination(e.n, e.d);
    }
    inline Node get_destination(const EdgeTime et) const
    {
        return get_destination(et.n, et.d);
    }
    inline Pair<Position, Position> get_destination_xy(const Edge e) const
    {
        return get_xy(get_destination(e));
    }
    inline Pair<Position, Position> get_destination_xy(const EdgeTime et) const
    {
        return get_xy(get_destination(et));
    }
    inline Edge get_undirected_edge(const Edge e) const
    {
        switch (e.d)
        {
            case (Direction::WEST): { return Edge{get_west(e.n), Direction::EAST}; }
            case (Direction::SOUTH): { return Edge{get_south(e.n), Direction::NORTH}; }
            default: { return e; }
        }
    }
    inline Edge get_opposite_edge(const Edge e) const
    {
        switch (e.d)
        {
            case (Direction::NORTH): { return Edge{get_north(e.n), Direction::SOUTH}; }
            case (Direction::SOUTH): { return Edge{get_south(e.n), Direction::NORTH}; }
            case (Direction::EAST): { return Edge{get_east(e.n), Direction::WEST}; }
            case (Direction::WEST): { return Edge{get_west(e.n), Direction::EAST}; }
            default: { return e; }
        }
    }

    // Setters
    void resize(const Position width, const Position height)
    {
        debug_assert(empty());
        passable_.resize(width * height, false);
        latest_visit_time_.resize(width * height, -1);
        width_ = width;
        height_ = height;
    }
    void set_passable(const Node n)
    {
        debug_assert(n < size());
        passable_[n] = true;
        latest_visit_time_[n] = std::numeric_limits<Time>::max();
    }
    void set_obstacle(const Node n)
    {
        debug_assert(n < size());
        passable_[n] = false;
        latest_visit_time_[n] = -1;
    }

    // Debug
    void print() const
    {
        for (Node n = 0; n < size(); ++n)
        {
            if (n % width() == 0)
                fmt::print("\n");
            fmt::print("{}", (*this)[n] ? '.' : '@');
        }
        fmt::print("\n");
        fflush(stdout);
    }
};

}
#endif
