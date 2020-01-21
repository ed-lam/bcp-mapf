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

    // Setters
    void resize(const Position width, const Position height)
    {
        debug_assert(empty());
        passable_.resize(width * height);
        width_ = width;
        height_ = height;
    }
    void set_passable(const Node n)
    {
        debug_assert(n < size());
        passable_[n] = true;
    }
    void set_obstacle(const Node n)
    {
        debug_assert(n < size());
        passable_[n] = false;
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
