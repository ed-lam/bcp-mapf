#pragma once

#include "problem/debug.h"
#include "types/file_system.h"
#include "types/tuple.h"
#include "types/map_types.h"
#include "types/vector.h"

class Map
{
    Vector<Bool> passable_;    // Row-major matrix
    Position width_ = 0;
    Position height_ = 0;

  public:
    // Constructors and destructor
    Map() = default;
    ~Map() = default;
    Map(const Map&) = default;
    Map(Map&&) = default;
    Map& operator=(const Map&) = default;
    Map& operator=(Map&&) = default;

    // Custom constructor
    Map(const FilePath& map_path);

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
        debug_assert(0 <= n && n < size());
        return passable_[n];
    }
    inline Node get_n(const Position x, const Position y) const
    {
        return y * width_ + x;
    }
    inline Node get_n(const Pair<Position, Position> xy) const
    {
        return get_n(xy.first, xy.second);
    }
    inline Node get_n(const XY xy) const
    {
        return get_n(xy.x, xy.y);
    }
    inline Position get_x(const Node n) const
    {
        // debug_assert(0 <= n && n < size());
        return n % width_;
    }
    inline Position get_y(const Node n) const
    {
        // debug_assert(0 <= n && n < size());
        return n / width_;
    }
    inline Pair<Position, Position> get_xy(const Node n) const
    {
        return {get_x(n), get_y(n)};
    }
    inline Node get_north(const Node n) const
    {
        debug_assert(0 <= n && n < size());
        return n - width_;
    }
    inline Node get_south(const Node n) const
    {
        debug_assert(0 <= n && n < size());
        return n + width_;
    }
    inline Node get_west(const Node n) const
    {
        debug_assert(0 <= n && n < size());
        return n - 1;
    }
    inline Node get_east(const Node n) const
    {
        debug_assert(0 <= n && n < size());
        return n + 1;
    }
    inline Node get_wait(const Node n) const
    {
        debug_assert(0 <= n && n < size());
        return n;
    }
    inline Direction get_direction(const Node n1, const Node n2) const
    {
        // Check.
        debug_assert(n2 == get_north(n1) ||
                     n2 == get_south(n1) ||
                     n2 == get_west(n1) ||
                     n2 == get_east(n1) ||
                     n2 == get_wait(n1));

        // Return direction.
        if      (n2 == get_north(n1)) { return Direction::NORTH; }
        else if (n2 == get_south(n1)) { return Direction::SOUTH; }
        else if (n2 == get_west(n1))  { return Direction::WEST; }
        else if (n2 == get_east(n1))  { return Direction::EAST; }
        else                          { return Direction::WAIT; }
    }
    inline Node get_destination(const Node n, const Direction d) const
    {
        debug_assert(d == Direction::NORTH ||
                     d == Direction::SOUTH ||
                     d == Direction::EAST ||
                     d == Direction::WEST ||
                     d == Direction::WAIT);
        const auto dest = n +
               width_ * (static_cast<Node>(d == Direction::SOUTH) - static_cast<Node>(d == Direction::NORTH)) +
                        (static_cast<Node>(d == Direction::EAST)  - static_cast<Node>(d == Direction::WEST));
        debug_assert((d == Direction::NORTH) == (dest == get_north(n)));
        debug_assert((d == Direction::SOUTH) == (dest == get_south(n)));
        debug_assert((d == Direction::WEST) == (dest == get_west(n)));
        debug_assert((d == Direction::EAST) == (dest == get_east(n)));
        debug_assert((d == Direction::WAIT) == (dest == get_wait(n)));
        return dest;
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
            case (Direction::EAST): { return Edge{get_east(e.n), Direction::WEST}; }
            case (Direction::SOUTH): { return Edge{get_south(e.n), Direction::NORTH}; }
            default: { return e; }
        }
    }
    inline Direction get_opposite_direction(const Direction d) const
    {
        switch (d)
        {
            case (Direction::NORTH): { return Direction::SOUTH; }
            case (Direction::SOUTH): { return Direction::NORTH; }
            case (Direction::WEST): { return Direction::EAST; }
            case (Direction::EAST): { return Direction::WEST; }
            default: { return d; }
        }
    }
    inline Edge get_opposite_edge(const Edge e) const
    {
        switch (e.d)
        {
            case (Direction::NORTH): { return Edge{get_north(e.n), Direction::SOUTH}; }
            case (Direction::SOUTH): { return Edge{get_south(e.n), Direction::NORTH}; }
            case (Direction::WEST): { return Edge{get_west(e.n), Direction::EAST}; }
            case (Direction::EAST): { return Edge{get_east(e.n), Direction::WEST}; }
            default: { return e; }
        }
    }

    // Setters
    void set_passable(const Node n);
    void set_obstacle(const Node n);

    // Debug
    void print() const;

  private:
    // Setters
    void resize(const Position width, const Position height);
};
