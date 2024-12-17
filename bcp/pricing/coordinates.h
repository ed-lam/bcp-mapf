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

#ifndef TRUFFLEHOG_COORDINATES_H
#define TRUFFLEHOG_COORDINATES_H

#include "problem/includes.h"

namespace TruffleHog
{

using Node = Int;
using Time = Int;
enum Direction: uint8_t
{
    NORTH = 0,
    SOUTH = 1,
    EAST = 2,
    WEST = 3,
    WAIT = 4,
    INVALID = 5
};

union Edge
{
    struct
    {
        Node n: 29;
        Direction d: 3;
    };
    uint32_t id;

    Edge() noexcept = default;
    explicit Edge(const Node n, const Direction d) noexcept : n(n), d(d) {}
};
static_assert(sizeof(Edge) == 4);
static_assert(std::is_trivial<Edge>::value);
inline bool operator==(const Edge a, const Edge b)
{
    return a.id == b.id;
}
inline bool operator!=(const Edge a, const Edge b)
{
    return !(a == b);
}

union NodeTime
{
    struct
    {
        Node n;
        Time t;
    };
    uint64_t nt;

    NodeTime() noexcept = default;
    NodeTime(const uint64_t nt) noexcept : nt(nt) {}
    explicit NodeTime(const Node n, const Time t) noexcept : n(n), t(t) {}
};
static_assert(sizeof(NodeTime) == 8);
static_assert(std::is_trivial<NodeTime>::value);
inline bool operator==(const NodeTime a, const NodeTime b)
{
    return a.nt == b.nt;
}
inline bool operator!=(const NodeTime a, const NodeTime b)
{
    return a.nt != b.nt;
}

union EdgeTime
{
    struct
    {
        Edge e;
        Time t;
    } et;
    struct
    {
        Node n : 29;
        Direction d : 3;
        Time t;
    };
    uint64_t id;

    EdgeTime() noexcept = default;
    explicit EdgeTime(const Edge e, const Time t) noexcept : et{e, t} {}
    explicit EdgeTime(const NodeTime nt, const Direction d) noexcept : n{nt.n}, d{d}, t{nt.t} {}
    explicit EdgeTime(const Node n, const Direction d, const Time t) noexcept : n{n}, d{d}, t{t} {}

    inline NodeTime nt() const noexcept { return NodeTime{n, t}; }
};
static_assert(sizeof(EdgeTime) == 8);
static_assert(std::is_trivial<EdgeTime>::value);
inline bool operator==(const EdgeTime a, const EdgeTime b)
{
    return a.id == b.id;
}
inline bool operator!=(const EdgeTime a, const EdgeTime b)
{
    return !(a == b);
}

}

union AgentTime
{
    struct
    {
        Agent a{-1};
        Time t{0};
    };
    uint64_t id;
};
static_assert(sizeof(AgentTime) == 8);
static_assert(std::is_trivially_copyable<AgentTime>::value);
inline bool operator==(const AgentTime a, const AgentTime b)
{
    return a.a == b.a && a.t == b.t; // TODO
}
inline bool operator!=(const AgentTime a, const AgentTime b)
{
    return !(a == b);
}

struct AgentNodeTime
{
    Agent a{-1};
    Node n{0};
    Time t{0};
};
static_assert(sizeof(AgentNodeTime) == 12);
static_assert(std::is_trivially_copyable<AgentNodeTime>::value);
inline bool operator==(const AgentNodeTime a, const AgentNodeTime b)
{
    return a.a == b.a && a.n == b.n && a.t == b.t;
}
inline bool operator!=(const AgentNodeTime a, const AgentNodeTime b)
{
    return !(a == b);
}

struct AgentEdgeTime
{
    Agent a{-1};
    Edge e;
    Time t{0};
};
static_assert(sizeof(AgentEdgeTime) == 12);
static_assert(std::is_trivially_copyable<AgentEdgeTime>::value);
inline bool operator==(const AgentEdgeTime a, const AgentEdgeTime b)
{
    return a.a == b.a && a.e == b.e && a.t == b.t;
}
inline bool operator!=(const AgentEdgeTime a, const AgentEdgeTime b)
{
    return !(a == b);
}

template<class T>
inline void hash_combine(std::size_t& s, const T& v)
{
    robin_hood::hash<T> h;
    s ^= h(v) + 0x9e3779b9 + (s << 6) + (s >> 2);
}

namespace robin_hood
{

template<>
struct hash<AgentTime>
{
    inline std::size_t operator()(const AgentTime at) const noexcept
    {
        return robin_hood::hash<uint64_t>{}(at.id);
    }
};

template<>
struct hash<AgentNodeTime>
{
    inline std::size_t operator()(const AgentNodeTime ant) const noexcept
    {
        auto x = robin_hood::hash<Agent>{}(ant.a);
        hash_combine(x, ant.n);
        hash_combine(x, ant.t);
        return x;
    }
};

template<>
struct hash<AgentEdgeTime>
{
    inline std::size_t operator()(const AgentEdgeTime ant) const noexcept
    {
        auto x = robin_hood::hash<Agent>{}(ant.a);
        hash_combine(x, ant.e);
        hash_combine(x, ant.t);
        return x;
    }
};

}

namespace fmt
{

template<>
struct formatter<AgentNodeTime>
{
    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx) { return ctx.begin(); }

    template<typename FormatContext>
    inline auto format(const AgentNodeTime& ant, FormatContext& ctx)
    {
        return format_to(ctx.out(), "(a={},n={},t={})", ant.a, ant.n, ant.t);
    }
};

}

namespace robin_hood
{

template<>
struct hash<TruffleHog::Edge>
{
    inline std::size_t operator()(const TruffleHog::Edge e) const noexcept
    {
        return robin_hood::hash<uint32_t>{}(e.id);
    }
};

template<>
struct hash<TruffleHog::NodeTime>
{
    inline std::size_t operator()(const TruffleHog::NodeTime nt) const noexcept
    {
        return robin_hood::hash<uint64_t>{}(nt.nt);
    }
};

template<>
struct hash<TruffleHog::EdgeTime>
{
    inline std::size_t operator()(const TruffleHog::EdgeTime et) const noexcept
    {
        return robin_hood::hash<uint64_t>{}(et.id);
    }
};

}

template<>
struct fmt::formatter<TruffleHog::Direction> : formatter<std::string_view>
{
    auto format(TruffleHog::Direction d, fmt::format_context& ctx) const
    {
        std::string_view name = "INVALID";
        switch (d)
        {
            case TruffleHog::Direction::NORTH: name = "NORTH"; break;
            case TruffleHog::Direction::SOUTH: name = "SOUTH"; break;
            case TruffleHog::Direction::EAST:  name = "EAST";  break;
            case TruffleHog::Direction::WEST:  name = "WEST";  break;
            case TruffleHog::Direction::WAIT:  name = "WAIT";  break;
            default: break;
        }
        return fmt::formatter<string_view>::format(name, ctx);
    }
};

template<>
struct fmt::formatter<TruffleHog::Edge> : formatter<std::string_view>
{
    auto format(const TruffleHog::Edge e, fmt::format_context& ctx) const
    {
        return fmt::format_to(ctx.out(), "(n={},d={})", e.n, e.d);
    }
};

template<>
struct fmt::formatter<TruffleHog::NodeTime> : formatter<std::string_view>
{
    auto format(const TruffleHog::NodeTime nt, fmt::format_context& ctx) const
    {
        return fmt::format_to(ctx.out(), "(n={},t={})", nt.n, nt.t);
    }
};

template<>
struct fmt::formatter<TruffleHog::EdgeTime> : formatter<std::string_view>
{
    auto format(const TruffleHog::EdgeTime et, fmt::format_context& ctx) const
    {
        return fmt::format_to(ctx.out(), "(n={},d={},t={})", et.n, et.d, et.t);
    }
};

//template<class T>
//inline void hash_combine(std::size_t& s, const T& v)
//{
//    s ^= robin_hood::hash<T>(v) + 0x9e3779b9 + (s << 6) + (s >> 2);
//}

#endif
