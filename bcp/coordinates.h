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

#ifndef MAPF_COORDINATES_H
#define MAPF_COORDINATES_H

#include "Includes.h"
#include "trufflehog/Coordinates.h"

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

#endif
