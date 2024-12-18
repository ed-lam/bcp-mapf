#pragma once

#include "types/basic_types.h"

using Position = Int16;

union XY
{
    struct
    {
        Position x;
        Position y;
    };
    UInt32 xy;

    constexpr XY() noexcept = default;
    constexpr XY(const UInt32 xy) noexcept : xy(xy) {}
    constexpr XY(const Position x, const Position y) noexcept : x(x), y(y) {}

    // auto operator<=>(const XY& rhs) const { return xy <=> rhs.xy; }
};
static_assert(sizeof(Position) == 2);
static_assert(sizeof(XY) == 4);
static_assert(std::has_unique_object_representations_v<XY>);
static_assert(std::is_trivial<XY>::value);
inline bool operator==(const XY lhs, const XY rhs)
{
    return lhs.xy == rhs.xy;
}
inline bool operator!=(const XY lhs, const XY rhs)
{
    return !(lhs == rhs);
}

union XYT
{
    struct
    {
        Position x;
        Position y;
        Time t;
    };
    UInt64 xyt;

    constexpr XYT() noexcept = default;
    constexpr XYT(const UInt64 xyt) noexcept : xyt(xyt) {}
    constexpr XYT(const Position x, const Position y, const Time t) noexcept : x(x), y(y), t(t) {}
};
static_assert(sizeof(Position) == 2);
static_assert(sizeof(Time) == 4);
static_assert(sizeof(XYT) == 8);
static_assert(std::has_unique_object_representations_v<XYT>);
static_assert(std::is_trivial<XYT>::value);
inline bool operator==(const XYT lhs, const XYT rhs)
{
    return lhs.xyt == rhs.xyt;
}
inline bool operator!=(const XYT lhs, const XYT rhs)
{
    return !(lhs == rhs);
}

using Node = Int32;

union NodeTime
{
    struct
    {
        Node n;
        Time t;
    };
    UInt64 nt;

    constexpr NodeTime() noexcept = default;
    constexpr NodeTime(const UInt64 nt) noexcept : nt(nt) {}
    constexpr NodeTime(const Node n, const Time t) noexcept : n(n), t(t) {}
};
static_assert(sizeof(Node) == 4);
static_assert(sizeof(Time) == 4);
static_assert(sizeof(NodeTime) == 8);
static_assert(std::has_unique_object_representations_v<NodeTime>);
static_assert(std::is_trivial<NodeTime>::value);
inline bool operator==(const NodeTime lhs, const NodeTime rhs)
{
    return lhs.nt == rhs.nt;
}
inline bool operator!=(const NodeTime lhs, const NodeTime rhs)
{
    return !(lhs == rhs);
}

enum Direction : uint8_t
{
    NORTH = 0,  // 0000
    SOUTH = 1,  // 0001
    WEST = 2,   // 0010
    EAST = 3,   // 0011
    WAIT = 4,   // 0100
    INVALID = 5 // 0101
};
// inline Direction& operator++(Direction& d)
// {
//     d = static_cast<Direction>(static_cast<std::underlying_type<Direction>::type>(d) + 1);
//     return d;
// }

union Edge
{
    struct
    {
        Node n : 29;
        Direction d : 3;
    };
    UInt32 id;

    constexpr Edge() noexcept = default;
    constexpr Edge(const Node n, const Direction d) noexcept : n(n), d(d) {}
};
static_assert(sizeof(Node) == 4);
static_assert(sizeof(Edge) == 4);
static_assert(std::has_unique_object_representations_v<Edge>);
static_assert(std::is_trivial<Edge>::value);
inline bool operator==(const Edge lhs, const Edge rhs)
{
    return lhs.id == rhs.id;
}
inline bool operator!=(const Edge lhs, const Edge rhs)
{
    return !(lhs == rhs);
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
    UInt64 id;

    constexpr EdgeTime() noexcept = default;
    constexpr EdgeTime(const Edge e, const Time t) noexcept : et{e, t} {}
    constexpr EdgeTime(const NodeTime nt, const Direction d) noexcept : n(nt.n), d(d), t(nt.t) {}
    constexpr EdgeTime(const Node n, const Direction d, const Time t) noexcept : n(n), d(d), t(t) {}

    inline NodeTime nt() const noexcept { return NodeTime{n, t}; }
};
static_assert(sizeof(EdgeTime) == 8);
static_assert(std::has_unique_object_representations_v<EdgeTime>);
static_assert(std::is_trivial<EdgeTime>::value);
inline bool operator==(const EdgeTime lhs, const EdgeTime rhs)
{
    return lhs.id == rhs.id;
}
inline bool operator!=(const EdgeTime lhs, const EdgeTime rhs)
{
    return !(lhs == rhs);
}

union AgentTime
{
    struct
    {
        Int32 a{-1};
        Time t{0};
    };
    UInt64 id;
};
static_assert(sizeof(AgentTime) == 8);
static_assert(std::has_unique_object_representations_v<AgentTime>);
inline Bool operator==(const AgentTime lhs, const AgentTime rhs)
{
    return lhs.id == rhs.id;
}
inline Bool operator!=(const AgentTime lhs, const AgentTime rhs)
{
    return !(lhs == rhs);
}

struct AgentNodeTime
{
    Int32 a{-1};
    Node n{0};
    Time t{0};
};
static_assert(sizeof(AgentNodeTime) == 12);
static_assert(std::has_unique_object_representations_v<AgentNodeTime>);
inline bool operator==(const AgentNodeTime lhs, const AgentNodeTime rhs)
{
    return lhs.a == rhs.a && lhs.n == rhs.n && lhs.t == rhs.t;
}
inline bool operator!=(const AgentNodeTime lhs, const AgentNodeTime rhs)
{
    return !(lhs == rhs);
}

struct AgentEdgeTime
{
    Int32 a{-1};
    Edge e;
    Time t{0};
};
static_assert(sizeof(AgentEdgeTime) == 12);
// static_assert(std::has_unique_object_representations_v<AgentEdgeTime>);
inline bool operator==(const AgentEdgeTime lhs, const AgentEdgeTime rhs)
{
    return lhs.a == rhs.a && lhs.e == rhs.e && lhs.t == rhs.t;
}
inline bool operator!=(const AgentEdgeTime lhs, const AgentEdgeTime rhs)
{
    return !(lhs == rhs);
}
