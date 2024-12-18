#pragma once

#include "types/map_types.h"
#include <robin_hood.h>

template<class T, class Hash = robin_hood::hash<T>, class KeyEqual = std::equal_to<T>>
using HashSet = robin_hood::unordered_flat_set<T, Hash, KeyEqual>;

template<class Key, class T, class Hash = robin_hood::hash<Key>, class KeyEqual = std::equal_to<Key>>
using HashMap = robin_hood::unordered_flat_map<Key, T, Hash, KeyEqual>;

template<class Key, class Hash = robin_hood::hash<Key>>
inline void hash_combine(std::size_t& s, const Key& v)
{
    s ^= Hash{}(v) + 0x9e3779b9 + (s << 6) + (s >> 2);
}

template<class KeyIterator, class Hash = robin_hood::hash<KeyIterator>>
inline std::size_t hash_range(KeyIterator begin, KeyIterator end)
{
    size_t x = 0;
    for (; begin != end; ++begin)
    {
        hash_combine(x, *begin);
    }
    return x;
}

template<>
struct robin_hood::hash<XYT>
{
    inline std::size_t operator()(const XYT xyt) const noexcept
    {
        static_assert(sizeof(XYT) == sizeof(UInt64));
        return robin_hood::hash<UInt64>{}(xyt.xyt);
    }
};

template<>
struct robin_hood::hash<NodeTime>
{
    inline std::size_t operator()(const NodeTime nt) const noexcept
    {
        static_assert(sizeof(NodeTime) == sizeof(UInt64));
        return robin_hood::hash<UInt64>{}(nt.nt);
    }
};

template<>
struct robin_hood::hash<Edge>
{
    inline std::size_t operator()(const Edge e) const noexcept
    {
        static_assert(sizeof(Edge) == sizeof(UInt32));
        return robin_hood::hash<UInt32>{}(e.id);
    }
};

template<>
struct robin_hood::hash<EdgeTime>
{
    inline std::size_t operator()(const EdgeTime et) const noexcept
    {
        static_assert(sizeof(EdgeTime) == sizeof(UInt64));
        return robin_hood::hash<UInt64>{}(et.id);
    }
};

template<>
struct robin_hood::hash<AgentTime>
{
    inline std::size_t operator()(const AgentTime at) const noexcept
    {
        static_assert(sizeof(AgentTime) == 8);
        return robin_hood::hash<UInt64>{}(at.id);
    }
};

template<>
struct robin_hood::hash<AgentNodeTime>
{
    inline std::size_t operator()(const AgentNodeTime ant) const noexcept
    {
        static_assert(sizeof(AgentNodeTime) == 12);
        auto x = robin_hood::hash<NodeTime>{}(ant.a);
        hash_combine(x, ant.n);
        hash_combine(x, ant.t);
        return x;
    }
};

template<>
struct robin_hood::hash<AgentEdgeTime>
{
    inline std::size_t operator()(const AgentEdgeTime aet) const noexcept
    {
        static_assert(sizeof(AgentEdgeTime) == 12);
        auto x = robin_hood::hash<Edge>{}(aet.e);
        hash_combine(x, aet.a);
        hash_combine(x, aet.t);
        return x;
    }
};
