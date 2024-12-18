#pragma once

#include "types/basic_types.h"
#include <climits>

static inline Bool get_bitset(const void* bitset, const size_t i)
{
    const auto idx = i / CHAR_BIT;
    const auto mask = 1 << (i % CHAR_BIT);
    return (reinterpret_cast<const Byte*>(bitset)[idx] & mask) != 0;
}

static inline void set_bitset(void* const bitset, const size_t i)
{
    const auto idx = i / CHAR_BIT;
    const auto mask = 1 << (i % CHAR_BIT);
    reinterpret_cast<Byte*>(bitset)[idx] |= mask;
}

// static inline void clear_bitset(void* const bitset, const size_t i)
// {
//     const auto idx = i / CHAR_BIT;
//     const auto mask = 1 << (i % CHAR_BIT);
//     reinterpret_cast<Byte*>(bitset)[idx] &= ~mask;
// }

// static inline Bool flip_bitset(void* const bitset, const size_t i)
// {
//     const auto idx = i / CHAR_BIT;
//     const auto mask = 1 << (i % CHAR_BIT);
//     reinterpret_cast<Byte*>(bitset)[idx] ^= mask;
//     return (reinterpret_cast<Byte*>(bitset)[idx] & mask) != 0;
// }

// static inline void and_bitset(void* const bitset1, const void* const bitset2, const size_t size)
// {
//     for (size_t idx = 0; idx < size; ++idx)
//     {
//         reinterpret_cast<Byte*>(bitset1)[idx] &= reinterpret_cast<const Byte*>(bitset2)[idx];
//     }
// }

// static inline void or_bitset(void* const bitset1, const void* const bitset2, const size_t size)
// {
//     for (size_t idx = 0; idx < size; ++idx)
//     {
//         reinterpret_cast<Byte*>(bitset1)[idx] |= reinterpret_cast<const Byte*>(bitset2)[idx];
//     }
// }
