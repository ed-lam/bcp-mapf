#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

using Byte = unsigned char;
using Bool = bool;
using Int16 = int16_t;
using Int32 = int32_t;
using Int64 = int64_t;
using UInt16 = uint16_t;
using UInt32 = uint32_t;
using UInt64 = uint64_t;
using Float = double;
using Size = std::ptrdiff_t;

// using Agent = Int16;
// using Cost = Float;
// using Time = Int32;

#include "pricing/coordinates.h"

constexpr auto INF = std::numeric_limits<Cost>::infinity();
constexpr auto TIME_MAX = std::numeric_limits<Time>::max();
