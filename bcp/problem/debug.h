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

#pragma once

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "scip/scip.h"
#pragma GCC diagnostic pop

#include <fmt/color.h>
#include <fmt/format.h>
#include <fmt/ranges.h>

#ifdef DEBUG
#define println(format, ...) do { \
    fmt::print(format "\n", ##__VA_ARGS__); \
    fflush(stdout); \
} while (false)
#else
#define println(format, ...) do { \
    fmt::print(format "\n", ##__VA_ARGS__); \
} while (false)
#endif

#ifdef PRINT_DEBUG
#define debugln(format, ...) println(format, ##__VA_ARGS__)
#define debug(format, ...) fmt::print(format, ##__VA_ARGS__);
#else
#define debugln(format, ...) {}
#define debug(format, ...) {}
#endif

#ifdef DEBUG
#define err(format, ...) do { \
    fmt::print(stderr, "Error: " format "\n", ##__VA_ARGS__); \
    fmt::print(stderr, "Function: {}\n", __PRETTY_FUNCTION__); \
    fmt::print(stderr, "File: {}\n", __FILE__); \
    fmt::print(stderr, "Line: {}\n", __LINE__); \
    std::abort(); \
} while (false)
#else
#define err(format, ...) do { \
    fmt::print(stderr, "Error: " format "\n", ##__VA_ARGS__); \
    std::abort(); \
} while (false)
#endif

#define release_assert(condition, ...) do { \
    if (!(condition)) err(__VA_ARGS__); \
} while (false)

#ifdef DEBUG
#define debug_assert(condition) release_assert(condition, "{}", #condition)
#else
#define debug_assert(condition) {}
#endif

#define scip_assert(statement, ...) do { \
    const auto error = statement; \
    release_assert(error == SCIP_OKAY, "SCIP error {}", static_cast<int>(error)); \
} while (false)

#define not_yet_implemented() do { \
    err("Reached unimplemented section in function \"{}\"", __PRETTY_FUNCTION__); \
} while (false)

#define not_yet_checked() do { \
    err("Reached unchecked section in function \"{}\"", __PRETTY_FUNCTION__); \
} while (false)

#define unreachable() do { \
    err("Reached unreachable section in function \"{}\"", __PRETTY_FUNCTION__); \
} while (false)

