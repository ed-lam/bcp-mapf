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

// ---------------------------------------------------------------------------------------

#define OUTPUTS_DIR "outputs"
#define ARTIFICIAL_VAR_COST                                 1e6
#define PRICE_PRIORITY_DECAY_FACTOR                         1.3
#define CUT_VIOLATION                                       0.1

// ---------------------------------------------------------------------------------------

#include "types/basic_types.h"
#include "problem/debug.h"
#include <string>
#include <memory>
#include <array>
#include <string>
#include <memory>
#include <utility>
#include "robin-hood-hashing/src/include/robin_hood.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "scip/scip.h"
#pragma GCC diagnostic pop

using Int = Int32;
using Waypoint = Int32;

using String = std::string;

template<class T, size_t Size>
using Array = std::array<T, Size>;

template<class T1, class T2>
using Pair = std::pair<T1, T2>;

template <class T>
using UniquePtr = std::unique_ptr<T>;

// ---------------------------------------------------------------------------------------

template<class ...T>
using Tuple = std::tuple<T...>;

template<class T>
using SharedPtr = std::shared_ptr<T>;

// ---------------------------------------------------------------------------------------

