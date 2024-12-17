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

#ifndef MAPF_INCLUDES_H
#define MAPF_INCLUDES_H

// ---------------------------------------------------------------------------------------

#define OUTPUTS_DIR "outputs"
#define ARTIFICIAL_VAR_COST                                 1e6
#define PRICE_PRIORITY_DECAY_FACTOR                         1.3
#define CUT_VIOLATION                                       0.1

// ---------------------------------------------------------------------------------------

#include "problem/debug.h"
#include <string>
#include <memory>
#include <array>
#include <vector>
#include <string>
#include <memory>
#include <utility>
#include "robin-hood-hashing/src/include/robin_hood.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "scip/scip.h"
#pragma GCC diagnostic pop

namespace TruffleHog
{

using Int = int32_t;
//using UInt = uint32_t;
using Float = double;

using Position = Int;
using Agent = Int;
using Cost = Float;
using IntCost = Int;
using Waypoint = Int;

using String = std::string;

template<class T, size_t Size>
using Array = std::array<T, Size>;

template<class T>
using Vector = std::vector<T>;

template<class Key, class T, class Hash = robin_hood::hash<Key>, class KeyEqual = std::equal_to<Key>>
using HashTable = robin_hood::unordered_flat_map<Key, T, Hash, KeyEqual, 60>;

template<class T1, class T2>
using Pair = std::pair<T1, T2>;

template <class T>
using UniquePtr = std::unique_ptr<T>;

}

// ---------------------------------------------------------------------------------------

using namespace TruffleHog;

template<class ...T>
using Tuple = std::tuple<T...>;

template<class T>
using SharedPtr = std::shared_ptr<T>;

// ---------------------------------------------------------------------------------------

#endif
