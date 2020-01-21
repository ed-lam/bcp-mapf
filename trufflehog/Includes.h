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

#ifndef TRUFFLEHOG_INCLUDES_H
#define TRUFFLEHOG_INCLUDES_H

#include "Debug.h"
#include <vector>
#include <string>
#include <memory>
#include <utility>
#include "robin-hood-hashing/src/include/robin_hood.h"

namespace TruffleHog
{

using Int = int32_t;
//using UInt = uint32_t;
using Float = double;

using Position = Int;
using Agent = Int;
using Cost = Float;
using IntCost = Int;

using String = std::string;

template<class T>
using Vector = std::vector<T>;

template<class Key, class T, class Hash = robin_hood::hash<Key>, class KeyEqual = std::equal_to<Key>>
using HashTable = robin_hood::unordered_flat_map<Key, T, Hash, KeyEqual>;

template<class T1, class T2>
using Pair = std::pair<T1, T2>;

template <class T>
using UniquePtr = std::unique_ptr<T>;

}

#endif
