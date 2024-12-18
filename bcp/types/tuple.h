#pragma once

#include <utility>

template<class T1, class T2>
using Pair = std::pair<T1, T2>;

template<class... T>
using Tuple = std::tuple<T...>;
