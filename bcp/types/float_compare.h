#pragma once

#include "types/basic_types.h"
#include <cmath>

#define EPS (1e-6)

template<class T>
inline Bool is_eq(T x, T y) { return (x == y) || (std::abs(x - y) <= EPS); }

template<class T>
inline Bool is_lt(T x, T y) { return x - y < -EPS; }

template<class T>
inline Bool is_le(T x, T y) { return x - y <= EPS; }

template<class T>
inline Bool is_gt(T x, T y) { return x - y > EPS; }

template<class T>
inline Bool is_ge(T x, T y) { return x - y >= -EPS; }

template<class T>
inline T eps_floor(T x) { return std::floor(x + EPS); }

template<class T>
inline T eps_ceil(T x) { return std::ceil(x - EPS); }

template<class T>
inline T eps_round(T x) { return std::ceil(x - 0.5 + EPS); }

template<class T>
inline T eps_frac(T x) { return x - eps_floor(x); }

template<class T>
inline Bool is_integral(T x) { return eps_frac(x) <= EPS; }
