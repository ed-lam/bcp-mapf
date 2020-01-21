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

#ifndef TRUFFLEHOG_RESERVATIONTABLE_H
#define TRUFFLEHOG_RESERVATIONTABLE_H

#include "Includes.h"
#include "Coordinates.h"
#include <cmath>

namespace TruffleHog
{

class ReservationTable
{
    char* table_;
    Time timesteps_;
    const Int map_size_;

  public:
    // Constructors
    ReservationTable(const Int map_size) :
        map_size_(map_size)
    {
        timesteps_ = 4 * std::sqrt(map_size_);
        table_ = static_cast<char*>(std::malloc(table_size(timesteps_)));
        clear_reservations();
    }
    ReservationTable() = delete;
    ReservationTable(const ReservationTable&) = delete;
    ReservationTable(ReservationTable&&) = delete;
    ReservationTable& operator=(const ReservationTable&) = delete;
    ReservationTable& operator=(ReservationTable&&) = delete;
    ~ReservationTable() noexcept
    {
        std::free(table_);
    }

    // Check and make reservation
    inline auto map_size() const
    {
        return map_size_;
    }
    inline bool is_reserved(const NodeTime nt) const
    {
        // Check.
        debug_assert(nt.n < map_size_);
        debug_assert(nt.t >= 0);

        // Not reserved if beyond allocated memory.
        if (nt.t >= timesteps_)
        {
            return false;
        }

        // Get the bit.
        const auto elem = static_cast<size_t>(nt.t) * map_size_ + nt.n;
        const auto idx = elem / CHAR_BIT;
        const char mask = 0x1 << (elem % CHAR_BIT);
        const auto val = (table_[idx] & mask) != 0x0;
        return val;
    }
    inline void reserve(const NodeTime nt)
    {
        // Check.
        debug_assert(nt.n < map_size_);
        debug_assert(nt.t >= 0);

        // Reallocate if not enough memory.
        if (nt.t >= timesteps_)
        {
            // Reallocate.
            const auto new_timesteps = nt.t + 50;
            table_ = static_cast<char*>(std::realloc(table_, table_size(new_timesteps)));
            release_assert(table_, "Failed to reallocate memory for reservation table");

            // Clear new section of memory.
            const auto begin = table_ + table_size(timesteps_);
            const auto end = table_ + table_size(new_timesteps);
            memset(begin, 0, end - begin);

            // Set size of memory.
            timesteps_ = new_timesteps;
        }

        // Set the bit.
        const auto elem = static_cast<size_t>(nt.t) * map_size_ + nt.n;
        const auto idx = elem / CHAR_BIT;
        const char mask = 0x1 << (elem % CHAR_BIT);
        table_[idx] |= mask;
    }
    inline void unreserve(const NodeTime nt)
    {
        // Check.
        debug_assert(nt.n < map_size_);
        debug_assert(nt.t >= 0);

        // Clear the bit.
        if (nt.t < timesteps_)
        {
            const auto elem = static_cast<size_t>(nt.t) * map_size_ + nt.n;
            const auto idx = elem / CHAR_BIT;
            const char mask = 0x1 << (elem % CHAR_BIT);
            table_[idx] &= ~mask;
        }
    }
    inline void clear_reservations()
    {
        memset(table_, 0, table_size(timesteps_));
    }

  private:
    // Calculate the size of the reservation table in memory
    size_t table_size(const Time timesteps)
    {
        return (static_cast<size_t>(timesteps) * static_cast<size_t>(map_size_)) / CHAR_BIT;
    }
};

}

#endif
