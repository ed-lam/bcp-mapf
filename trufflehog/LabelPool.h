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

#ifndef TRUFFLEHOG_LABELPOOL_H
#define TRUFFLEHOG_LABELPOOL_H

#include "Includes.h"
#include "Coordinates.h"
#include <memory>
#include <cstddef>

namespace TruffleHog
{

class LabelPool
{
    Vector<UniquePtr<std::byte[]>> blocks_;
    Int block_idx_;
    Int byte_idx_;
    Int label_size_;

  public:
    // Constructors
    LabelPool();
    LabelPool(const LabelPool&) = delete;
    LabelPool(LabelPool&&) = delete;
    LabelPool& operator=(const LabelPool&) = delete;
    LabelPool& operator=(LabelPool&&) = delete;
    ~LabelPool() = default;

    // Getters
    Int label_size() const { return label_size_; }

    // Get pointer to store a label
    void* get_label_buffer();
    void take_label();

    // Reset all labels
    void reset(const Int label_size);

  private:
    // Allocate
    void allocate_blocks();
};

}

#endif
