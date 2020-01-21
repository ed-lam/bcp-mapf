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

#include "LabelPool.h"

#define BLOCK_SIZE (10 * 1024 * 1024)

namespace TruffleHog
{

LabelPool::LabelPool()
    : blocks_(),
      block_idx_(0),
      byte_idx_(0),
      label_size_(1)
{
    blocks_.reserve(50);
    blocks_.push_back(std::make_unique<std::byte[]>(BLOCK_SIZE));
    debug_assert(blocks_.back());
}

void* LabelPool::get_label_buffer()
{
    // Move to the next block if there's no space in the current block.
    if (byte_idx_ + label_size_ >= BLOCK_SIZE)
    {
        block_idx_++;
        byte_idx_ = 0;
    }

    // Allocate new block if no space left.
    if (block_idx_ == static_cast<Int>(blocks_.size()))
    {
        blocks_.push_back(std::make_unique<std::byte[]>(BLOCK_SIZE));
        debug_assert(blocks_.back());
    }

    // Find the memory to store the label.
    debug_assert(block_idx_ < static_cast<Int>(blocks_.size()));
    debug_assert(byte_idx_ < BLOCK_SIZE);
    auto label = reinterpret_cast<void*>(&(blocks_[block_idx_][byte_idx_]));
    debug_assert(reinterpret_cast<uintptr_t>(label) % 8 == 0);

    // Done.
    return label;
}

void LabelPool::take_label()
{
    debug_assert(block_idx_ < static_cast<Int>(blocks_.size()));
    debug_assert(byte_idx_ < BLOCK_SIZE);
    byte_idx_ += label_size_;
}

void LabelPool::reset(const Int label_size)
{
    block_idx_ = 0;
    byte_idx_ = 0;
    label_size_ = label_size % 8 ? // Round up to next multiple of 8
                  label_size + (8 - label_size % 8) :
                  label_size;
}

}
