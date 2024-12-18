#include "problem/debug.h"
#include "types/memory_pool.h"
#include "types/tracy.h"

#define BLOCK_SIZE (100 * 1024)

#define TRACY_COLOUR tracy::Color::ColorType::LightGrey

MemoryPool::MemoryPool() :
    blocks_(),
    block_idx_(0),
    byte_idx_(0),
    object_size_(1)
{
    ZoneScopedC(TRACY_COLOUR);

    debug_assert(BLOCK_SIZE % 8 == 0);

    // blocks_.reserve(50);
    blocks_.push_back(std::make_unique<Byte[]>(BLOCK_SIZE));
    debug_assert(blocks_.back());
}

template<Bool commit, Bool zero_out>
void* MemoryPool::get_buffer()
{
    // ZoneScopedC(TRACY_COLOUR);

    // Advance to the next block if there's no space in the current block.
    if (byte_idx_ + object_size_ >= BLOCK_SIZE)
    {
        // Advance to the next block.
        block_idx_++;
        byte_idx_ = 0;

        // Allocate new block if no space left.
        if (block_idx_ == blocks_.size())
        {
            blocks_.push_back(std::make_unique<Byte[]>(BLOCK_SIZE));
            debug_assert(blocks_.back());
        }
    }

    // Find an address to store the object.
    debug_assert(block_idx_ < blocks_.size());
    debug_assert(byte_idx_ < BLOCK_SIZE);
    auto object = reinterpret_cast<void*>(&(blocks_[block_idx_][byte_idx_]));
    debug_assert(reinterpret_cast<uintptr_t>(object) % 8 == 0);

    // Zero out the memory if needed.
    if constexpr (zero_out)
    {
        std::memset(object, 0, object_size_);
    }

    // Commit the buffer if needed.
    if constexpr (commit)
    {
        commit_buffer();
    }

    // Done.
    return object;
}
template void* MemoryPool::get_buffer<false, false>();
template void* MemoryPool::get_buffer<false, true>();
template void* MemoryPool::get_buffer<true, false>();
template void* MemoryPool::get_buffer<true, true>();

void MemoryPool::commit_buffer()
{
    // ZoneScopedC(TRACY_COLOUR);

    debug_assert(block_idx_ < blocks_.size());
    debug_assert(byte_idx_ < BLOCK_SIZE);
    debug_assert(object_size_ % 8 == 0);
    byte_idx_ += object_size_;
}

void MemoryPool::reset(const Size object_size)
{
    // ZoneScopedC(TRACY_COLOUR);

    block_idx_ = 0;
    byte_idx_ = 0;
    object_size_ = ((object_size + 7) & (-8)); // Round up to next multiple of 8
}
