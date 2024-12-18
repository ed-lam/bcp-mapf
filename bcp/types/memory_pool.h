#pragma once

#include "types/pointers.h"
#include "types/basic_types.h"
#include "types/vector.h"
#include <cstddef>
#include <memory>

class MemoryPool
{
    Vector<UniquePtr<Byte[]>> blocks_;
    Size block_idx_;
    Size byte_idx_;
    Size object_size_;

  public:
    // Constructors and destructor
    MemoryPool();
    MemoryPool(const MemoryPool&) = delete;
    MemoryPool(MemoryPool&&) noexcept = default;
    MemoryPool& operator=(const MemoryPool&) = delete;
    MemoryPool& operator=(MemoryPool&&) noexcept = default;
    ~MemoryPool() = default;

    // Getters
    inline auto object_size() const { return object_size_; }

    // Get pointer to store an object
    template<Bool commit, Bool zero_out>
    void* get_buffer();
    void commit_buffer();

    // Clear all storage
    void reset(const Size object_size);

  private:
    // Allocate
    void allocate_blocks();
};
