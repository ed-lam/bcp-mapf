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

#ifndef TRUFFLEHOG_PRIORITYQUEUE_H
#define TRUFFLEHOG_PRIORITYQUEUE_H

// #define CHECK_HEAP

#include "Includes.h"

#define EPS (1e-6)
#define isLE(x, y) ((x)-(y) <= (EPS))

namespace TruffleHog
{

template<class Label, class Compare>
class PriorityQueue
{
  protected:
    Compare cmp_;
    Label** elts_;
    Int capacity_;
    Int size_;

  public:
    // Constructors
    template<class ...Args>
    PriorityQueue(Args... args) :
        cmp_(args...),
        elts_(nullptr),
        capacity_(0),
        size_(0)
    {
        enlarge(8192);
    }
    PriorityQueue(const PriorityQueue&) = delete;
    PriorityQueue(PriorityQueue&&) = delete;
    PriorityQueue& operator=(const PriorityQueue&) = delete;
    PriorityQueue& operator=(PriorityQueue&&) = delete;
    ~PriorityQueue()
    {
        std::free(elts_);
    }

    // Remove all elements
    inline void clear()
    {
        size_ = 0;
    }

    // Add an element
    void push(Label* label)
    {
#ifdef CHECK_HEAP
        check_heap();
#endif

        if (size_ >= capacity_)
        {
            enlarge(capacity_ * 2);
        }

        auto index = size_;
        elts_[index] = label;
        update_pqueue_index(elts_[index], index);
        size_++;
        heapify_up(index);

#ifdef CHECK_HEAP
        check_heap();
#endif
    }

    // Remove the top element
    Label* pop()
    {
        debug_assert(size_ > 0);

#ifdef CHECK_HEAP
        check_heap();
#endif

        auto label = elts_[0];
        update_pqueue_index(label, -1);
        size_--;

        if (size_ > 0)
        {
            elts_[0] = elts_[size_];
            update_pqueue_index(elts_[0], 0);
            heapify_down(0);
        }

#ifdef CHECK_HEAP
        check_heap();
#endif

        return label;
    }

    // Retrieve the top element without removing it
    inline Label* top() const
    {
        debug_assert(size_ > 0);
        return elts_[0];
    }

    // Get the number of elements stored within
    inline auto size() const
    {
        return size_;
    }

    // Check whether the priority queue is empty
    inline auto empty() const
    {
        return size() == 0;
    }

    // Get the comparison function
    inline Compare& cmp() { return cmp_; }
    inline const Compare& cmp() const { return cmp_; }

  protected:
    // Reorder the subtree containing elts_[index]
    void heapify_up(Int index)
    {
        debug_assert(index < size_);

        while (index > 0)
        {
            const auto parent = (index - 1) >> 1;
            if (cmp_(elts_[index], elts_[parent]))
            {
                swap(index, parent);
                index = parent;
            }
            else
            {
                break;
            }
        }
    }

    // Reorders the subtree under elts_[index]
    void heapify_down(Int index)
    {
        debug_assert(index < size_);

        const auto first_leaf_index = size_ >> 1;
        while (index < first_leaf_index)
        {
            // Find Better child.
            const auto child1 = (index << 1) + 1;
            const auto child2 = (index << 1) + 2;
            const auto which = child2 < size_ && cmp_(elts_[child2], elts_[child1]) ?
                               child2 :
                               child1;

            // Swap child with parent if necessary.
            if (cmp_(elts_[which], elts_[index]))
            {
                swap(index, which);
                index = which;
            }
            else
            {
                break;
            }
        }
    }

    // Allocate more memory
    void enlarge(const Int new_capacity)
    {
        debug_assert(new_capacity > capacity_);
        elts_ = reinterpret_cast<Label**>(std::realloc(elts_, new_capacity * sizeof(Label*)));
        release_assert(elts_, "Failed to reallocate memory");
        capacity_ = new_capacity;
    }

    // Swap the positions of two labels
    inline void swap(const Int index1, const Int index2)
    {
        debug_assert(index1 < size_ && index2 < size_);

        std::swap(elts_[index1], elts_[index2]);
        update_pqueue_index(elts_[index1], index1);
        update_pqueue_index(elts_[index2], index2);
    }

    // Modify the handle in the label pointing to its position in the priority queue
    virtual void update_pqueue_index(Label* label, const Int pqueue_index) = 0;

    // Checks.
#ifdef DEBUG
    virtual Cost get_f(const Label* label) const = 0;
    virtual void check_pqueue_index() const = 0;
    void check_heap() const
    {
        check_pqueue_index();

        for (Int index = 1; index < size_; ++index)
        {
            const auto parent = (index - 1) >> 1;
            release_assert(isLE(get_f(elts_[parent]), get_f(elts_[index])));
            // release_assert(!cmp_(elts_[index], elts_[parent]));
        }
    }
#endif
};

}

#endif
