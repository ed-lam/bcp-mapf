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

#include "Includes.h"

namespace TruffleHog
{

template<class Label, class Compare, bool IsHeuristic>
class PriorityQueue
{
    Compare cmp_;
    Int capacity_;
    Int size_;
    Label** elts_;

  public:
    // Constructors
    template<class ...Args>
    PriorityQueue(Args... args) :
        cmp_(args...),
        capacity_(0),
        size_(0),
        elts_(nullptr)
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

    // Reprioritise an element up or down
    void decrease_key(Label* label)
    {
        if constexpr (!IsHeuristic)
        {
            debug_assert(contains(label));
            heapify_up(label->pqueue_index);
        }
        else
        {
            err("");
        }
    }
    void increase_key(Label* label)
    {
        if constexpr (!IsHeuristic)
        {
            debug_assert(contains(label));
            heapify_down(label->pqueue_index);
        }
        else
        {
            err("");
        }
    }

    // Add an element
    void push(Label* label)
    {
        if (size_ + 1 > capacity_)
        {
            enlarge(capacity_ * 2);
        }

        auto index = size_;
        elts_[index] = label;
        if constexpr (!IsHeuristic)
        {
            label->pqueue_index = index;
        }
        size_++;
        heapify_up(index);
    }

    // Remove the top element
    Label* pop()
    {
        debug_assert(size_ > 0);

        auto label = elts_[0];
        if constexpr (!IsHeuristic)
        {
            label->pqueue_index = -1;
        }
        size_--;

        if (size_ > 0)
        {
            elts_[0] = elts_[size_];
            if constexpr (!IsHeuristic)
            {
                elts_[0]->pqueue_index = 0;
            }
            heapify_down(0);
        }

        return label;
    }

    // Check if a label is contained within
    inline bool contains(Label* label) const
    {
        if constexpr (!IsHeuristic)
        {
            const auto index = label->pqueue_index;
            return index < size_ && label == elts_[index];
        }
        else
        {
            err("");
        }
    }

    // Retrieve the top element without removing it
    inline Label* top()
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

  private:
    // Reorder the subtree containing elts_[index]
    void heapify_up(Int index)
    {
        debug_assert(index < size_);

        while (index > 0)
        {
            const auto parent = (index - 1) >> 1;
            if (cmp_(elts_[index], elts_[parent]))
            {
                swap(parent, index);
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
            // find smallest (or largest, depending on heap type) child
            const auto child1 = (index << 1) + 1;
            const auto child2 = (index << 1) + 2;
            const auto which = child2 < size_ && cmp_(elts_[child2], elts_[child1]) ?
                               child2 :
                               child1;

            // swap child with parent if necessary
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

        if constexpr (!IsHeuristic)
        {
            elts_[index1]->pqueue_index = index2;
            elts_[index2]->pqueue_index = index1;
        }
        std::swap(elts_[index1], elts_[index2]);
    }

    // Check.
#ifdef DEBUG
    void check() const
    {
        if constexpr (!IsHeuristic)
        {
            for (Int index = 0; index < size_; ++index)
            {
                debug_assert(elts_[index]->pqueue_index == index);
            }
        }
    }
  public:
    void check_label(Label* label)
    {
        debug_assert(label &&
                     (-1 == label->pqueue_index ||
                      (label->pqueue_index < size_ && elts_[label->pqueue_index] == label)));
    }
#endif
};

}

#endif
