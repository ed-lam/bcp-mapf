#pragma once

#ifdef DEBUG
// #define CHECK_HEAP
#endif

#include "problem/debug.h"
#include "types/float_compare.h"
#include "types/vector.h"
#include "types/basic_types.h"

template<class Item, class Comparison, class SizeType = int64_t, SizeType BranchingFactor = 4>
class PriorityQueue
{
  protected:
    Vector<Item> items_;

  public:
    // Constructors and destructor
    PriorityQueue() : items_() {}
    PriorityQueue(const PriorityQueue&) = delete;
    PriorityQueue(PriorityQueue&&) = default;
    PriorityQueue& operator=(const PriorityQueue&) = delete;
    PriorityQueue& operator=(PriorityQueue&&) = default;
    ~PriorityQueue() = default;

    // Remove all items
    inline void clear() { items_.clear(); }

    // Add a new item
    template<class... T>
    void push(T&&... args)
    {
        // Check for consistency.
        check_heap();

        // Add the item.
        const SizeType index = items_.size();
        auto& item = items_.emplace_back(std::forward<T>(args)...);
        update_index(item, index);
        check_indices();
        heapify_up(index);
    }

    // Remove the top item
    Item pop()
    {
        // Check for consistency.
        debug_assert(!items_.empty());
        check_heap();

        // Get the item at the top.
        auto item = items_[0];
        update_index(item, -1);

        // Restore heap invariant.
        if (items_.size() > 1)
        {
            items_[0] = std::move(items_.back());
            items_.pop_back();
            update_index(items_[0], 0);
            check_indices();
            heapify_down(0);
        }
        else
        {
            items_.pop_back();
        }

        // Check for consistency.
        check_heap();

        // Return the top item.
        return item;
    }

    // Change the priority of an existing item
    template<class... T>
    void update(SizeType index, T&&... args)
    {
        // Replace the item if required.
        auto& item = items_[index];
        if (sizeof...(args) > 0)
        {
            item = Item{std::forward<T>(args)...};
        }

        // Update the item's index.
        update_index(item, index);
        check_indices();

        // Restore heap invariant.
        if (index == 0 || Comparison::lt(items_[(index - 1) / BranchingFactor], item))
        {
            heapify_down(index);
        }
        else
        {
            heapify_up(index);
        }
    }
    template<class... T>
    void increase(SizeType index, T&&... args)
    {
        // Replace the item if required.
        auto& item = items_[index];
        if (sizeof...(args) > 0)
        {
            item = Item{std::forward<T>(args)...};
        }

        // Update the item's index.
        update_index(item, index);
        check_indices();

        // Restore heap invariant.
        debug_assert(!(index == 0 || Comparison::lt(items_[(index - 1) / BranchingFactor], item)));
        heapify_up(index);
    }
    template<class... T>
    void decrease(SizeType index, T&&... args)
    {
        // Replace the item if required.
        auto& item = items_[index];
        if (sizeof...(args) > 0)
        {
            item = Item{std::forward<T>(args)...};
        }

        // Update the item's index.
        update_index(item, index);
        check_indices();

        // Restore heap invariant.
        debug_assert(index == 0 || Comparison::lt(items_[(index - 1) / BranchingFactor], item));
        heapify_down(index);
    }

    // Delete an item in the given position
    void erase(const SizeType index)
    {
        // Check for consistency.
        debug_assert(0 <= index && index < items_.size());
        check_heap();

        // Update the index of the item.
        update_index(items_[index], -1);

        // Shuffle items around.
        const SizeType back_index = items_.size() - 1;
        if (index == back_index)
        {
            // Remove the item.
            items_.pop_back();

            // Check for consistency.
            check_heap();
        }
        else
        {
            // Replace the item with the item at the back.
            items_[index] = std::move(items_.back());
            items_.pop_back();
            update_index(items_[index], index);

            // Restore heap invariant.
            if (index == 0 || Comparison::lt(items_[(index - 1) / BranchingFactor], items_[index]))
            {
                heapify_down(index);
            }
            else
            {
                heapify_up(index);
            }
        }
    }

    // Get the top item without removing it
    inline const auto& top() const { debug_assert(!items_.empty()); return items_[0]; }

    // Get the number of items stored
    inline SizeType size() const { return items_.size(); }

    // Check whether the priority queue is empty
    inline auto empty() const { return items_.empty(); }

  protected:
    // Reorder the subtree containing items_[index]
    void heapify_up(SizeType index)
    {
        // Move items up.
        debug_assert(0 <= index && index < items_.size());
        while (index > 0)
        {
            const auto parent = (index - 1) / BranchingFactor;
            debug_assert(0 <= parent && parent < index);
            if (Comparison::lt(items_[index], items_[parent]))
            {
                swap(index, parent);
                index = parent;
                check_indices();
            }
            else
            {
                break;
            }
        }

        // Check for consistency.
        check_heap();
    }

    // Reorders the subtree under items_[index]
    void heapify_down(SizeType index)
    {
        // Move items down.
        debug_assert(0 <= index && index < items_.size());
        while (true)
        {
            // Find best child.
            auto smallest = index;
            {
                const auto last = std::min<SizeType>(items_.size(), (index * BranchingFactor) + BranchingFactor + 1);
                for (auto child = (index * BranchingFactor) + 1; child < last; ++child)
                    if (Comparison::lt(items_[child], items_[smallest]))
                    {
                        smallest = child;
                    }
            }

            // Swap with the smallest child if available.
            if (smallest == index)
            {
                break;
            }
            swap(index, smallest);
            index = smallest;
            check_indices();
        }

        // Check for consistency.
        check_heap();
    }

    // Swap the positions of two items
    inline void swap(const SizeType index1, const SizeType index2)
    {
        debug_assert(0 <= index1 && index1 < items_.size());
        debug_assert(0 <= index2 && index2 < items_.size());
        std::swap(items_[index1], items_[index2]);
        update_index(items_[index1], index1);
        update_index(items_[index2], index2);
        check_indices();
    }

    // Modify the handle in the label pointing to its position in the priority queue
    virtual void update_index(Item item, const SizeType index) = 0;

    // Checks.
    virtual Bool check_index(const Item& item, const SizeType index) const = 0;
    inline void check_indices() const
    {
#ifdef DEBUG
#ifdef CHECK_HEAP
        for (SizeType index = 0; index < items_.size(); ++index)
        {
            debug_assert(check_index(items_[index], index));
        }
#endif
#endif
    }
    inline void check_heap() const
    {
#ifdef DEBUG
#ifdef CHECK_HEAP
        check_indices();
        for (SizeType index = 1; index < items_.size(); ++index)
        {
            const auto parent = (index - 1) / BranchingFactor;
            debug_assert(0 <= parent && parent < index);
            release_assert(Comparison::le(items_[parent], items_[index]));
        }
#endif
#endif
    }
};
