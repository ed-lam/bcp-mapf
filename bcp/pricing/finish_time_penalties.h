#pragma once

#include "problem/debug.h"
#include "types/basic_types.h"
#include "types/vector.h"

class FinishTimePenalties
{
    Vector<Cost> penalties_;
    Vector<Cost> h_;

  public:
    // Constructors and destructor
    FinishTimePenalties() noexcept = default;
    FinishTimePenalties(const FinishTimePenalties& other) = default;
    FinishTimePenalties(FinishTimePenalties&& other) noexcept = default;
    FinishTimePenalties& operator=(const FinishTimePenalties& other) = default;
    FinishTimePenalties& operator=(FinishTimePenalties&& other) noexcept = default;
    ~FinishTimePenalties() noexcept = default;

    // Getters
    inline Time size() const { return penalties_.size(); }
    // auto operator[](const Time t) const { debug_assert(t < penalties_.size()); return penalties_[t]; }

    // Get the lower bound (h value) at the current time (assuming the agent is already at the goal location)
    inline Cost get_h(const Time t) const { return t < h_.size() ? h_[t] : 0.0; }
    inline Cost get_penalty(const Time t) const { return t < penalties_.size() ? penalties_[t] : 0.0; }

    // Clear
    void clear()
    {
        penalties_.clear();
        h_.clear();
    }
    void move_to_old()
    {
#ifdef USE_GTL
        penalties_.clear();
#endif
        debug_assert(penalties_.empty());
        h_.clear();
    }

    // Add a finish time penalty for finishing at or before time leq_time
    void add(const Time leq_time, const Cost cost)
    {
        debug_assert(leq_time >= 0);
        debug_assert(cost >= 0);
        if (leq_time + 1 >= size())
        {
            penalties_.resize(leq_time + 1);
        }
        for (Time t = 0; t <= leq_time; ++t)
        {
            penalties_[t] += cost;
        }
    }

    // Prepare data structures for efficient lookup before solving
    void finalise()
    {
        // Sum up the penalties as a lower bound (h value).
        debug_assert(h_.empty());
        h_.resize(penalties_.size());
        for (Time t = 0; t < h_.size(); ++t)
        {
            // Wait until after all finish time penalties have elapsed and then finish at no cost.
            h_[t] = penalties_.size() - t;

            // Wait until time i and then finish with a penalty.
            for (Time i = t; i < penalties_.size(); ++i)
            {
                h_[t] = std::min(h_[t], i - t + penalties_[i]);
            }
        }
    }

    // Print
    void print() const
    {
        println("Finish time penalties:");
        println("{:>8s}{:>15s}", "T", "Penalty");
        for (Time t = 0; t < penalties_.size(); ++t)
        {
            println("{:>8d}{:>15.2f}", t, penalties_[t]);
        }
        println("");
    }
};
