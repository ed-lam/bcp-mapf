// #define PRINT_DEBUG

#include "pricing/distance_heuristic.h"
#include "types/tracy.h"

#define TRACY_COLOUR tracy::Color::ColorType::LightSkyBlue

DistanceHeuristic::DistanceHeuristic(const Map& map) :
    map_(map),
    h_(),
    open_()
{
    ZoneScopedC(TRACY_COLOUR);

    h_.reserve(1000);
}

void DistanceHeuristic::generate_start(const Node n, Time* h)
{
    ZoneScopedC(TRACY_COLOUR);

    const Time g = 0;
    h[n] = g;
    open_.push(Label{g, n});
}

void DistanceHeuristic::generate(const Node current, const Node next, Time* h)
{
    ZoneScopedC(TRACY_COLOUR);

    const auto g = h[current] + 1;
    if (g < h[next])
    {
        h[next] = g;
        open_.push(Label{g, next});
    }
}

void DistanceHeuristic::generate_neighbours(const Node current, Time* h, const Map& map)
{
    ZoneScopedC(TRACY_COLOUR);

    // Expand in four directions.
    if (const auto next = map.get_north(current); map[next])
    {
        generate(current, next, h);
    }
    if (const auto next = map.get_south(current); map[next])
    {
        generate(current, next, h);
    }
    if (const auto next = map.get_west(current); map[next])
    {
        generate(current, next, h);
    }
    if (const auto next = map.get_east(current); map[next])
    {
        generate(current, next, h);
    }
}

void DistanceHeuristic::search(const Node goal, Time* h, const Map& map)
{
    ZoneScopedC(TRACY_COLOUR);

    // Reset.
    open_.clear();

    // Fill with infeasible time.
    std::fill(h, h + map.size(), TIME_MAX);

    // Solve.
    generate_start(goal, h);
    while (!open_.empty())
    {
        // Get a label from priority queue.
        const auto [_, current] = open_.top();
        open_.pop();

        // Generate neighbours.
        generate_neighbours(current, h, map);
    }
}

const Time* DistanceHeuristic::get_h(const Node goal)
{
    ZoneScopedC(TRACY_COLOUR);

    auto& h = h_[goal];
    if (h.empty())
    {
        // Allocate memory for N+1 nodes, ranging from -1 to N-1 , where n = -1 represents the end.
        h.resize(map_.size() + 1);
        h[0] = 0;

        // Compute the h values for this goal.
        search(goal, h.data() + 1, map_);
    }
    return h.data() + 1; // INFO: Returning pointer to data in vector so h_ can change.
}

Vector<Time> DistanceHeuristic::get_h_using_map(const Node goal, const Map& map)
{
    ZoneScopedC(TRACY_COLOUR);

    Vector<Time> h(map.size());
    search(goal, h.data(), map);
    return h;
}
