#include "output/formatting.h"
#include "problem/instance.h"
#include "types/hash_map.h"
#include "types/float_compare.h"
#include <fmt/color.h>

String format_node(const Node n, const Map& map)
{
    return fmt::format("({},{})", map.get_x(n), map.get_y(n));
}

String format_nodetime(const NodeTime nt, const Map& map)
{
    return fmt::format("({},{})", format_node(nt.n, map), nt.t);
}

String format_edgetime(const EdgeTime et, const Map& map)
{
    const auto dest = map.get_destination(et);
    return fmt::format("({},{},{})", format_node(et.n, map), et.t, format_node(dest, map));
}

// Make a string of the coordinates in a path
String format_path(
    const Vector<Edge>& path,    // Path
    const Map& map,              // Map
    const String& separator      // Separator
)
{
    String str;
    Time t = 0;
    str.append(format_node(path[t].n, map));
    for (++t; t < path.size(); ++t)
    {
        str.append(fmt::format("{}{}", separator, format_node(path[t].n, map)));
    }
    return str;
}
String format_path_with_time(
    const Vector<Edge>& path,    // Path
    const Map& map,              // Map
    const String& separator      // Separator
)
{
    String str;
    Time t = 0;
    str.append(format_nodetime(NodeTime{path[t].n, t}, map));
    for (++t; t < path.size(); ++t)
    {
        str.append(fmt::format("{}{}", separator, format_nodetime(NodeTime{path[t].n, t}, map)));
    }
    return str;
}

// Make a string of the coordinates in a path in columns
String format_path_spaced(
    const Vector<Edge>& path,    // Path
    const Map& map               // Map
)
{
    String str;
    for (Time t = 0; t < path.size(); ++t)
    {
        str.append(fmt::format("{:>9s}", format_node(path[t].n, map)));
    }
    return str;
}
String format_path_with_time_spaced(
    const Vector<Edge>& path,    // Path
    const Map& map               // Map
)
{
    String str;
    for (Time t = 0; t < path.size(); ++t)
    {
        str.append(fmt::format("{:>12s}", format_nodetime(NodeTime{path[t].n, t}, map)));
    }
    return str;
}

// // Make a string of 0 and 1 of a bitset
// String format_bitset(const void* bitset, const size_t size)
// {
//     String str;
//     for (size_t i = 0; i < size; ++i)
//     {
//         if (get_bitset(bitset, i))
//         {
//             str.push_back('1');
//         }
//         else
//         {
//             str.push_back('0');
//         }
//     }
//     return str;
// }

// // Print paths with positive value
// void print_positive_paths(const Map& map, const Vector<Vector<MasterVariable>>& agent_path_vars)
// {
//     // Get the problem data.
//     const Agent A = agent_path_vars.size();
//
//     // Find the makespan.
//     Time makespan = 0;
//     for (Agent a = 0; a < A; ++a)
//         for (const auto& [_, val, path, __, ___] : agent_path_vars[a])
//             if (is_gt(val, 0.0))
//             {
//                 makespan = std::max<Time>(makespan, path.size());
//             }
//
//     // Get fractional edges and determine if the solution is fractional.
//     Bool is_fractional = false;
//     HashMap<NodeTime, HashMap<Agent, Float>> used_nodetimes;
//     HashMap<EdgeTime, HashMap<Agent, Float>> used_edgetimes;
//     for (Agent a = 0; a < A; ++a)
//         for (const auto& [_, val, path, __, ___] : agent_path_vars[a])
//         {
//             // Store fractional status.
//             is_fractional |= !is_integral(val);
//
//             // Store used arcs and edges.
//             if (is_gt(val, 0.0))
//             {
//                 Time t = 0;
//                 for (; t < path.size() - 1; ++t)
//                 {
//                     const NodeTime nt{path[t].n, t};
//                     used_nodetimes[nt][a] += val;
//
//                     const EdgeTime et{map.get_undirected_edge(path[t]), t};
//                     used_edgetimes[et][a] += val;
//                 }
//                 const auto n = path[t].n;
//                 for (; t < makespan; ++t)
//                 {
//                     const NodeTime nt{n, t};
//                     used_nodetimes[nt][a] += val;
//                 }
//             }
//         }
//
//     // Print time header line.
//     fmt::print("                                 ");
//     for (Time t = 0; t < makespan; ++t)
//     {
//         fmt::print("{:>10d}", t);
//     }
//     println("");
//
//     // Print paths.
//     for (Agent a = 0; a < A; ++a)
//         for (const auto& [_, val, path, __, ___] : agent_path_vars[a])
//             // if ((is_fractional && !is_integral(val)) || (!is_fractional && is_gt(val, 0.0)))
//             if (is_gt(val, 0.0))
//             {
//                 // Print value.
//                 if (is_integral(val))
//                 {
//                     fmt::print("    ");
//                 }
//                 else
//                 {
//                     fmt::print("   *");
//                 }
//                 fmt::print("Agent {:3d}, Val: {:6.4f}, Path:", a, std::abs(val));
//
//                 // Print path.
//                 Bool prev_edgetime_is_fractional = false;
//                 Time t = 0;
//                 for (; t < path.size() - 1; ++t)
//                 {
//                     // Get the nodetime and edgetime.
//                     const auto x = map.get_x(path[t].n);
//                     const auto y = map.get_y(path[t].n);
//                     const NodeTime nt{path[t].n, t};
//                     const EdgeTime et{map.get_undirected_edge(path[t]), t};
//
//                     // Check if fractional.
//                     auto nodetime_is_fractional = false;
//                     if (auto it1 = used_nodetimes.find(nt); it1 != used_nodetimes.end())
//                     {
//                         const auto& agents = it1->second;
//                         if (agents.size() > 1)
//                         {
//                             nodetime_is_fractional = true;
//                         }
//                         // else if (auto it2 = agents.find(a); it2 != agents.end() && !is_integral(it2->second))
//                         // {
//                         //     nodetime_is_fractional = true;
//                         // }
//                     }
//                     auto edgetime_is_fractional = false;
//                     if (auto it1 = used_edgetimes.find(et); it1 != used_edgetimes.end())
//                     {
//                         const auto& agents = it1->second;
//                         if (agents.size() > 1)
//                         {
//                             edgetime_is_fractional = true;
//                         }
//                         // else if (auto it2 = agents.find(a); it2 != agents.end() && !is_integral(it2->second))
//                         // {
//                         //     edgetime_is_fractional = true;
//                         // }
//                     }
//
//                     // Print.
//                     if (nodetime_is_fractional)
//                     {
//                         fmt::print(fmt::emphasis::bold | fg(fmt::terminal_color::blue), "{:>10}", fmt::format("({},{})", x, y));
//                     }
//                     else if (edgetime_is_fractional || prev_edgetime_is_fractional)
//                     {
//                         fmt::print(fmt::emphasis::bold | fg(fmt::terminal_color::red), "{:>10}", fmt::format("({},{})", x, y));
//                     }
//                     else
//                     {
//                         fmt::print("{:>10}", fmt::format("({},{})", x, y));
//                     }
//                     prev_edgetime_is_fractional = edgetime_is_fractional;
//                 }
//                 {
//                     // Get the nodetime.
//                     const auto [x, y] = map.get_xy(path[t].n);
//                     const NodeTime nt{path[t].n, t};
//
//                     // Check if fractional.
//                     auto nodetime_is_fractional = false;
//                     if (auto it1 = used_nodetimes.find(nt); it1 != used_nodetimes.end())
//                     {
//                         const auto& agents = it1->second;
//                         if (agents.size() > 1)
//                         {
//                             nodetime_is_fractional = true;
//                         }
//                         else if (auto it2 = agents.find(a); it2 != agents.end() && !is_integral(it2->second))
//                         {
//                             nodetime_is_fractional = true;
//                         }
//                     }
//
//                     // Print.
//                     if (nodetime_is_fractional)
//                     {
//                         fmt::print(fmt::emphasis::bold | fg(fmt::terminal_color::blue), "{:>10}", fmt::format("({},{})", x, y));
//                     }
//                     else if (prev_edgetime_is_fractional)
//                     {
//                         fmt::print(fmt::emphasis::bold | fg(fmt::terminal_color::red), "{:>10}", fmt::format("({},{})", x, y));
//                     }
//                     else
//                     {
//                         fmt::print("{:>10}", fmt::format("({},{})", x, y));
//                     }
//                 }
//                 println("");
//             }
// }
//
// void print_solution(const Map& map, const Vector<Vector<Edge>>& paths)
// {
//     // Get the problem data.
//     const Agent A = paths.size();
//
//     // Find the makespan.
//     Time makespan = 0;
//     for (Agent a = 0; a < A; ++a)
//     {
//         const auto path = paths[a];
//         makespan = std::max<Time>(makespan, path.size());
//     }
//
//     // Print time header line.
//     fmt::print("              ");
//     for (Time t = 0; t < makespan; ++t)
//     {
//         fmt::print("{:>11d}", t);
//     }
//     println("");
//
//     // Print paths.
//     for (Agent a = 0; a < A; ++a)
//     {
//         // Print value.
//         fmt::print("    ");
//         fmt::print("Agent {:3d}:", a);
//
//         // Print path.
//         const auto path = paths[a];
//         for (Time t = 0; t < path.size(); ++t)
//         {
//             // Get the nodetime and edgetime.
//             const auto x = map.get_x(path[t].n) - 1;
//             const auto y = map.get_y(path[t].n) - 1;
//             const NodeTime nt{path[t].n, t};
//             const EdgeTime et{map.get_undirected_edge(path[t]), t};
//
//             // Print.
//             fmt::print("{:>11}", fmt::format("({},{})", x, y));
//         }
//         println("");
//     }
// }
