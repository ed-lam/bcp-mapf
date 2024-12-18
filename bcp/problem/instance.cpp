// #define PRINT_DEBUG

#include <cmath>
#include <fstream>
#include <sstream>
#include <regex>
#include "problem/instance.h"

struct Line
{
    Node start;
    Node goal;
};

Instance::Instance(const FilePath& scenario_path_, const Agent agent_limit_)
{
    // Check.
    // release_assert(agent_limit > 0, "Using {} agents but must have at least 1 agent", agent_limit);

    // Get instance name.
    scenario_path = scenario_path_;
    name = scenario_path.filename().stem().string();

    // Open scenario file.
    std::ifstream scenario_file;
    scenario_file.open(scenario_path, std::ios::in);
    release_assert(scenario_file.good(), "Cannot find scenario file \"{}\"", scenario_path.string());

    // Read file.
    char buffer[1024];
    scenario_file.getline(buffer, 1024);
    release_assert(strstr(buffer, "version 1"), "Expecting \"version 1\" scenario file format");

    // Read agents data.
    Vector<Line> agent_lines;
    String agent_map_path;
    Position agent_map_width;
    Position agent_map_height;
    Position start_x;
    Position start_y;
    Position goal_x;
    Position goal_y;
    Float tmp;
    String first_agent_map_path;
    while (scenario_file >>
           tmp >>
           agent_map_path >>
           agent_map_width >> agent_map_height >>
           start_x >> start_y >>
           goal_x >> goal_y >>
           tmp)
    {
        // Add padding.
        agent_map_width += 2;
        agent_map_height += 2;
        start_x++;
        start_y++;
        goal_x++;
        goal_y++;

        // Read map.
        if (map.empty())
        {
            // Prepend the directory of the scenario file.
            map_path = scenario_path.parent_path().append(agent_map_path);

            // Store map data for checking.
            first_agent_map_path = agent_map_path;

            // Read map.
            map = Map(map_path);
        }

        // Check.
        const auto a = agent_lines.size();
        const auto start = map.get_n(start_x, start_y);
        const auto goal = map.get_n(goal_x, goal_y);
        release_assert(agent_map_path == first_agent_map_path, "Agent {} uses a different map", a);
        release_assert(agent_map_width == map.width(),
                        "Map width of agent {} does not match actual map size", a);
        release_assert(agent_map_height == map.height(),
                        "Map height of agent {} does not match actual map size", a);
        release_assert(start < map.size() && map[start], "Agent {} starts at an obstacle", a);
        release_assert(goal < map.size() && map[goal], "Agent {} ends at an obstacle", a);

        // Store.
        auto& line = agent_lines.emplace_back();
        line.start = start;
        line.goal = goal;
    }

    // Close file.
    scenario_file.close();

    // Split agents data into agents and requests.
    release_assert(agent_limit_ == -1 || (1 <= agent_limit_ && agent_limit_ <= agent_lines.size()),
                   "Cannot read {} agents from a scenario with only {} agents",
                   agent_limit_, agent_lines.size());
    const auto agent_limit = agent_limit_ == -1 ? agent_lines.size() : agent_limit_;
    for (auto it = agent_lines.begin(); it != agent_lines.begin() + agent_limit; ++it)
    {
        const auto& line = *it;
        auto& agent_data = agents.emplace_back();
        agent_data.start = line.start;
        agent_data.goal = line.goal;
    }
    num_agents = agent_limit;

    // Check.
    debug_assert(num_agents == agents.size());
}
