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

#ifdef USE_LNS2_PRIMAL_HEURISTIC

// #define PRINT_DEBUG

#define HEUR_NAME             "mapf-lns2"
#define HEUR_DESC             "MAPF-LNS2"
#define HEUR_DISPCHAR         'L'
#define HEUR_PRIORITY         20000
#define HEUR_FREQ             0
#define HEUR_FREQOFS          0
#define HEUR_MAXDEPTH         -1
#define HEUR_TIMING           SCIP_HEURTIMING_BEFORENODE
#define HEUR_USESSUBSCIP      FALSE    // Does the heuristic use a secondary SCIP instance?

#include "Heuristic_LNS2.h"
#include "ProblemData.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wreorder"
#pragma GCC diagnostic ignored "-Winit-self"
#include "LNS.h"
#include "AnytimeBCBS.h"
#include "AnytimeEECBS.h"
#include "PIBT/pibt.h"
#pragma GCC diagnostic pop

// Copy method for primal heuristic
// static
// SCIP_DECL_HEURCOPY(heurCopyLNS2)
// {
//     debug_assert(scip);
//     debug_assert(heur);
//     debug_assert(strcmp(SCIPheurGetName(heur), HEUR_NAME) == 0);

//     SCIP_CALL(SCIPincludeHeurLNS(scip));

//     return SCIP_OKAY;
// }

Vector<Vector<Node>> run_lns2(const std::filesystem::path& scenario_path,
                              const std::filesystem::path& map_path,
                              const Int num_agents,
                              const Map& map)
{
    lns::PIBTPPS_option pipp_option;
    pipp_option.windowSize = 5;
    pipp_option.winPIBTSoft = true;

	lns::Instance instance(map_path, scenario_path, num_agents);
    double time_limit = 5;
#ifdef PRINT_DEBUG
    int screen = 1;
#else
    int screen = 0;
#endif
	srand(0);

    const String initAlgo{"PP"};
    const String replanAlgo{"PP"};
    const String destoryStrategy{"Adaptive"};
    const int neighborSize = std::max(std::min(8, num_agents - 1), 1);
    const int maxIterations = 100;
    const bool initLNS = true;
    const String initDestoryStrategy{"Adaptive"};
    const bool sipp = true;
    lns::LNS lns(instance, time_limit,
                 initAlgo,
                 replanAlgo,
                 destoryStrategy,
                 neighborSize,
                 maxIterations,
                 initLNS,
                 initDestoryStrategy,
                 sipp,
                 screen, pipp_option);
    bool succ = lns.run();
#ifdef DEBUG
    if (succ)
        lns.validateSolution();
#endif
    // if (vm.count("output"))
    //     lns.writeResultToFile(vm["output"].as<string>());
    // if (vm.count("stats"))
    //     lns.writeIterStatsToFile(vm["stats"].as<string>());

    Vector<Vector<Node>> paths;
    if (succ)
    {
        int a = 0;
        for (const auto& agent : lns.agents)
        {
            auto& path = paths.emplace_back();
            for (const auto &state : agent.path)
            {
                const auto x = instance.getColCoordinate(state.location);
                const auto y = instance.getRowCoordinate(state.location);
                path.emplace_back(map.get_id(x + 1, y + 1));
            }

            release_assert(agent.id == a);
            ++a;
        }
    }

    return paths;
}

// Execution method of primal heuristic
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_HEUREXEC(heurExecLNS2)
{
    // Print.
    debugln("Starting LNS2 primal heuristic at node {}, depth {}:",
            SCIPnodeGetNumber(SCIPgetCurrentNode(scip)),
            SCIPgetDepth(scip));

    // Initialize.
    *result = SCIP_DIDNOTFIND;

   // Get problem data.
   auto probdata = SCIPgetProbData(scip);
   const auto N = SCIPprobdataGetN(probdata);
   const auto& map = SCIPprobdataGetMap(probdata);
   const auto& agents = SCIPprobdataGetAgentsData(probdata);

    // Run LNS.
    const auto& scenario_path = SCIPprobdataGetScenarioPath(probdata);
    const auto& map_path = SCIPprobdataGetMapPath(probdata);
    const auto paths = run_lns2(scenario_path, map_path, N, map);
    if (!paths.empty())
    {
        // Create solution object.
        SCIP_SOL* sol;
        SCIP_CALL(SCIPcreateSol(scip, &sol, heur));
        for (Agent a = 0; a < N; ++a)
        {
            // Check.
            release_assert(paths[a].front() == agents[a].start);
            release_assert(paths[a].back() == agents[a].goal);

            // Get the solution.
            Vector<Edge> path;
            for (auto it = paths[a].begin(); it != paths[a].end(); ++it)
            {
                const auto d = it != paths[a].end() - 1 ?
                            map.get_direction(*it, *(it + 1)) :
                            Direction::INVALID;
                path.push_back(Edge{*it, d});
            }

            // Print.
            debugln("      Found path with length {} ({})",
                    path.size(),
                    format_path(probdata, path.size(), path.data()));

            // Add column.
            SCIP_VAR* var = nullptr;
            SCIP_CALL(SCIPprobdataAddInitialVar(scip,
                                                probdata,
                                                a,
                                                path.size(),
                                                path.data(),
                                                &var));
            debug_assert(var);
            SCIP_CALL(SCIPsetSolVal(scip, sol, var, 1.0));
        }

        // Inject solution.
        SCIP_Bool success;
        SCIP_CALL(SCIPtrySol(scip, sol, FALSE, FALSE, FALSE, TRUE, TRUE, &success));
        if (success)
        {
            *result = SCIP_FOUNDSOL;
        }

        // Deallocate.
        SCIP_CALL(SCIPfreeSol(scip, &sol));
    }

    // Done.
    debugln("");
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Create the LNS primal heuristic and include it in SCIP
SCIP_RETCODE SCIPincludeHeurLNS2(
    SCIP* scip
)
{
    // Create heuristic.
    SCIP_HEUR* heur;
    SCIP_CALL(SCIPincludeHeurBasic(scip,
                                   &heur,
                                   HEUR_NAME,
                                   HEUR_DESC,
                                   HEUR_DISPCHAR,
                                   HEUR_PRIORITY,
                                   HEUR_FREQ,
                                   HEUR_FREQOFS,
                                   HEUR_MAXDEPTH,
                                   HEUR_TIMING,
                                   HEUR_USESSUBSCIP,
                                   heurExecLNS2,
                                   nullptr));
    debug_assert(heur);

    // Done.
    return SCIP_OKAY;
}

#endif
