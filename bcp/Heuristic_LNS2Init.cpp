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

#ifdef USE_LNS2_INIT_PRIMAL_HEURISTIC

// #define PRINT_DEBUG
#define ENTER_CONFLICTING_PATH_TO_LNS2

#define HEUR_NAME             "lns2init"
#define HEUR_DESC             "MAPF-LNS2 initialization"
#define HEUR_DISPCHAR         'I'
#define HEUR_PRIORITY         20001
#define HEUR_FREQ             0
#define HEUR_FREQOFS          0
#define HEUR_MAXDEPTH         -1
#define HEUR_TIMING           SCIP_HEURTIMING_BEFORENODE
#define HEUR_USESSUBSCIP      FALSE    // Does the heuristic use a secondary SCIP instance?

#include "Heuristic_LNS2Init.h"
#include "problem/problem.h"
#include "problem/variable_data.h"
#include "Clock.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wreorder"
#pragma GCC diagnostic ignored "-Winit-self"
#include "lns2/inc/LNS.h"
#include "lns2/inc/AnytimeBCBS.h"
#include "lns2/inc/AnytimeEECBS.h"
#include "lns2/inc/PIBT/pibt.h"
#pragma GCC diagnostic pop

struct LNS2InitData
{
    lns::PIBTPPS_option pipp_option;
    lns::Instance instance;
    lns::LNS lns;
    bool called;

    LNS2InitData(const String& scenario_path, const String& map_path, const Agent N) :
        pipp_option(create_pipp_option()),
        instance(map_path, scenario_path, N),
        lns(create_init_lns(pipp_option, instance, N)),
        called(false)
    {
    }
    static lns::PIBTPPS_option create_pipp_option()
    {
        lns::PIBTPPS_option pipp_option;
        pipp_option.windowSize = 5;
        pipp_option.winPIBTSoft = true;
        return pipp_option;
    }
    static lns::LNS create_init_lns(const lns::PIBTPPS_option& pipp_option,
                                    const lns::Instance& instance,
                                    const Agent N)
    {
#ifdef PRINT_DEBUG
        const int screen = 3;
#else
        const int screen = 0;
#endif

        const String initAlgo = "PP";
        const String replanAlgo{"PP"};
        const String destoryStrategy{"Adaptive"};
        const int neighborSize = std::max(std::min(8, N - 1), 1);
        const int maxIterations = 1000000;
        const bool initLNS = true;
        const String initDestoryStrategy{"Adaptive"};
        const bool sipp = true;
        return lns::LNS(instance,
                        LNS2_INIT_TIME_LIMIT,
                        initAlgo,
                        replanAlgo,
                        destoryStrategy,
                        neighborSize,
                        maxIterations,
                        initLNS,
                        initDestoryStrategy,
                        sipp,
                        screen,
                        pipp_option);
    }
};

// Copy method for primal heuristic
// static
// SCIP_DECL_HEURCOPY(heurCopyLNS2Init)
// {
//     debug_assert(scip);
//     debug_assert(heur);
//     debug_assert(strcmp(SCIPheurGetName(heur), HEUR_NAME) == 0);

//     SCIP_CALL(SCIPincludeHeurLNS(scip));

//     return SCIP_OKAY;
// }

// Free method for primal heuristic
static
SCIP_DECL_HEURFREE(heurFreeLNS2Init)
{
    debug_assert(scip);
    debug_assert(heur);
    debug_assert(strcmp(SCIPheurGetName(heur), HEUR_NAME) == 0);

    auto lns_data = reinterpret_cast<LNS2InitData*>(SCIPheurGetData(heur));
    SCIPfreeBlockMemory(scip, &lns_data);

    return SCIP_OKAY;
}

// Execution method of primal heuristic
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_HEUREXEC(heurExecLNS2Init)
{
    // Skip if previously called.
    auto lns_data = reinterpret_cast<LNS2InitData*>(SCIPheurGetData(heur));
    if (!lns_data->called)
    {
        // Print.
        debugln("Starting LNS2 initialization primal heuristic at node {}, depth {}, node LB {}:",
                SCIPnodeGetNumber(SCIPgetCurrentNode(scip)),
                SCIPgetDepth(scip),
                SCIPgetNodeLowerbound(scip, SCIPgetCurrentNode(scip)));
    }
    else
    {
        debugln("Skipping LNS2 initialization primal heuristic at node {}, depth {}, node LB {}",
                SCIPnodeGetNumber(SCIPgetCurrentNode(scip)),
                SCIPgetDepth(scip),
                SCIPgetNodeLowerbound(scip, SCIPgetCurrentNode(scip)));
        return SCIP_OKAY;
    }

    // Initialize.
    *result = SCIP_DIDNOTFIND;

    // Get problem data.
    auto probdata = SCIPgetProbData(scip);
    const auto& map = SCIPprobdataGetMap(probdata);
    const auto& agents = SCIPprobdataGetAgentsData(probdata);

    // Get LNS2 object.
    const auto& instance = lns_data->instance;
    auto& lns = lns_data->lns;
    lns.reset();

    // Run.
    const int lb = std::max<SCIP_Real>(SCIPceil(scip, SCIPgetNodeLowerbound(scip, SCIPgetCurrentNode(scip))), 0.);
    const auto succ = lns.run(lb);
    lns_data->called = true;

    // Check.
#ifdef DEBUG
    if (succ)
    {
        lns.validateSolution();
    }
#endif

    // Get solution.
    if (succ && lns.sum_of_costs < SCIPgetUpperbound(scip))
    {
        // Create solution object.
        SCIP_SOL* sol;
        SCIP_CALL(SCIPcreateSol(scip, &sol, heur));
        debugln("Found solution with cost {}:", lns.sum_of_costs);

        // Create paths.
        Vector<Edge> path;
        int a = 0;
        for (const auto& agent : lns.agents)
        {
            // Get the path from LNS2.
            release_assert(agent.id == a);
            path.clear();
            for (const auto &state : agent.path)
            {
                const auto x = instance.getColCoordinate(state.location);
                const auto y = instance.getRowCoordinate(state.location);
                path.emplace_back(Edge{map.get_id(x + 1, y + 1), Direction::INVALID});
            }
            for (Time t = 0; t < static_cast<Time>(path.size()) - 1; ++t)
            {
                path[t].d = map.get_direction(path[t].n, path[t + 1].n);
            }
            release_assert(path.front().n == agents[a].start);
            release_assert(path.back().n == agents[a].goal);

            // Add column.
            SCIP_VAR* var = nullptr;
            SCIP_CALL(SCIPprobdataAddHeuristicVar(scip,
                                                  probdata,
                                                  a,
                                                  path.size(),
                                                  path.data(),
                                                  &var));
            debug_assert(var);
            SCIP_CALL(SCIPsetSolVal(scip, sol, var, 1.0));

            // Print.
            debugln("Agent {:4d}: {}", a, format_path_spaced(probdata, path.size(), path.data()));

            // Advance to next agent.
            ++a;
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
SCIP_RETCODE SCIPincludeHeurLNS2Init(
    SCIP* scip
)
{
    // Set random seed.
    srand(0);

    // Get problem data.
    auto probdata = SCIPgetProbData(scip);
    const auto& scenario_path = SCIPprobdataGetScenarioPath(probdata);
    const auto& map_path = SCIPprobdataGetMapPath(probdata);
    const auto N = SCIPprobdataGetN(probdata);

    // Create LNS2 data.
    LNS2InitData* lns_data = nullptr;
    SCIP_CALL(SCIPallocBlockMemory(scip, &lns_data));
    debug_assert(lns_data);
    new(lns_data) LNS2InitData(scenario_path, map_path, N);

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
                                   heurExecLNS2Init,
                                   reinterpret_cast<SCIP_HeurData*>(lns_data)));
    debug_assert(heur);

    // Set callbacks.
    SCIP_CALL(SCIPsetHeurFree(scip, heur, heurFreeLNS2Init));

    // Done.
    return SCIP_OKAY;
}

#endif
