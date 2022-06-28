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

#ifdef USE_EECBS_PRIMAL_HEURISTIC

// #define PRINT_DEBUG

#define HEUR_NAME             "eecbs"
#define HEUR_DESC             "MAPF-EECBS"
#define HEUR_DISPCHAR         'E'
#define HEUR_PRIORITY         19999
#define HEUR_FREQ             0
#define HEUR_FREQOFS          0
#define HEUR_MAXDEPTH         -1
#define HEUR_TIMING           SCIP_HEURTIMING_BEFORENODE
#define HEUR_USESSUBSCIP      FALSE    // Does the heuristic use a secondary SCIP instance?

#include "Heuristic_EECBS.h"
#include "ProblemData.h"
#include "VariableData.h"
#include "Clock.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wreorder"
#pragma GCC diagnostic ignored "-Winit-self"
#include "eecbs/inc/ECBS.h"
#pragma GCC diagnostic pop

struct EECBSData
{
    eecbs::Instance instance;
    eecbs::ECBS ecbs;
    bool called;

    EECBSData(const String& scenario_path, const String& map_path, const Agent N) :
        instance(map_path, scenario_path, N),
        ecbs(create_ecbs(instance)),
        called(false)
    {
    }
    static eecbs::ECBS create_ecbs(const eecbs::Instance& instance)
    {
#ifdef PRINT_DEBUG
        const int screen = 2;
#else
        const int screen = 0;
#endif

        eecbs::ECBS ecbs(instance, false, screen);
        ecbs.setPrioritizeConflicts(true);
        ecbs.setDisjointSplitting(false);
        ecbs.setBypass(true);
        ecbs.setRectangleReasoning(true);
        ecbs.setCorridorReasoning(true);
        ecbs.setHeuristicType(eecbs::heuristics_type::WDG, eecbs::heuristics_type::GLOBAL);
        ecbs.setTargetReasoning(true);
        ecbs.setMutexReasoning(false);
        ecbs.setConflictSelectionRule(eecbs::conflict_selection::EARLIEST);
        ecbs.setNodeSelectionRule(eecbs::node_selection::NODE_CONFLICTPAIRS);
        ecbs.setSavingStats(false);
        ecbs.setHighLevelSolver(eecbs::high_level_solver_type::EES, EECBS_SUBOPTIMALITY);
        return ecbs;
    }
};

// Copy method for primal heuristic
// static
// SCIP_DECL_HEURCOPY(heurCopyEECBS)
// {
//     debug_assert(scip);
//     debug_assert(heur);
//     debug_assert(strcmp(SCIPheurGetName(heur), HEUR_NAME) == 0);

//     SCIP_CALL(SCIPincludeHeurEECBS(scip));

//     return SCIP_OKAY;
// }

// Free method for primal heuristic
static
SCIP_DECL_HEURFREE(heurFreeEECBS)
{
    debug_assert(scip);
    debug_assert(heur);
    debug_assert(strcmp(SCIPheurGetName(heur), HEUR_NAME) == 0);

    auto eecbs_data = reinterpret_cast<EECBSData*>(SCIPheurGetData(heur));
    SCIPfreeBlockMemory(scip, &eecbs_data);

    return SCIP_OKAY;
}

// Execution method of primal heuristic
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_HEUREXEC(heurExecEECBS)
{
    // Skip if previously called.
    auto eecbs_data = reinterpret_cast<EECBSData*>(SCIPheurGetData(heur));
    if (!eecbs_data->called)
    {
        // Print.
        debugln("Starting EECBS primal heuristic at node {}, depth {}, node LB {}:",
                SCIPnodeGetNumber(SCIPgetCurrentNode(scip)),
                SCIPgetDepth(scip),
                SCIPgetNodeLowerbound(scip, SCIPgetCurrentNode(scip)));
    }
    else
    {
        debugln("Skipping EECBS primal heuristic at node {}, depth {}, node LB {}",
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
    const auto N = SCIPprobdataGetN(probdata);

    // Get ECBS object.
    const auto& instance = eecbs_data->instance;
    auto& ecbs = eecbs_data->ecbs;

    // Run.
    const int lowerbound = std::max<SCIP_Real>(SCIPceil(scip, SCIPgetNodeLowerbound(scip, SCIPgetCurrentNode(scip))), 0.);
	srand(0);
    ecbs.clear();
    ecbs.solve(EECBS_TIME_LIMIT, lowerbound);

    // Get solution.
    debug_assert(!ecbs.solution_found || ecbs.solution_cost >= 0);
    if (ecbs.solution_found && ecbs.solution_cost < SCIPgetUpperbound(scip))
    {
        // Create solution object.
        SCIP_SOL* sol;
        SCIP_CALL(SCIPcreateSol(scip, &sol, heur));
        debugln("Found solution with cost {}:", ecbs.solution_cost);

        // Create paths.
        Vector<Edge> path;
        const auto& paths = ecbs.getPaths();
        for (Agent a = 0; a < N; ++a)
        {
            // Get the path from EECBS.
            path.clear();
            for (const auto& t : *paths[a])
            {
                const auto x = instance.getColCoordinate(t.location);
                const auto y = instance.getRowCoordinate(t.location);
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

    // Finish.
    ecbs.clearSearchEngines();
    eecbs_data->called = true;

    // Done.
    debugln("");
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Create the LNS primal heuristic and include it in SCIP
SCIP_RETCODE SCIPincludeHeurEECBS(
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

    // Create EECBS data.
    EECBSData* eecbs_data = nullptr;
    SCIP_CALL(SCIPallocBlockMemory(scip, &eecbs_data));
    debug_assert(eecbs_data);
    new(eecbs_data) EECBSData(scenario_path, map_path, N);

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
                                   heurExecEECBS,
                                   reinterpret_cast<SCIP_HeurData*>(eecbs_data)));
    debug_assert(heur);

    // Set callbacks.
    SCIP_CALL(SCIPsetHeurFree(scip, heur, heurFreeEECBS));

    // Done.
    return SCIP_OKAY;
}

#endif
