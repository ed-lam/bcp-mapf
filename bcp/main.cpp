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

#include "Includes.h"
#include "Reader.h"
#include "Output.h"

#include "scip/scipshell.h"
#include "scip/scipdefplugins.h"
#include "cxxopts.hpp"

static
SCIP_RETCODE start_solver(
    int argc,      // Number of shell parameters
    char** argv    // Array with shell parameters
)
{
    // Parse program options.
    String instance_file;
    Agent agent_limit = std::numeric_limits<Agent>::max();
    SCIP_Real time_limit = 0;
    SCIP_Longint node_limit = 0;
    SCIP_Real gap_limit = 0;
    try
    {
        // Create program options.
        cxxopts::Options options(argv[0],
                                 "BCP-MAPF - branch-and-cut-and-price for multi-agent path finding");
        options.positional_help("instance_file").show_positional_help();
        options.add_options()
            ("help", "Print help")
            ("f,file", "Path to instance file", cxxopts::value<String>())
            ("a,agent-limit", "Read the first several agents only", cxxopts::value<Agent>())
            ("t,time-limit", "Time limit in seconds", cxxopts::value<SCIP_Real>())
            ("n,node-limit", "Maximum number of branch-and-bound nodes", cxxopts::value<SCIP_Longint>())
            ("g,gap-limit", "Solve to an optimality gap", cxxopts::value<SCIP_Real>())
        ;
        options.parse_positional({"file"});

        // Parse options.
        auto result = options.parse(argc, argv);

        // Print help.
        if (result.count("help") || !result.count("file"))
        {
            println("{}", options.help());
            exit(0);
        }

        // Get path to instance.
        if (result.count("file"))
        {
            instance_file = result["file"].as<String>();
        }

        // Get agent limit.
        if (result.count("agent-limit"))
        {
            agent_limit = result["agent-limit"].as<Agent>();
        }

        // Get time limit.
        if (result.count("time-limit"))
        {
            time_limit = result["time-limit"].as<SCIP_Real>();
        }

        // Get node limit.
        if (result.count("node-limit"))
        {
            node_limit = result["node-limit"].as<SCIP_Longint>();
        }

        // Get optimality gap limit.
        if (result.count("gap-limit"))
        {
            gap_limit = result["gap-limit"].as<SCIP_Real>();
        }
    }
    catch (const cxxopts::exceptions::exception& e)
    {
        err("{}", e.what());
    }

    // Print.
    println("Branch-and-cut-and-price for multi-agent path finding");
    println("Edward Lam <ed@ed-lam.com>");
    println("Monash University, Melbourne, Australia");
#ifdef DEBUG
    println("Compiled in debug mode");
#ifdef USE_WAITEDGE_CONFLICTS
    println("Using wait-edge conflict constraints");
#endif
#ifdef USE_RECTANGLE_KNAPSACK_CONFLICTS
    println("Using rectangle knapsack conflict constraints");
#endif
#ifdef USE_RECTANGLE_CLIQUE_CONFLICTS
    println("Using rectangle clique conflict constraints");
#endif
#if !defined(USE_WAITCORRIDOR_CONFLICTS) && defined(USE_CORRIDOR_CONFLICTS)
    println("Using corridor conflict constraints");
#endif
#ifdef USE_WAITCORRIDOR_CONFLICTS
    println("Using wait corridor conflict constraints");
#endif
#ifdef USE_STEPASIDE_CONFLICTS
    println("Using step aside conflict constraints");
#endif
#ifdef USE_WAITDELAY_CONFLICTS
    println("Using wait delay conflict constraints");
#endif
#ifdef USE_EXITENTRY_CONFLICTS
    println("Using exit entry conflict constraints");
#endif
#if !defined(USE_WAITTWOEDGE_CONFLICTS) && defined(USE_TWOEDGE_CONFLICTS)
    println("Using two edge conflict constraints");
#endif
#ifdef USE_WAITTWOEDGE_CONFLICTS
    println("Using wait two edge conflict constraints");
#endif
#ifdef USE_AGENTWAITEDGE_CONFLICTS
    println("Using agent wait edge conflict constraints");
#endif
#ifdef USE_TWOVERTEX_CONFLICTS
    println("Using two vertex conflict constraints");
#endif
#ifdef USE_THREEVERTEX_CONFLICTS
    println("Using three vertex conflict constraints");
#endif
#ifdef USE_FOUREDGE_CONFLICTS
    println("Using four edge conflict constraints");
#endif
#ifdef USE_FIVEEDGE_CONFLICTS
    println("Using five edge conflict constraints");
#endif
#ifdef USE_SIXEDGE_CONFLICTS
    println("Using six edge conflict constraints");
#endif
#ifdef USE_VERTEX_FOUREDGE_CONFLICTS
    println("Using vertex four edge conflict constraints");
#endif
#ifdef USE_CLIQUE_CONFLICTS
    println("Using clique conflict constraints");
#endif
#ifdef USE_GOAL_CONFLICTS
    println("Using goal conflict constraints");
#endif
#ifdef USE_PATH_LENGTH_NOGOODS
    println("Using path length nogoods");
#endif
#endif
    println("");

    // Initialize SCIP.
    SCIP* scip = nullptr;
    SCIP_CALL(SCIPcreate(&scip));

    // Set up plugins.
    {
        // Include some default SCIP plugins.
        {
            SCIP_CALL( SCIPincludeConshdlrLinear(scip) ); /* linear must be before its specializations due to constraint upgrading */
            SCIP_CALL( SCIPincludeConshdlrIndicator(scip) );
            SCIP_CALL( SCIPincludeConshdlrIntegral(scip) );
            SCIP_CALL( SCIPincludeConshdlrKnapsack(scip) );
            SCIP_CALL( SCIPincludeConshdlrSetppc(scip) );

            SCIP_CALL( SCIPincludeNodeselBfs(scip) );
            SCIP_CALL( SCIPincludeNodeselBreadthfirst(scip) );
            SCIP_CALL( SCIPincludeNodeselDfs(scip) );
            SCIP_CALL( SCIPincludeNodeselEstimate(scip) );
            SCIP_CALL( SCIPincludeNodeselHybridestim(scip) );
            SCIP_CALL( SCIPincludeNodeselRestartdfs(scip) );
            SCIP_CALL( SCIPincludeNodeselUct(scip) );

            SCIP_CALL( SCIPincludeEventHdlrEstim(scip) );
            SCIP_CALL( SCIPincludeEventHdlrSolvingphase(scip) );

            SCIP_CALL( SCIPincludeHeurActconsdiving(scip) );
            SCIP_CALL( SCIPincludeHeurAdaptivediving(scip) );
            SCIP_CALL( SCIPincludeHeurBound(scip) );
            SCIP_CALL( SCIPincludeHeurClique(scip) );
            SCIP_CALL( SCIPincludeHeurCoefdiving(scip) );
            SCIP_CALL( SCIPincludeHeurCompletesol(scip) );
            SCIP_CALL( SCIPincludeHeurConflictdiving(scip) );
            SCIP_CALL( SCIPincludeHeurCrossover(scip) );
            SCIP_CALL( SCIPincludeHeurDins(scip) );
            SCIP_CALL( SCIPincludeHeurDistributiondiving(scip) );
            SCIP_CALL( SCIPincludeHeurDualval(scip) );
            SCIP_CALL( SCIPincludeHeurFarkasdiving(scip) );
            SCIP_CALL( SCIPincludeHeurFeaspump(scip) );
            SCIP_CALL( SCIPincludeHeurFixandinfer(scip) );
            SCIP_CALL( SCIPincludeHeurFracdiving(scip) );
            SCIP_CALL( SCIPincludeHeurGins(scip) );
            SCIP_CALL( SCIPincludeHeurGuideddiving(scip) );
            SCIP_CALL( SCIPincludeHeurZeroobj(scip) );
            SCIP_CALL( SCIPincludeHeurIndicator(scip) );
            SCIP_CALL( SCIPincludeHeurIntdiving(scip) );
            SCIP_CALL( SCIPincludeHeurIntshifting(scip) );
            SCIP_CALL( SCIPincludeHeurLinesearchdiving(scip) );
            SCIP_CALL( SCIPincludeHeurLocalbranching(scip) );
            SCIP_CALL( SCIPincludeHeurLocks(scip) );
            SCIP_CALL( SCIPincludeHeurLpface(scip) );
            SCIP_CALL( SCIPincludeHeurAlns(scip) );
            SCIP_CALL( SCIPincludeHeurNlpdiving(scip) );
            SCIP_CALL( SCIPincludeHeurMutation(scip) );
            SCIP_CALL( SCIPincludeHeurMultistart(scip) );
            SCIP_CALL( SCIPincludeHeurMpec(scip) );
            SCIP_CALL( SCIPincludeHeurObjpscostdiving(scip) );
            SCIP_CALL( SCIPincludeHeurOctane(scip) );
            SCIP_CALL( SCIPincludeHeurOfins(scip) );
            SCIP_CALL( SCIPincludeHeurOneopt(scip) );
            SCIP_CALL( SCIPincludeHeurPADM(scip) );
            SCIP_CALL( SCIPincludeHeurProximity(scip) );
            SCIP_CALL( SCIPincludeHeurPscostdiving(scip) );
            SCIP_CALL( SCIPincludeHeurRandrounding(scip) );
            SCIP_CALL( SCIPincludeHeurRens(scip) );
            SCIP_CALL( SCIPincludeHeurReoptsols(scip) );
            SCIP_CALL( SCIPincludeHeurRepair(scip) );
            SCIP_CALL( SCIPincludeHeurRins(scip) );
            SCIP_CALL( SCIPincludeHeurRootsoldiving(scip) );
            SCIP_CALL( SCIPincludeHeurRounding(scip) );
            SCIP_CALL( SCIPincludeHeurShiftandpropagate(scip) );
            SCIP_CALL( SCIPincludeHeurShifting(scip) );
            SCIP_CALL( SCIPincludeHeurSimplerounding(scip) );
            SCIP_CALL( SCIPincludeHeurSubNlp(scip) );
            SCIP_CALL( SCIPincludeHeurTrivial(scip) );
            SCIP_CALL( SCIPincludeHeurTrivialnegation(scip) );
            SCIP_CALL( SCIPincludeHeurTrustregion(scip) );
            SCIP_CALL( SCIPincludeHeurTrySol(scip) );
            SCIP_CALL( SCIPincludeHeurTwoopt(scip) );
            SCIP_CALL( SCIPincludeHeurUndercover(scip) );
            SCIP_CALL( SCIPincludeHeurVbounds(scip) );
            SCIP_CALL( SCIPincludeHeurVeclendiving(scip) );
            SCIP_CALL( SCIPincludeHeurZirounding(scip) );

            SCIP_CALL( SCIPincludeDispDefault(scip) );
            SCIP_CALL( SCIPincludeTableDefault(scip) );

            SCIP_CALL( SCIPincludeConcurrentScipSolvers(scip) );
        }
        // SCIP_CALL(SCIPincludeDefaultPlugins(scip));

        // Disable parallel solve.
        SCIP_CALL(SCIPsetIntParam(scip, "parallel/maxnthreads", 1));
        SCIP_CALL(SCIPsetIntParam(scip, "lp/threads", 1));

        // Set parameters.
        SCIP_CALL(SCIPsetIntParam(scip, "presolving/maxrounds", 0));
        // SCIP_CALL(SCIPsetIntParam(scip, "propagating/rootredcost/freq", -1));
        SCIP_CALL(SCIPsetIntParam(scip, "separating/maxaddrounds", -1));
        SCIP_CALL(SCIPsetIntParam(scip, "separating/maxstallrounds", 5));
        SCIP_CALL(SCIPsetIntParam(scip, "separating/maxstallroundsroot", 20));
        SCIP_CALL(SCIPsetIntParam(scip, "separating/cutagelimit", -1));

        // Turn off all separation algorithms.
        SCIP_CALL(SCIPsetSeparating(scip, SCIP_PARAMSETTING_OFF, TRUE));

        // Set node selection rule.
        SCIP_CALL(SCIPsetIntParam(scip, "nodeselection/bfs/stdpriority", 0));
        SCIP_CALL(SCIPsetIntParam(scip, "nodeselection/bfs/memsavepriority", 0));
        SCIP_CALL(SCIPsetIntParam(scip, "nodeselection/breadthfirst/stdpriority", 0));
        SCIP_CALL(SCIPsetIntParam(scip, "nodeselection/breadthfirst/memsavepriority", 0));
        SCIP_CALL(SCIPsetIntParam(scip, "nodeselection/dfs/stdpriority", 0));
        SCIP_CALL(SCIPsetIntParam(scip, "nodeselection/dfs/memsavepriority", 0));
        SCIP_CALL(SCIPsetIntParam(scip, "nodeselection/estimate/stdpriority", 0));
        SCIP_CALL(SCIPsetIntParam(scip, "nodeselection/estimate/memsavepriority", 0));
        SCIP_CALL(SCIPsetIntParam(scip, "nodeselection/hybridestim/stdpriority", 0));
        SCIP_CALL(SCIPsetIntParam(scip, "nodeselection/hybridestim/memsavepriority", 0));
        SCIP_CALL(SCIPsetIntParam(scip, "nodeselection/restartdfs/stdpriority", 0));
        SCIP_CALL(SCIPsetIntParam(scip, "nodeselection/restartdfs/memsavepriority", 0));
        SCIP_CALL(SCIPsetIntParam(scip, "nodeselection/uct/stdpriority", 0));
        SCIP_CALL(SCIPsetIntParam(scip, "nodeselection/uct/memsavepriority", 0));
#ifdef USE_BEST_FIRST_NODE_SELECTION
        SCIP_CALL(SCIPsetIntParam(scip, "nodeselection/bfs/stdpriority", 500000));
        SCIP_CALL(SCIPsetIntParam(scip, "nodeselection/bfs/memsavepriority", 500000));
#endif
#ifdef USE_DEPTH_FIRST_NODE_SELECTION
        SCIP_CALL(SCIPsetIntParam(scip, "nodeselection/dfs/stdpriority", 500000));
        SCIP_CALL(SCIPsetIntParam(scip, "nodeselection/dfs/memsavepriority", 500000));
#endif
#ifdef USE_BEST_ESTIMATE_NODE_SELECTION
        SCIP_CALL(SCIPsetIntParam(scip, "nodeselection/estimate/stdpriority", 500000));
        SCIP_CALL(SCIPsetIntParam(scip, "nodeselection/estimate/memsavepriority", 500000));
#endif
#ifdef USE_HYBRID_ESTIMATE_BOUND_NODE_SELECTION
        SCIP_CALL(SCIPsetIntParam(scip, "nodeselection/hybridestim/stdpriority", 500000));
        SCIP_CALL(SCIPsetIntParam(scip, "nodeselection/hybridestim/memsavepriority", 500000));
#endif
#ifdef USE_RESTART_DEPTH_FIRST_NODE_SELECTION
        SCIP_CALL(SCIPsetIntParam(scip, "nodeselection/restartdfs/stdpriority", 500000));
        SCIP_CALL(SCIPsetIntParam(scip, "nodeselection/restartdfs/memsavepriority", 500000));
#endif

        // Turn on aggressive primal heuristics.
        SCIP_CALL(SCIPsetHeuristics(scip, SCIP_PARAMSETTING_AGGRESSIVE, TRUE));

        // Turn off some primal heuristics.
        {
            const auto nheurs = SCIPgetNHeurs(scip);
            auto heurs = SCIPgetHeurs(scip);
            for (Int idx = 0; idx < nheurs; ++idx)
            {
                auto heur = heurs[idx];
                const String name(SCIPheurGetName(heur));
                if (name == "alns" ||
                    name == "bound" ||
                    name == "coefdiving" ||
                    name == "crossover" ||
                    name == "dins" ||
                    name == "fixandinfer" ||
                    name == "gins" ||
                    name == "guideddiving" ||
                    name == "intdiving" ||
                    name == "localbranching" ||
                    name == "locks" ||
                    name == "mutation" ||
                    name == "oneopt" ||
                    name == "rens" ||
                    name == "repair" ||
                    name == "rins" ||
                    name == "trivial" ||
                    name == "zeroobj" ||
                    name == "zirounding" ||
                    name == "proximity" || // Buggy
                    name == "twoopt")      // Buggy
                {
                    SCIPheurSetFreq(heur, -1);
                }
            }
        }
    }

    // Read instance.
    release_assert(agent_limit > 0, "Cannot limit to {} number of agents", agent_limit);
    SCIP_CALL(read_instance(scip, instance_file.c_str(), agent_limit));

    // Set time limit.
    if (time_limit > 0)
    {
        SCIP_CALL(SCIPsetRealParam(scip, "limits/time", time_limit));
    }

    // Set node limit.
    if (node_limit > 0)
    {
        SCIP_CALL(SCIPsetLongintParam(scip, "limits/nodes", node_limit));
    }

    // Set optimality gap limit.
    if (gap_limit > 0)
    {
        SCIP_CALL(SCIPsetRealParam(scip, "limits/gap", gap_limit));
    }

    // Solve.
    SCIP_CALL(SCIPsolve(scip));

    // Output.
    {
        // Print.
        println("");
        SCIP_CALL(SCIPprintStatistics(scip, NULL));

        // Write best solution to file.
        SCIP_CALL(write_best_solution(scip));
    }

    // Free memory.
    SCIP_CALL(SCIPfree(&scip));

    // Check if memory is leaked.
    BMScheckEmptyMemory();

    // Done.
    return SCIP_OKAY;
}

int main(int argc, char** argv)
{
    const SCIP_RETCODE retcode = start_solver(argc, argv);
    if (retcode != SCIP_OKAY)
    {
        SCIPprintError(retcode);
        return -1;
    }
    return 0;
}
