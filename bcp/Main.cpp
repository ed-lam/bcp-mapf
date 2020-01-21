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
#include "scip/scip.h"
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
    SCIP_Real time_limit = 0;
    Agent agents_limit = std::numeric_limits<Agent>::max();
    try
    {
        // Create program options.
        cxxopts::Options options(argv[0],
                                 "BCP-MAPF - branch-and-cut-and-price solver for "
                                 "multi-agent path finding");
        options.positional_help("instance_file").show_positional_help();
        options.add_options()
            ("help", "Print help")
            ("f,file", "Path to instance file", cxxopts::value<Vector<String>>())
            ("t,time-limit", "Time limit in seconds", cxxopts::value<SCIP_Real>())
            ("a,agents-limit", "Read first N agents only", cxxopts::value<int>())
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
            instance_file = result["file"].as<Vector<String>>().at(0);
        }

        // Get time limit.
        if (result.count("time-limit"))
        {
            time_limit = result["time-limit"].as<SCIP_Real>();
        }

        // Get agents limit.
        if (result.count("agents-limit"))
        {
            agents_limit = result["agents-limit"].as<int>();
        }
    }
    catch (const cxxopts::OptionException& e)
    {
        err("{}", e.what());
    }

    // Print.
    println("Branch-and-cut-and-price solver for multi-agent path finding");
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
#ifdef USE_CORRIDOR_CONFLICTS
    println("Using corridor conflict constraints");
#endif
#ifdef USE_WAITDELAY_CONFLICTS
    println("Using wait-delay conflict constraints");
#endif
#ifdef USE_ENTRYEXIT_CONFLICTS
    println("Using entry-exit conflict constraints");
#endif
#ifdef USE_TWOEDGE_CONFLICTS
    println("Using two-edge conflict constraints");
#endif
#ifdef USE_GOAL_CONFLICTS
    println("Using goal conflict constraints");
#endif
#endif
    println("");

    // Initialize SCIP.
    SCIP* scip = nullptr;
    SCIP_CALL(SCIPcreate(&scip));

    // Set up plugins.
    {
        // Include default SCIP plugins.
        SCIP_CALL(SCIPincludeDefaultPlugins(scip));

        // Disable parallel solve.
        SCIP_CALL(SCIPsetIntParam(scip, "parallel/maxnthreads", 1));
        SCIP_CALL(SCIPsetIntParam(scip, "lp/threads", 1));

        // Set parameters.
        SCIP_CALL(SCIPsetIntParam(scip, "presolving/maxrounds", 0));
        SCIP_CALL(SCIPsetIntParam(scip, "propagating/rootredcost/freq", -1));
        SCIP_CALL(SCIPsetIntParam(scip, "separating/maxaddrounds", -1));
        SCIP_CALL(SCIPsetIntParam(scip, "separating/maxstallrounds", 5));
        SCIP_CALL(SCIPsetIntParam(scip, "separating/maxstallroundsroot", 20));
        SCIP_CALL(SCIPsetIntParam(scip, "separating/cutagelimit", -1));

        // Turn off all separation algorithms.
        SCIP_CALL(SCIPsetSeparating(scip, SCIP_PARAMSETTING_OFF, TRUE));

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
    release_assert(agents_limit > 0, "Cannot limit to {} number of agents", agents_limit);
    SCIP_CALL(read_instance(scip, instance_file.c_str(), agents_limit));

    // Set time limit.
    if (time_limit > 0)
    {
        SCIP_CALL(SCIPsetRealParam(scip, "limits/time", time_limit));
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
