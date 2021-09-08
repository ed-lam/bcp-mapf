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

//#define PRINT_DEBUG

#ifdef DEBUG
//#define MAKE_NAMES
#endif

#ifndef DEBUG
#define REMOVE_PADDING
#endif

#include "ProblemData.h"
#include "VariableData.h"
#include "Pricer_TruffleHog.h"
#include "scip/cons_setppc.h"
#include "scip/cons_knapsack.h"
#include "ConstraintHandler_VertexConflicts.h"
#include "ConstraintHandler_EdgeConflicts.h"
#include "Separator_Preprocessing.h"
#ifdef USE_RECTANGLE_KNAPSACK_CONFLICTS
#include "Separator_RectangleKnapsackConflicts.h"
#endif
#ifdef USE_RECTANGLE_CLIQUE_CONFLICTS
#include "Separator_RectangleCliqueConflicts.h"
#endif
#if defined(USE_CORRIDOR_CONFLICTS) || defined(USE_WAITCORRIDOR_CONFLICTS)
#include "Separator_CorridorConflicts.h"
#endif
#ifdef USE_STEPASIDE_CONFLICTS
#include "Separator_StepAsideConflicts.h"
#endif
#ifdef USE_WAITDELAY_CONFLICTS
#include "Separator_WaitDelayConflicts.h"
#endif
#ifdef USE_EXITENTRY_CONFLICTS
#include "Separator_ExitEntryConflicts.h"
#endif
#if defined(USE_TWOEDGE_CONFLICTS) || defined(USE_WAITTWOEDGE_CONFLICTS)
#include "Separator_TwoEdgeConflicts.h"
#endif
#ifdef USE_TWOVERTEX_CONFLICTS
#include "Separator_TwoVertexConflicts.h"
#endif
#ifdef USE_THREEVERTEX_CONFLICTS
#include "Separator_ThreeVertexConflicts.h"
#endif
#ifdef USE_FOUREDGE_CONFLICTS
#include "Separator_FourEdgeConflicts.h"
#endif
#ifdef USE_FIVEEDGE_CONFLICTS
#include "Separator_FiveEdgeConflicts.h"
#endif
#ifdef USE_SIXEDGE_CONFLICTS
#include "Separator_SixEdgeConflicts.h"
#endif
#ifdef USE_AGENTWAITEDGE_CONFLICTS
#include "Separator_AgentWaitEdgeConflicts.h"
#endif
#ifdef USE_VERTEX_FOUREDGE_CONFLICTS
#include "Separator_VertexFourEdgeConflicts.h"
#endif
#ifdef USE_CLIQUE_CONFLICTS
#include "Separator_CliqueConflicts.h"
#endif
#ifdef USE_GOAL_CONFLICTS
#include "Separator_GoalConflicts.h"
#endif
#include "BranchingRule.h"
#include "Constraint_VertexBranching.h"
#include "Constraint_WaitBranching.h"
#include "Constraint_LengthBranching.h"

// Problem data
struct SCIP_ProbData
{
    // Instance data
    SharedPtr<Instance> instance;                                        // Instance
    Agent N;                                                             // Number of agents

    // Model data
    SCIP_PricerData* pricerdata;                                         // Pricer data
    SharedPtr<AStar> astar;                                              // Pricing solver

    // Variables
    Vector<SCIP_VAR*> vars;                                              // Array of variables
    Vector<SCIP_VAR*> dummy_vars;                                        // Array of dummy variables
    Vector<Vector<SCIP_VAR*>> agent_vars;                                // Array of variables for each agent
    Vector<HashTable<NodeTime, SCIP_Real>> fractional_vertices;          // Fractional vertices
    Vector<HashTable<EdgeTime, SCIP_Real>> fractional_edges;             // Fractional edges
    Vector<HashTable<EdgeTime, SCIP_Real>> fractional_edges_no_waits;    // Fractional edges without waits

    // Constraints
    Vector<SCIP_CONS*> agent_part;                                       // Agent partition constraints
    SCIP_CONS* vertex_conflicts;                                         // Constraint for vertex conflicts
    SCIP_CONS* edge_conflicts;                                           // Constraint for edge conflicts
    Vector<TwoAgentRobustCut> two_agent_robust_cuts;                     // Robust cuts over two agents
#ifdef USE_RECTANGLE_KNAPSACK_CONFLICTS
    SCIP_SEPA* rectangle_knapsack_conflicts;                             // Separator for rectangle knapsack conflicts
#endif
#ifdef USE_RECTANGLE_CLIQUE_CONFLICTS
    SCIP_SEPA* rectangle_clique_conflicts;                               // Separator for rectangle clique conflicts
#endif
#ifdef USE_GOAL_CONFLICTS
    Vector<GoalConflict> goal_conflicts;                                 // Goal conflicts
#endif
};

// Create problem data for transformed problem
static
SCIP_DECL_PROBTRANS(probtrans)
{
    // Check.
    debug_assert(scip);
    debug_assert(sourcedata);
    debug_assert(targetdata);

    // Get problem data.
    const auto N = sourcedata->N;

    // Create transformed problem data object.
    SCIP_CALL(SCIPallocBlockMemory(scip, targetdata));
    debug_assert(*targetdata);
    new(*targetdata) SCIP_ProbData;

    // Copy instance data.
    (*targetdata)->instance = sourcedata->instance;
    (*targetdata)->N = sourcedata->N;

    // Copy model data.
    (*targetdata)->pricerdata = sourcedata->pricerdata;
    (*targetdata)->astar = sourcedata->astar;

    // Copy agent path variables.
    (*targetdata)->vars.resize(sourcedata->vars.size());
    (*targetdata)->vars.reserve(N * 5000);
    (*targetdata)->agent_vars.resize(N);
    for (Agent a = 0; a < N; ++a)
    {
        (*targetdata)->agent_vars[a].reserve(5000);
    }
    SCIP_CALL(SCIPtransformVars(scip,
                                sourcedata->vars.size(),
                                sourcedata->vars.data(),
                                (*targetdata)->vars.data()));
    for (auto var : (*targetdata)->vars)
    {
        debug_assert(var);
        auto vardata = SCIPvarGetData(var);
        const auto a = SCIPvardataGetAgent(vardata);

        (*targetdata)->agent_vars[a].push_back(var);
    }

    // Copy dummy variables.
    debug_assert(static_cast<Agent>(sourcedata->dummy_vars.size()) == N);
    (*targetdata)->dummy_vars.resize(N);
    SCIP_CALL(SCIPtransformVars(scip,
                                N,
                                sourcedata->dummy_vars.data(),
                                (*targetdata)->dummy_vars.data()));

    // Allocate memory for database of fractional vertices and edges.
    (*targetdata)->fractional_vertices.resize(N);
    (*targetdata)->fractional_edges.resize(N);
    (*targetdata)->fractional_edges_no_waits.resize(N);
//    for (Agent a = 0; a < N; ++a)
//    {
//        (*targetdata)->fractional_vertices[a].reserve(1);
//        (*targetdata)->fractional_edges[a].reserve(1);
//        (*targetdata)->fractional_edges_no_waits[a].reserve(1);
//    }

    // Copy agent partition constraints.
    debug_assert(static_cast<Agent>(sourcedata->agent_part.size()) == N);
    (*targetdata)->agent_part.resize(N);
    SCIP_CALL(SCIPtransformConss(scip,
                                 N,
                                 sourcedata->agent_part.data(),
                                 (*targetdata)->agent_part.data()));

    // Copy constraint for vertex conflicts.
    debug_assert(sourcedata->vertex_conflicts);
    (*targetdata)->vertex_conflicts = sourcedata->vertex_conflicts;
    SCIP_CALL(SCIPtransformCons(scip,
                                (*targetdata)->vertex_conflicts,
                                &(*targetdata)->vertex_conflicts));

    // Copy constraint for edge conflicts.
    debug_assert(sourcedata->edge_conflicts);
    (*targetdata)->edge_conflicts = sourcedata->edge_conflicts;
    SCIP_CALL(SCIPtransformCons(scip,
                                (*targetdata)->edge_conflicts,
                                &(*targetdata)->edge_conflicts));

    // Allocate memory for two-agent robust cuts.
    debug_assert(sourcedata->two_agent_robust_cuts.empty());
    (*targetdata)->two_agent_robust_cuts.reserve(5000);

    // Copy separator for rectangle knapsack conflicts.
#ifdef USE_RECTANGLE_KNAPSACK_CONFLICTS
    debug_assert(sourcedata->rectangle_knapsack_conflicts);
    (*targetdata)->rectangle_knapsack_conflicts = sourcedata->rectangle_knapsack_conflicts;
#endif

    // Copy separator for rectangle clique conflicts.
#ifdef USE_RECTANGLE_CLIQUE_CONFLICTS
    debug_assert(sourcedata->rectangle_clique_conflicts);
    (*targetdata)->rectangle_clique_conflicts = sourcedata->rectangle_clique_conflicts;
#endif
    
    // Create a warm-start solution.
    release_assert(SCIPgetProbData(scip) == *targetdata, "Error in transforming problem");
//    SCIP_CALL(add_initial_solution(scip));

    // Done.
    return SCIP_OKAY;
}

// Free problem data
static
SCIP_RETCODE probdataFree(
    SCIP* scip,                 // SCIP
    SCIP_ProbData** probdata    // Problem data
)
{
    // Check.
    debug_assert(scip);
    debug_assert(probdata);

    // Release variables.
    for (auto& var : (*probdata)->vars)
    {
        SCIP_CALL(SCIPreleaseVar(scip, &var));
    }

    // Release dummy variables.
    for (auto& var : (*probdata)->dummy_vars)
    {
        SCIP_CALL(SCIPreleaseVar(scip, &var));
    }

    // Release variables for agents.
    for (auto& vars : (*probdata)->agent_vars)
        for (auto& var : vars)
        {
            SCIP_CALL(SCIPreleaseVar(scip, &var));
        }

    // Release agent partition constraints.
    for (auto& cons : (*probdata)->agent_part)
    {
        SCIP_CALL(SCIPreleaseCons(scip, &cons));
    }

    // Release constraint for vertex conflicts.
    SCIP_CALL(SCIPreleaseCons(scip, &(*probdata)->vertex_conflicts));

    // Release constraint for edge conflicts.
    SCIP_CALL(SCIPreleaseCons(scip, &(*probdata)->edge_conflicts));

    // Release two-agent robust cuts.
    for (auto& cut : (*probdata)->two_agent_robust_cuts)
    {
        if (cut.is_same_time())
        {
            auto ptr = cut.edges_a1().first;
            const auto size = cut.edges_a2().second - cut.edges_a1().first;
            SCIPfreeBlockMemoryArray(scip, &ptr, size);
        }
        else
        {
            auto ptr = cut.edge_times_a1().first;
            const auto size = cut.edge_times_a2().second - cut.edge_times_a1().first;
            SCIPfreeBlockMemoryArray(scip, &ptr, size);
        }
    }

    // Destroy object.
    (*probdata)->~SCIP_ProbData();
    SCIPfreeBlockMemory(scip, probdata);

    // Done.
    return SCIP_OKAY;
}

// Free memory of original problem
static
SCIP_DECL_PROBDELORIG(probdelorig)
{
    // Destroy object.
    SCIP_CALL(probdataFree(scip, probdata));

    // Done.
    return SCIP_OKAY;
}

// Free problem data of transformed problem
static
SCIP_DECL_PROBDELTRANS(probdeltrans)
{
    // Destroy object.
    SCIP_CALL(probdataFree(scip, probdata));

    // Done.
    return SCIP_OKAY;
}

// Free when finishing branch-and-bound
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_PROBEXITSOL(probexitsol)
{
    // Free rows of two-agent robust cuts.
    for (auto& cut : probdata->two_agent_robust_cuts)
    {
        auto row = cut.row();
        SCIP_CALL(SCIPreleaseRow(scip, &row));
    }

    // Free rows of goal conflicts.
#ifdef USE_GOAL_CONFLICTS
    for (auto& goal_conflict : probdata->goal_conflicts)
    {
        auto row = goal_conflict.row;
        SCIP_CALL(SCIPreleaseRow(scip, &row));
    }
#endif

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Add a new dummy variable at the start
SCIP_RETCODE SCIPprobdataAddDummyVar(
    SCIP* scip,                 // SCIP
    SCIP_ProbData* probdata,    // Problem data
    const Agent a,              // Agent
    SCIP_VAR** var              // Output new variable
)
{
    // Calculate column cost.
    constexpr SCIP_Real obj = ARTIFICIAL_VAR_COST;

    // Create and add variable.
#ifdef MAKE_NAMES
    const auto name = fmt::format("dummy_path({})", a);
#endif
    SCIP_CALL(SCIPcreateVar(scip,
                            var,
#ifdef MAKE_NAMES
                            name.c_str(),
#else
                            "",
#endif
                            obj,
                            nullptr));
    debug_assert(*var);
    SCIP_CALL(SCIPaddVar(scip, *var));

    // Print.
    debugln("   Adding artificial variable with obj {:4.0f}, agent {:2d}",
            obj, a);

    // Change the upper bound of the binary variable to lazy since the upper bound is
    // already enforced due to the objective function the set covering constraint. The
    // reason for doing is that, is to avoid the bound of x <= 1 in the LP relaxation
    // since this bound constraint would produce a dual variable which might have a
    // positive reduced cost.
    SCIP_CALL(SCIPchgVarUbLazy(scip, *var, 1.0));

    // Add coefficient to agent partition constraint.
    debug_assert(a < static_cast<Agent>(probdata->agent_part.size()));
    SCIP_CALL(SCIPaddCoefSetppc(scip, probdata->agent_part[a], *var));

    // Store variable in dummy variables array.
    debug_assert(a < static_cast<Agent>(probdata->dummy_vars.size()));
    probdata->dummy_vars[a] = *var;

    // Done.
    return SCIP_OKAY;
}

// Add a new variable for an warm-start solution
SCIP_RETCODE SCIPprobdataAddInitialVar(
    SCIP* scip,                 // SCIP
    SCIP_ProbData* probdata,    // Problem data
    const Agent a,              // Agent
    const Time path_length,     // Path length
    const Edge* const path,     // Path
    SCIP_VAR** var              // Output new variable
)
{
    // Check.
    debug_assert(var);
    debug_assert(path);

    // Check that the path doesn't already exist.
#ifdef DEBUG
    for (auto var : probdata->agent_vars.at(a))
    {
        debug_assert(var);
        auto vardata = SCIPvarGetData(var);
        const auto existing_path_length = SCIPvardataGetPathLength(vardata);
        const auto existing_path = SCIPvardataGetPath(vardata);
        release_assert(!std::equal(path,
                                   path + path_length,
                                   existing_path,
                                   existing_path + existing_path_length),
                       "Path {} already exists for agent {} with value {}",
                       format_path(probdata, path_length, path),
                       a,
                       SCIPgetSolVal(scip, nullptr, var));
    }
#endif

    // Create variable data.
    SCIP_VarData* vardata = nullptr;
    SCIP_CALL(SCIPvardataCreate(scip, a, path_length, path, &vardata));
    debug_assert(vardata);

    // Calculate column cost.
    const SCIP_Real obj = path_length - 1;

    // Create and add variable.
#ifdef MAKE_NAMES
    const auto name = fmt::format("path({},({}))",
                                  a, format_path(probdata, path_length, path)).substr(0, 255);
#endif
    SCIP_CALL(SCIPcreateVar(scip,
                            var,
#ifdef MAKE_NAMES
        name.c_str(),
#else
                            "",
#endif
                            obj,
                            vardata));
    debug_assert(*var);
    SCIP_CALL(SCIPaddVar(scip, *var));

    // Print.
    debugln("      Adding column with obj {:4.0f}, agent {:2d}, path {}",
            obj, a, format_path_spaced(probdata, path_length, path));

    // Change the upper bound of the binary variable to lazy since the upper bound is
    // already enforced due to the objective function the set covering constraint. The
    // reason for doing is that, is to avoid the bound of x <= 1 in the LP relaxation
    // since this bound constraint would produce a dual variable which might have a
    // positive reduced cost.
    SCIP_CALL(SCIPchgVarUbLazy(scip, *var, 1.0));

    // Add coefficient to agent partition constraint.
//    debug_assert(SCIPconsIsEnabled(probdata->agent_part[a]));
    SCIP_CALL(SCIPaddCoefSetppc(scip, probdata->agent_part[a], *var));

    // Add coefficient to vertex conflicts constraints.
    SCIP_CALL(vertex_conflicts_add_var(scip,
                                       probdata->vertex_conflicts,
                                       *var,
                                       path_length,
                                       path));

    // Add coefficient to edge conflicts constraints.
    SCIP_CALL(edge_conflicts_add_var(scip,
                                     probdata->edge_conflicts,
                                     *var,
                                     path_length,
                                     path));

    // Add coefficients to two-agent robust cuts.
    for (const auto& cut : probdata->two_agent_robust_cuts)
    {
        // Calculate the coefficient.
        SCIP_Real coeff = 0.0;
        if (a == cut.a1() || a == cut.a2())
        {
            if (cut.is_same_time())
            {
                const auto t = cut.t();
                for (auto [it, end] = cut.edges(a); it != end; ++it)
                {
                    const auto e = *it;
                    coeff += (t < path_length && path[t] == e);
                }
            }
            else
            {
                for (auto [it, end] = cut.edge_times(a); it != end; ++it)
                {
                    const auto [e, t] = it->et;
                    coeff += (t < path_length && path[t] == e);
                }
            }
        }

        // Add variable to the cut.
        if (coeff)
        {
            SCIP_CALL(SCIPaddVarToRow(scip, cut.row(), *var, coeff));
        }
    }

    // Add coefficient to goal conflicts constraints.
#ifdef USE_GOAL_CONFLICTS
    SCIP_CALL(goal_conflicts_add_var(scip,
                                     probdata->goal_conflicts,
                                     *var,
                                     a,
                                     path_length,
                                     path));
#endif

    // Store variable in array of all variables.
    probdata->vars.push_back(*var);

    // Store variable in agent variables array.
    debug_assert(a < static_cast<Agent>(probdata->agent_vars.size()));
    probdata->agent_vars[a].push_back(*var);

    // Capture variable again. Previously captured in addVar.
    SCIP_CALL(SCIPcaptureVar(scip, *var));

    // Done.
    return SCIP_OKAY;
}

// Add a new variable from pricing
SCIP_RETCODE SCIPprobdataAddPricedVar(
    SCIP* scip,                 // SCIP
    SCIP_ProbData* probdata,    // Problem data
    const Agent a,              // Agent
    const Time path_length,     // Path length
    const Edge* const path,     // Path
    SCIP_VAR** var              // Output new variable
)
{
    // Check.
    debug_assert(var);
    debug_assert(path);

    // Check that the path doesn't already exist.
#ifdef DEBUG
    for (auto var : probdata->agent_vars.at(a))
    {
        debug_assert(var);
        auto vardata = SCIPvarGetData(var);
        const auto existing_path_length = SCIPvardataGetPathLength(vardata);
        const auto existing_path = SCIPvardataGetPath(vardata);
        release_assert(!std::equal(path,
                                   path + path_length,
                                   existing_path,
                                   existing_path + existing_path_length),
                       "Path {} already exists for agent {} with value {}",
                       format_path(probdata, path_length, path),
                       a,
                       SCIPgetSolVal(scip, nullptr, var));
    }
#endif

    // Create variable data.
    SCIP_VarData* vardata = nullptr;
    SCIP_CALL(SCIPvardataCreate(scip, a, path_length, path, &vardata));
    debug_assert(vardata);

    // Calculate column cost.
    const SCIP_Real obj = path_length - 1;

    // Create and add variable.
#ifdef MAKE_NAMES
    const auto name = fmt::format("path({},({}))",
                                  a, format_path(probdata, path_length, path)).substr(0, 255);
#endif
    SCIP_CALL(SCIPcreateVar(scip,
                            var,
#ifdef MAKE_NAMES
                            name.c_str(),
#else
                            "",
#endif
                            obj,
                            vardata));
    debug_assert(*var);
    SCIP_CALL(SCIPaddPricedVar(scip, *var, 1.0));

    // Print.
    debugln("      Adding column with obj {:4.0f}, agent {:2d}, path {}",
            obj, a, format_path_spaced(probdata, path_length, path));

    // Change the upper bound of the binary variable to lazy since the upper bound is
    // already enforced due to the objective function the set covering constraint. The
    // reason for doing is that, is to avoid the bound of x <= 1 in the LP relaxation
    // since this bound constraint would produce a dual variable which might have a
    // positive reduced cost.
    SCIP_CALL(SCIPchgVarUbLazy(scip, *var, 1.0));

    // Add coefficient to agent partition constraint.
    debug_assert(SCIPconsIsEnabled(probdata->agent_part[a]));
    SCIP_CALL(SCIPaddCoefSetppc(scip, probdata->agent_part[a], *var));

    // Add coefficient to vertex conflicts constraints.
    SCIP_CALL(vertex_conflicts_add_var(scip,
                                       probdata->vertex_conflicts,
                                       *var,
                                       path_length,
                                       path));

    // Add coefficient to edge conflicts constraints.
    SCIP_CALL(edge_conflicts_add_var(scip,
                                     probdata->edge_conflicts,
                                     *var,
                                     path_length,
                                     path));

    // Add coefficients to two-agent robust cuts.
    for (const auto& cut : probdata->two_agent_robust_cuts)
    {
        // Calculate the coefficient.
        SCIP_Real coeff = 0.0;
        if (a == cut.a1() || a == cut.a2())
        {
            if (cut.is_same_time())
            {
                const auto t = cut.t();
                for (auto [it, end] = cut.edges(a); it != end; ++it)
                {
                    const auto e = *it;
                    coeff += (t < path_length - 1 && e == path[t]) ||
                             (t >= path_length - 1 && e.n == path[path_length - 1].n && e.d == Direction::WAIT);
                }
            }
            else
            {
                for (auto [it, end] = cut.edge_times(a); it != end; ++it)
                {
                    const auto [e, t] = it->et;
                    coeff += (t < path_length - 1 && e == path[t]) ||
                             (t >= path_length - 1 && e.n == path[path_length - 1].n && e.d == Direction::WAIT);
                }
            }
        }

        // Add variable to the cut.
        if (coeff)
        {
            SCIP_CALL(SCIPaddVarToRow(scip, cut.row(), *var, coeff));
        }
    }

    // Add coefficient to rectangle clique conflicts constraints.
#ifdef USE_RECTANGLE_CLIQUE_CONFLICTS
    SCIP_CALL(rectangle_clique_conflicts_add_var(scip,
                                                 probdata->rectangle_clique_conflicts,
                                                 *var,
                                                 a,
                                                 path_length,
                                                 path));
#endif

    // Add coefficient to goal conflicts constraints.
#ifdef USE_GOAL_CONFLICTS
    SCIP_CALL(goal_conflicts_add_var(scip,
                                     probdata->goal_conflicts,
                                     *var,
                                     a,
                                     path_length,
                                     path));
#endif

    // Store variable in array of all variables.
    probdata->vars.push_back(*var);

    // Store variable in agent variables array.
    debug_assert(a < static_cast<Agent>(probdata->agent_vars.size()));
    probdata->agent_vars[a].push_back(*var);

    // Capture variable again. Previously captured in addVar.
    SCIP_CALL(SCIPcaptureVar(scip, *var));

    // Done.
    return SCIP_OKAY;
}

// Add a new two-agent robust cut
SCIP_RETCODE SCIPprobdataAddTwoAgentRobustCut(
    SCIP* scip,                 // SCIP
    SCIP_ProbData* probdata,    // Problem data
    SCIP_SEPA* sepa,            // Separator
    TwoAgentRobustCut&& cut,    // Data for the cut
    const SCIP_Real rhs,        // RHS
    SCIP_RESULT* result,        // Output result
    Int* idx                    // Output index of the cut
)
{
    // Create a row.
    SCIP_ROW* row = nullptr;
    SCIP_CALL(SCIPcreateEmptyRowSepa(scip,
                                     &row,
                                     sepa,
#ifdef MAKE_NAMES
                                     cut.name().c_str(),
#else
                                     "",
#endif
                                     -SCIPinfinity(scip),
                                     rhs,
                                     FALSE,
                                     TRUE,
                                     FALSE));
    debug_assert(row);
    cut.set_row(row);

    // Get variables.
    const auto& agent_vars = SCIPprobdataGetAgentVars(probdata);

    // Add variables to the constraint.
#ifdef DEBUG
    SCIP_Real lhs = 0.0;
#endif
    SCIP_CALL(SCIPcacheRowExtensions(scip, row));
    for (const auto a : Array<Agent, 2>{cut.a1(), cut.a2()})
        for (auto var : agent_vars[a])
        {
            // Get the path.
            debug_assert(var);
            auto vardata = SCIPvarGetData(var);
            debug_assert(a == SCIPvardataGetAgent(vardata));
            const auto path_length = SCIPvardataGetPathLength(vardata);
            const auto path = SCIPvardataGetPath(vardata);

            // Add coefficients.
            SCIP_Real coeff = 0.0;
            if (cut.is_same_time())
            {
                const auto t = cut.t();
                for (auto [it, end] = cut.edges(a); it != end; ++it)
                {
                    const auto e = *it;
                    coeff += (t < path_length - 1 && e == path[t]) ||
                             (t >= path_length - 1 && e.n == path[path_length - 1].n && e.d == Direction::WAIT);
                }
            }
            else
            {
                for (auto [it, end] = cut.edge_times(a); it != end; ++it)
                {
                    const auto [e, t] = it->et;
                    coeff += (t < path_length - 1 && e == path[t]) ||
                             (t >= path_length - 1 && e.n == path[path_length - 1].n && e.d == Direction::WAIT);
                }
            }

            // Add coefficient.
            if (coeff)
            {
                // Add coefficient.
                SCIP_CALL(SCIPaddVarToRow(scip, row, var, coeff));
#ifdef DEBUG
                lhs += SCIPgetSolVal(scip, nullptr, var) * coeff;
#endif

                // Print.
                debugln("      Val: {:7.4f}:, Agent: {:3d}, Coeff: {:1.0f}, Path: {}",
                        SCIPgetSolVal(scip, nullptr, var),
                        a,
                        coeff,
                        format_path_spaced(SCIPgetProbData(scip), path_length, path));
            }
        }
    SCIP_CALL(SCIPflushRowExtensions(scip, row));
#ifdef DEBUG
    debug_assert(SCIPisSumGT(scip, lhs, rhs));
#endif

    // Check agent and edges.
#ifdef DEBUG
    release_assert(cut.a1() != cut.a2(), "Same agents in cut");
    if (cut.is_same_time())
    {
        for (auto [it1, end] = cut.edges_a1(); it1 != end; ++it1)
            for (auto it2 = it1 + 1; it2 != end; ++it2)
            {
                release_assert(*it1 != *it2, "Agent {} has a duplicate edge in cut", cut.a1());
            }
        for (auto [it1, end] = cut.edges_a2(); it1 != end; ++it1)
            for (auto it2 = it1 + 1; it2 != end; ++it2)
            {
                release_assert(*it1 != *it2, "Agent {} has a duplicate edge in cut", cut.a2());
            }
    }
    else
    {
        for (auto [it1, end] = cut.edge_times_a1(); it1 != end; ++it1)
            for (auto it2 = it1 + 1; it2 != end; ++it2)
            {
                release_assert(*it1 != *it2, "Agent {} has a duplicate edge in cut", cut.a1());
            }
        for (auto [it1, end] = cut.edge_times_a2(); it1 != end; ++it1)
            for (auto it2 = it1 + 1; it2 != end; ++it2)
            {
                release_assert(*it1 != *it2, "Agent {} has a duplicate edge in cut", cut.a2());
            }
    }
#endif

    // If the cut is a sum of edges <= 1, check that all edges are mutually incompatible.
#ifdef DEBUG
    if (SCIPisEQ(scip, rhs, 1.0))
    {
        const auto& map = SCIPprobdataGetMap(probdata);
        if (cut.is_same_time())
        {
            for (auto [it1, end1] = cut.edges_a1(); it1 != end1; ++it1)
                for (auto [it2, end2] = cut.edges_a2(); it2 != end2; ++it2)
                {
                    const auto a1_n1 = it1->n;
                    const auto a1_n2 = it1->d == Direction::NORTH ? map.get_north(a1_n1) :
                                       it1->d == Direction::SOUTH ? map.get_south(a1_n1) :
                                       it1->d == Direction::EAST ? map.get_east(a1_n1) :
                                       it1->d == Direction::WEST ? map.get_west(a1_n1) : a1_n1;
                    const auto a2_n1 = it2->n;
                    const auto a2_n2 = it2->d == Direction::NORTH ? map.get_north(a2_n1) :
                                       it2->d == Direction::SOUTH ? map.get_south(a2_n1) :
                                       it2->d == Direction::EAST ? map.get_east(a2_n1) :
                                       it2->d == Direction::WEST ? map.get_west(a2_n1) : a2_n1;

                    if (a1_n1 == a2_n1 ||                   // Vertex conflict at time t
                        a1_n2 == a2_n2 ||                   // Vertex conflict at time t+1
                        (a1_n1 == a2_n2 && a1_n2 == a2_n1)) // Edge conflict
                    {
                        continue;
                    }
                    err("Edges in robust cut are not mutually incompatible");
                }
        }
        else
        {
            for (auto [it1, end] = cut.edge_times_a1(); it1 != end; ++it1)
                for (auto it2 = it1 + 1; it2 != end; ++it2)
                {
                    const auto t1 = it1->t;
                    const auto n11 = it1->n;
                    const auto n12 = it1->d == Direction::NORTH ? map.get_north(n11) :
                                       it1->d == Direction::SOUTH ? map.get_south(n11) :
                                       it1->d == Direction::EAST ? map.get_east(n11) :
                                       it1->d == Direction::WEST ? map.get_west(n11) : n11;
                    const auto t2 = it2->t;
                    const auto n21 = it2->n;
                    const auto n22 = it2->d == Direction::NORTH ? map.get_north(n21) :
                                     it2->d == Direction::SOUTH ? map.get_south(n21) :
                                     it2->d == Direction::EAST ? map.get_east(n21) :
                                     it2->d == Direction::WEST ? map.get_west(n21) : n21;

                    if ((n12 != n21 && t1 == t2 - 1) || (n22 != n11 && t2 == t1 - 1) || (t1 == t2))
                    {
                        continue;
                    }
                    err("Edges in robust cut are not mutually incompatible");
                }
            for (auto [it1, end] = cut.edge_times_a2(); it1 != end; ++it1)
                for (auto it2 = it1 + 1; it2 != end; ++it2)
                {
                    const auto t1 = it1->t;
                    const auto n11 = it1->n;
                    const auto n12 = it1->d == Direction::NORTH ? map.get_north(n11) :
                                     it1->d == Direction::SOUTH ? map.get_south(n11) :
                                     it1->d == Direction::EAST ? map.get_east(n11) :
                                     it1->d == Direction::WEST ? map.get_west(n11) : n11;
                    const auto t2 = it2->t;
                    const auto n21 = it2->n;
                    const auto n22 = it2->d == Direction::NORTH ? map.get_north(n21) :
                                     it2->d == Direction::SOUTH ? map.get_south(n21) :
                                     it2->d == Direction::EAST ? map.get_east(n21) :
                                     it2->d == Direction::WEST ? map.get_west(n21) : n21;

                    if ((n12 != n21 && t1 == t2 - 1) || (n22 != n11 && t2 == t1 - 1) || (t1 == t2))
                    {
                        continue;
                    }
                    err("Edges in robust cut are not mutually incompatible");
                }

            for (auto [it1, end1] = cut.edge_times_a1(); it1 != end1; ++it1)
                for (auto [it2, end2] = cut.edge_times_a2(); it2 != end2; ++it2)
                {
                    const auto a1_t = it1->t;
                    const auto a1_n1 = it1->n;
                    const auto a1_n2 = it1->d == Direction::NORTH ? map.get_north(a1_n1) :
                                       it1->d == Direction::SOUTH ? map.get_south(a1_n1) :
                                       it1->d == Direction::EAST ? map.get_east(a1_n1) :
                                       it1->d == Direction::WEST ? map.get_west(a1_n1) : a1_n1;
                    const auto a2_t = it2->t;
                    const auto a2_n1 = it2->n;
                    const auto a2_n2 = it2->d == Direction::NORTH ? map.get_north(a2_n1) :
                                       it2->d == Direction::SOUTH ? map.get_south(a2_n1) :
                                       it2->d == Direction::EAST ? map.get_east(a2_n1) :
                                       it2->d == Direction::WEST ? map.get_west(a2_n1) : a2_n1;

                    if ((a1_n1 == a2_n1 && a1_t == a2_t) ||                   // Vertex conflict at time t
                        (a1_n2 == a2_n2 && a1_t == a2_t) ||                   // Vertex conflict at time t+1
                        (a1_n1 == a2_n2 && a1_n2 == a2_n1 && a1_t == a2_t) || // Edge conflict
                        (a1_n2 == a2_n1 && a1_t == a2_t - 1) ||               // Vertex conflict
                        (a2_n2 == a1_n1 && a2_t == a1_t - 1))                 // Vertex conflict
                    {
                        continue;
                    }
                    err("Edges in robust cut are not mutually incompatible");
                }
        }
    }
#endif

    // Add the row to the LP.
    SCIP_Bool infeasible;
    SCIP_CALL(SCIPaddRow(scip, row, TRUE, &infeasible));

    // Set status.
    *result = infeasible ? SCIP_CUTOFF : SCIP_SEPARATED;

    // Store the cut.
    if (idx)
    {
        *idx = probdata->two_agent_robust_cuts.size();
    }
    probdata->two_agent_robust_cuts.push_back(std::move(cut));

    // Done.
    return SCIP_OKAY;
}

// Create the problem
SCIP_RETCODE SCIPprobdataCreate(
    SCIP* scip,                       // SCIP
    const char* probname,             // Problem name
    SharedPtr<Instance>& instance,    // Instance
    SharedPtr<AStar>& astar           // Search algorithm
)
{
    // Check.
    debug_assert(scip);

    // Create problem.
    SCIP_CALL(SCIPcreateProbBasic(scip, probname));

    // Set optimization direction.
    SCIP_CALL(SCIPsetObjsense(scip, SCIP_OBJSENSE_MINIMIZE));

    // Tell SCIP that the objective value will be always integral.
    SCIP_CALL(SCIPsetObjIntegral(scip));

    // Get problem data.
    const auto N = instance->agents.size();
    release_assert(N > 0, "Zero agents in the instance");

    // Tell SCIP the B&B node is infeasible if it has objective value greater than the
    // cost of a dummy path.
    SCIP_CALL(SCIPsetObjlimit(scip, ARTIFICIAL_VAR_COST - 1));

    // Check start and end.
    {
        Position* start_x;
        Position* start_y;
        Position* end_x;
        Position* end_y;
        SCIP_CALL(SCIPallocBufferArray(scip, &start_x, N));
        SCIP_CALL(SCIPallocBufferArray(scip, &start_y, N));
        SCIP_CALL(SCIPallocBufferArray(scip, &end_x, N));
        SCIP_CALL(SCIPallocBufferArray(scip, &end_y, N));
        for (Agent a = 0; a < N; ++a)
        {
            const auto& agent_data = instance->agents[a];
            start_x[a] = agent_data.start_x;
            start_y[a] = agent_data.start_y;
            end_x[a] = agent_data.goal_x;
            end_y[a] = agent_data.goal_y;
        }
        for (Agent a = 0; a < N - 1; ++a)
            for (Agent b = a + 1; b < N; ++b)
            {
                release_assert(start_x[a] != start_x[b] || start_y[a] != start_y[b],
                               "Agent {} and {} both start at ({},{})",
                               a, b, start_x[a], start_y[a]);
                release_assert(end_x[a] != end_x[b] || end_y[a] != end_y[b],
                               "Agent {} and {} both end at ({},{})",
                               a, b, end_x[a], end_y[a]);
            }
        SCIPfreeBufferArray(scip, &start_x);
        SCIPfreeBufferArray(scip, &start_y);
        SCIPfreeBufferArray(scip, &end_x);
        SCIPfreeBufferArray(scip, &end_y);
    }

    // Create problem data object.
    SCIP_ProbData* probdata = nullptr;
    SCIP_CALL(SCIPallocBlockMemory(scip, &probdata));
    debug_assert(probdata);
    new(probdata) SCIP_ProbData;

    // Copy instance data.
    probdata->instance = instance;
    probdata->N = N;

    // Copy model data.
    probdata->pricerdata = nullptr;
    probdata->astar = astar;

    // Create agent partition constraints.
    probdata->agent_part.resize(N);
    for (Agent a = 0; a < N; ++a)
    {
        // Create constraint name.
        char name[SCIP_MAXSTRLEN];
        SCIPsnprintf(name, SCIP_MAXSTRLEN, "agent_part(%d)", a);

        // Create constraint.
        SCIP_CALL(SCIPcreateConsSetcover(scip,
                                         &(probdata->agent_part[a]),
                                         name,
                                         0,
                                         nullptr,
                                         TRUE,
                                         TRUE,
                                         TRUE,
                                         TRUE,
                                         TRUE,
                                         FALSE,
                                         TRUE,
                                         FALSE,
                                         FALSE,
                                         FALSE));
        debug_assert(&(probdata->agent_part[a]));

        // Add the constraint to the problem.
        SCIP_CALL(SCIPaddCons(scip, probdata->agent_part[a]));
    }

    // Create the constraint handler for vertex conflicts.
    SCIP_CALL(SCIPincludeConshdlrVertexConflicts(scip));
    SCIP_CALL(SCIPcreateConsVertexConflicts(scip,
                                            &probdata->vertex_conflicts,
                                            "vertex_conflicts",
                                            TRUE,
                                            TRUE,
                                            TRUE,
                                            TRUE,
                                            TRUE,
                                            FALSE,
                                            TRUE,
                                            FALSE,
                                            FALSE,
                                            FALSE));
    SCIP_CALL(SCIPaddCons(scip, probdata->vertex_conflicts));

    // Create the constraint handler for edge conflicts.
    SCIP_CALL(SCIPincludeConshdlrEdgeConflicts(scip));
    SCIP_CALL(SCIPcreateConsEdgeConflicts(scip,
                                          &probdata->edge_conflicts,
                                          "edge_conflicts",
                                          TRUE,
                                          TRUE,
                                          TRUE,
                                          TRUE,
                                          TRUE,
                                          FALSE,
                                          TRUE,
                                          FALSE,
                                          FALSE,
                                          FALSE));
    SCIP_CALL(SCIPaddCons(scip, probdata->edge_conflicts));

    // Include separator for preprocessing dummy constraint.
    SCIP_CALL(SCIPincludeSepaPreprocessing(scip));

    // Include separator for rectangle knapsack conflicts.
#ifdef USE_RECTANGLE_KNAPSACK_CONFLICTS
    SCIP_CALL(SCIPincludeSepaRectangleKnapsackConflicts(scip, &probdata->rectangle_knapsack_conflicts));
#endif

    // Include separator for rectangle clique conflicts.
#ifdef USE_RECTANGLE_CLIQUE_CONFLICTS
    SCIP_CALL(SCIPincludeSepaRectangleCliqueConflicts(scip, &probdata->rectangle_clique_conflicts));
#endif

    // Include separator for corridor conflicts.
#if defined(USE_CORRIDOR_CONFLICTS) || defined(USE_WAITCORRIDOR_CONFLICTS)
    SCIP_CALL(SCIPincludeSepaCorridorConflicts(scip));
#endif

    // Include separator for step-aside conflicts.
#ifdef USE_STEPASIDE_CONFLICTS
    SCIP_CALL(SCIPincludeSepaStepAsideConflicts(scip));
#endif

    // Include separator for wait-delay conflicts.
#ifdef USE_WAITDELAY_CONFLICTS
    SCIP_CALL(SCIPincludeSepaWaitDelayConflicts(scip));
#endif

    // Include separator for exit-entry conflicts.
#ifdef USE_EXITENTRY_CONFLICTS
    SCIP_CALL(SCIPincludeSepaExitEntryConflicts(scip));
#endif

    // Include separator for two-edge conflicts.
#if defined(USE_TWOEDGE_CONFLICTS) || defined(USE_WAITTWOEDGE_CONFLICTS)
    SCIP_CALL(SCIPincludeSepaTwoEdgeConflicts(scip));
#endif

    // Include separator for two-vertex conflicts.
#ifdef USE_TWOVERTEX_CONFLICTS
    SCIP_CALL(SCIPincludeSepaTwoVertexConflicts(scip));
#endif

    // Include separator for three-vertex conflicts.
#ifdef USE_THREEVERTEX_CONFLICTS
    SCIP_CALL(SCIPincludeSepaThreeVertexConflicts(scip));
#endif

    // Include separator for four-edge conflicts.
#ifdef USE_FOUREDGE_CONFLICTS
    SCIP_CALL(SCIPincludeSepaFourEdgeConflicts(scip));
#endif

    // Include separator for five-edge conflicts.
#ifdef USE_FIVEEDGE_CONFLICTS
    SCIP_CALL(SCIPincludeSepaFiveEdgeConflicts(scip));
#endif

    // Include separator for six-edge conflicts.
#ifdef USE_SIXEDGE_CONFLICTS
    SCIP_CALL(SCIPincludeSepaSixEdgeConflicts(scip));
#endif

    // Include separator for agent wait-edge conflicts.
#ifdef USE_AGENTWAITEDGE_CONFLICTS
    SCIP_CALL(SCIPincludeSepaAgentWaitEdgeConflicts(scip));
#endif

    // Include separator for vertex four-edge conflicts.
#ifdef USE_VERTEX_FOUREDGE_CONFLICTS
    SCIP_CALL(SCIPincludeSepaVertexFourEdgeConflicts(scip));
#endif

    // Include separator for clique conflicts.
#ifdef USE_CLIQUE_CONFLICTS
    SCIP_CALL(SCIPincludeSepaCliqueConflicts(scip));
#endif

    // Include separator for goal conflicts.
#ifdef USE_GOAL_CONFLICTS
    SCIP_CALL(SCIPincludeSepaGoalConflicts(scip));
#endif

    // Create dummy paths.
    probdata->dummy_vars.resize(N);
    for (Agent a = 0; a < N; ++a)
    {
        SCIP_VAR* var;
        SCIP_CALL(SCIPprobdataAddDummyVar(scip, probdata, a, &var));
        debug_assert(var);
    }

    // Calculate the longest path length. Do this internally after computing h.
    {
        const auto& agents = instance->agents;
        for (Agent a = 0; a < N; ++a)
        {
            const auto goal = agents[a].goal;
            probdata->astar->compute_h(goal);
        }
    }

    // Set problem data.
    SCIP_CALL(SCIPsetProbData(scip, probdata));

    // Include Trufflehog pricer.
    SCIP_CALL(SCIPincludePricerTruffleHog(scip));
    SCIP_CALL(SCIPpricerTruffleHogActivate(scip));

    // Include branching rule.
    SCIP_CALL(SCIPincludeBranchrule(scip));
    SCIP_CALL(SCIPincludeConshdlrVertexBranching(scip));
//    SCIP_CALL(SCIPincludeConshdlrWaitBranching(scip));
    SCIP_CALL(SCIPincludeConshdlrLengthBranching(scip));

    // Add callbacks.
    SCIP_CALL(SCIPsetProbTrans(scip, probtrans));
    SCIP_CALL(SCIPsetProbDelorig(scip, probdelorig));
    SCIP_CALL(SCIPsetProbDeltrans(scip, probdeltrans));
    SCIP_CALL(SCIPsetProbExitsol(scip, probexitsol));

    // Done.
    return SCIP_OKAY;
}

// Get array of variables
Vector<SCIP_VAR*>& SCIPprobdataGetVars(
    SCIP_ProbData* probdata    // Problem data
)
{
    debug_assert(probdata);
    return probdata->vars;
}

// Get array of dummy variables
Vector<SCIP_VAR*>& SCIPprobdataGetDummyVars(
    SCIP_ProbData* probdata    // Problem data
)
{
    debug_assert(probdata);
    return probdata->dummy_vars;
}

// Get array of variables for an agent
Vector<Vector<SCIP_VAR*>>& SCIPprobdataGetAgentVars(
    SCIP_ProbData* probdata    // Problem data
)
{
    debug_assert(probdata);
    return probdata->agent_vars;
}

// Get agent partition constraints
Vector<SCIP_CONS*>& SCIPprobdataGetAgentPartConss(
    SCIP_ProbData* probdata    // Problem data
)
{
    debug_assert(probdata);
    return probdata->agent_part;
}

// Get constraint for vertex conflicts
SCIP_CONS* SCIPprobdataGetVertexConflictsCons(
    SCIP_ProbData* probdata    // Problem data
)
{
    debug_assert(probdata);
    debug_assert(probdata->vertex_conflicts);
    return probdata->vertex_conflicts;
}

// Get constraint for edge conflicts
SCIP_CONS* SCIPprobdataGetEdgeConflictsCons(
    SCIP_ProbData* probdata    // Problem data
)
{
    debug_assert(probdata);
    debug_assert(probdata->edge_conflicts);
    return probdata->edge_conflicts;
}

// Get array of two-agent robust cuts
Vector<TwoAgentRobustCut>& SCIPprobdataGetTwoAgentRobustCuts(
    SCIP_ProbData* probdata    // Problem data
)
{
    debug_assert(probdata);
    return probdata->two_agent_robust_cuts;
}

// Get separator for rectangle knapsack conflicts
#ifdef USE_RECTANGLE_KNAPSACK_CONFLICTS
SCIP_SEPA* SCIPprobdataGetRectangleKnapsackConflictsSepa(
    SCIP_ProbData* probdata    // Problem data
)
{
    debug_assert(probdata);
    debug_assert(probdata->rectangle_knapsack_conflicts);
    return probdata->rectangle_knapsack_conflicts;
}
#endif

// Get separator for rectangle clique conflicts
#ifdef USE_RECTANGLE_CLIQUE_CONFLICTS
SCIP_SEPA* SCIPprobdataGetRectangleCliqueConflictsSepa(
    SCIP_ProbData* probdata    // Problem data
)
{
    debug_assert(probdata);
    debug_assert(probdata->rectangle_clique_conflicts);
    return probdata->rectangle_clique_conflicts;
}
#endif

// Get goal conflicts
#ifdef USE_GOAL_CONFLICTS
Vector<GoalConflict>& SCIPprobdataGetGoalConflicts(
    SCIP_ProbData* probdata    // Problem data
)
{
    debug_assert(probdata);
    return probdata->goal_conflicts;
}
#endif

// Get the vertices fractionally used by each agent
const Vector<HashTable<NodeTime, SCIP_Real>>& SCIPprobdataGetAgentFractionalVertices(
    SCIP_ProbData* probdata    // Problem data
)
{
    debug_assert(probdata);
    return probdata->fractional_vertices;
}

// Get the edges fractionally used by each agent
const Vector<HashTable<EdgeTime, SCIP_Real>>& SCIPprobdataGetAgentFractionalEdges(
    SCIP_ProbData* probdata    // Problem data
)
{
    debug_assert(probdata);
    return probdata->fractional_edges;
}

// Get the non-wait edges fractionally used by each agent
const Vector<HashTable<EdgeTime, SCIP_Real>>& SCIPprobdataGetAgentFractionalEdgesNoWaits(
    SCIP_ProbData* probdata    // Problem data
)
{
    debug_assert(probdata);
    return probdata->fractional_edges_no_waits;
}

// Update the database of fractionally used vertices and edges
void update_fractional_vertices_and_edges(
    SCIP* scip    // SCIP
)
{
    // Print.
    debugln("Updating fractional vertices and edges");

    // Get problem data.
    auto probdata = SCIPgetProbData(scip);
    const auto N = SCIPprobdataGetN(probdata);

    // Get variables.
    const auto& vars = SCIPprobdataGetVars(probdata);
    const auto& agent_vars = SCIPprobdataGetAgentVars(probdata);

    // Find the makespan.
    Time makespan = 0;
    for (auto var : vars)
    {
        // Get the path length.
        debug_assert(var);
        auto vardata = SCIPvarGetData(var);
        const auto path_length = SCIPvardataGetPathLength(vardata);

        // Get the variable value.
        const auto var_val = SCIPgetSolVal(scip, nullptr, var);

        // Store the length of the longest path.
        if (path_length > makespan && SCIPisPositive(scip, var_val))
        {
            makespan = path_length;
        }
    }

    // Get vertices of each agent.
    auto& fractional_vertices = probdata->fractional_vertices;
    auto& fractional_edges = probdata->fractional_edges;
    auto& fractional_edges_no_waits = probdata->fractional_edges_no_waits;
    for (Agent a = 0; a < N; ++a)
    {
        // Clear data from previous iteration.
        auto& agent_vertices = fractional_vertices[a];
        agent_vertices.clear();
        // agent_vertices.reserve(sqrt(map.width() * map.height()) * 15);

        // Clear data from previous iteration.
        auto& agent_edges = fractional_edges[a];
        agent_edges.clear();
        // agent_edges.reserve(sqrt(map.width() * map.height()) * 20);

        // Clear data from previous iteration.
        auto& agent_edges_no_waits = fractional_edges_no_waits[a];
        agent_edges_no_waits.clear();
        // agent_edges_no_waits.reserve(sqrt(map.width() * map.height()) * 20);

        // Calculate the number of times a vertex is used by summing the columns.
        for (auto var : agent_vars[a])
        {
            // Get the path.
            debug_assert(var);
            auto vardata = SCIPvarGetData(var);
            const auto path_length = SCIPvardataGetPathLength(vardata);
            const auto path = SCIPvardataGetPath(vardata);

            // Get the variable value.
            const auto var_val = SCIPgetSolVal(scip, nullptr, var);

            // Store the vertices and edges of the path.
            if (SCIPisPositive(scip, var_val) && !SCIPisIntegral(scip, var_val))
            {
                // Store everything except the last vertex.
                Time t = 0;
                for (; t < path_length - 1; ++t)
                {
                    // Store the vertex.
                    {
                        const NodeTime nt{path[t].n, t};
                        agent_vertices[nt] += var_val;
                    }

                    // Store the edge.
                    {
                        const EdgeTime et{path[t], t};
                        agent_edges[et] += var_val;
                    }

                    // Store the edge if not a wait edge.
                    if (path[t].d != Direction::WAIT)
                    {
                        const EdgeTime et{path[t], t};
                        agent_edges_no_waits[et] += var_val;
                    }
                }

                // Store the edges after the agent reaches its goal.
                const auto n = path[t].n;
                for (; t < makespan - 1; ++t)
                {
                    // Store the vertex.
                    {
                        const NodeTime nt{n, t};
                        agent_vertices[nt] += var_val;
                    }

                    // Store the edge.
                    {
                        const EdgeTime et{n, Direction::WAIT, t};
                        agent_edges[et] += var_val;
                    }
                }

                // Store the last vertex.
                {
                    const NodeTime nt{n, t};
                    agent_vertices[nt] += var_val;
                }
            }
        }

        // Delete vertices with integer values.
        for (auto it = agent_vertices.begin(); it != agent_vertices.end();)
        {
            const auto& [nt, val] = *it;
            if (SCIPisIntegral(scip, val))
            {
                it = agent_vertices.erase(it);
            }
            else
            {
                ++it;
            }
        }

        // Delete edges with integer values.
        for (auto it = agent_edges.begin(); it != agent_edges.end();)
        {
            const auto& [et, val] = *it;
            if (SCIPisIntegral(scip, val))
            {
                it = agent_edges.erase(it);
            }
            else
            {
                ++it;
            }
        }

        // Delete non-wait edges with integer values.
        for (auto it = agent_edges_no_waits.begin(); it != agent_edges_no_waits.end();)
        {
            const auto& [et, val] = *it;
            if (SCIPisIntegral(scip, val))
            {
                it = agent_edges_no_waits.erase(it);
            }
            else
            {
                ++it;
            }
        }

        // Print.
#ifdef PRINT_DEBUG
        if (!agent_vertices.empty())
        {
            println("   Fractional vertices for agent {}:", a);
            for (const auto [nt, val] : agent_vertices)
            {
                const auto [x, y] = map.get_xy(nt.n);
                println("      (({},{}),{}) val {:.4f}", x, y, nt.t, val);
            }
        }
#endif

        // Print.
#ifdef PRINT_DEBUG
        if (!agent_edges.empty())
        {
            println("   Fractional edges used by agent {}:", a);
            for (const auto [et, val] : agent_edges)
            {
                const auto [x1, y1] = map.get_xy(et.n);
                const auto [x2, y2] = map.get_destination_xy(et.et.e);
                println("      (({},{}),({},{}),{}) val {:.4f}", x1, y1, x2, y2, et.t, val);
            }
        }
#endif

        // Print.
#ifdef PRINT_DEBUG
        if (!agent_edges_no_waits.empty())
        {
            println("   Fractional non-wait edges used by agent {}:", a);
            for (const auto [et, val] : agent_edges_no_waits)
            {
                const auto [x1, y1] = map.get_xy(et.n);
                const auto [x2, y2] = map.get_destination_xy(et.et.e);
                println("      (({},{}),({},{}),{}) val {:.4f}", x1, y1, x2, y2, et.t, val);
            }
        }
#endif
    }
}

// Get pricer data
SCIP_PricerData* SCIPprobdataGetPricerData(
    SCIP_ProbData* probdata    // Problem data
)
{
    debug_assert(probdata);
    debug_assert(probdata->pricerdata);
    return probdata->pricerdata;
}

// Set pricer data
void SCIPprobdataSetPricerData(
    SCIP_ProbData* probdata,       // Problem data
    SCIP_PricerData* pricerdata    // Pricer data
)
{
    probdata->pricerdata = pricerdata;
}

// Get the map
const Map& SCIPprobdataGetMap(
    SCIP_ProbData* probdata    // Problem data
)
{
    debug_assert(probdata);
    return probdata->instance->map;
}

// Get the agents data
const AgentsData& SCIPprobdataGetAgentsData(
    SCIP_ProbData* probdata    // Problem data
)
{
    debug_assert(probdata);
    return probdata->instance->agents;
}

// Get the number of agents
Agent SCIPprobdataGetN(
    SCIP_ProbData* probdata    // Problem data
)
{
    debug_assert(probdata);
    debug_assert(probdata->N >= 1);
    return probdata->N;
}

// Get the pricing solver
AStar& SCIPprobdataGetAStar(
    SCIP_ProbData* probdata    // Problem data
)
{
    debug_assert(probdata);
    return *probdata->astar;
}

// Format path
String format_path(
    SCIP_ProbData* probdata,    // Problem data
    const Time path_length,     // Path length
    const Edge* const path      // Path
)
{
    const auto& map = SCIPprobdataGetMap(probdata);

    String str;
    Time t = 0;
    for (; t < path_length - 1; ++t)
    {
        auto [x, y] = map.get_xy(path[t].n);
#ifdef REMOVE_PADDING
        --x;
        --y;
#endif
        str += fmt::format("({},{}),", x, y);
    }
    {
        auto [x, y] = map.get_xy(path[t].n);
#ifdef REMOVE_PADDING
        --x;
        --y;
#endif
        str += fmt::format("({},{})", x, y);
    }
    return str;
}

// Format path
String format_path_spaced(
    SCIP_ProbData* probdata,    // Problem data
    const Time path_length,     // Path length
    const Edge* const path      // Path
)
{
    const auto& map = SCIPprobdataGetMap(probdata);

    String str;
    for (Time t = 0; t < path_length; ++t)
    {
        auto [x, y] = map.get_xy(path[t].n);
#ifdef REMOVE_PADDING
        --x;
        --y;
#endif
        str += fmt::format("{:>10s}", fmt::format("({},{})", x, y));
    }
    return str;
}

// Write LP relaxation to file
SCIP_RETCODE write_master(
    SCIP* scip    // SCIP
)
{
    static size_t iter = 0;
    SCIP_CALL(SCIPwriteLP(scip, fmt::format("master_{}.lp", ++iter).c_str()));
    return SCIP_OKAY;
}

// Print map
void print_map(
    SCIP_ProbData* probdata    // Problem data
)
{
    const auto& map = SCIPprobdataGetMap(probdata);
    const auto height = map.height();
    const auto width = map.width();
    for (Position y = 0; y < height; ++y)
    {
        for (Position x = 0; x < width; ++x)
        {
            const auto passable = map[map.get_id(x, y)];
            if (passable)
            {
                fmt::print(".");
            }
            else
            {
                fmt::print("T");
            }
        }
        println("");
    }
}

// Print paths with positive value
void print_used_paths(
    SCIP* scip,      // SCIP
    SCIP_SOL* sol    // Solution
)
{
    constexpr auto print_all_paths = false;

    // Get variables.
    auto probdata = SCIPgetProbData(scip);
    const auto& map = SCIPprobdataGetMap(probdata);
    const auto N = SCIPprobdataGetN(probdata);
    const auto& vars = SCIPprobdataGetVars(probdata);
    const auto& agent_vars = SCIPprobdataGetAgentVars(probdata);
    const auto& dummy_vars = SCIPprobdataGetDummyVars(probdata);

    // Calculate makespan.
    Time makespan = 0;
    for (Agent a = 0; a < N; ++a)
        for (auto var : agent_vars[a])
        {
            // Get the path length.
            debug_assert(var);
            auto vardata = SCIPvarGetData(var);
            const auto path_length = SCIPvardataGetPathLength(vardata);

            // Compute the makespan.
            if (makespan < path_length)
            {
                makespan = path_length;
            }
        }

    // Get fractional vertices.
    HashTable<NodeTime, HashTable<Agent, SCIP_Real>> vertex_times_used;
    HashTable<EdgeTime, HashTable<Agent, SCIP_Real>> edge_times_used;
    if (!sol &&
        SCIPgetStage(scip) == SCIP_STAGE_SOLVING &&
        SCIPgetLPSolstat(scip) == SCIP_LPSOLSTAT_OPTIMAL)
    {
        // Calculate the number of times a vertex or edge is used by summing the columns.
        for (auto var : vars)
        {
            // Get the path.
            debug_assert(var);
            auto vardata = SCIPvarGetData(var);
            const auto a = SCIPvardataGetAgent(vardata);
            const auto path_length = SCIPvardataGetPathLength(vardata);
            const auto path = SCIPvardataGetPath(vardata);

            // Get the variable value.
            const auto var_val = SCIPgetSolVal(scip, sol, var);

            // Sum vertex and edge values.
            if (SCIPisPositive(scip, var_val))
            {
                // Vertices.
                {
                    Time t = 1;
                    for (; t < path_length; ++t)
                    {
                        const NodeTime nt{path[t].n, t};
                        vertex_times_used[nt][a] += var_val;
                    }
                    const auto n = path[path_length - 1].n;
                    for (; t < makespan; ++t)
                    {
                        const NodeTime nt{n, t};
                        vertex_times_used[nt][a] += var_val;
                    }
                }

                // Edges.
                {
                    for (Time t = 0; t < path_length - 1; ++t)
                    {
                        const auto e = map.get_undirected_edge(path[t]);
                        const EdgeTime et(e, t);
                        edge_times_used[et][a] += var_val;
                    }
                }
            }
        }
    }

    // Print header.
    if (sol)
    {
        println("Paths from solution with objective value {:.4f}",
                SCIPgetSolOrigObj(scip, sol));
    }
    else if (SCIPgetStage(scip) == SCIP_STAGE_SOLVING)
    {
        println("Paths at node {}, depth {}, obj {:.4f}",
                SCIPnodeGetNumber(SCIPgetCurrentNode(scip)),
                SCIPgetDepth(scip),
                SCIPgetLPObjval(scip));
    }
    else
    {
        println("Paths");
    }

    // Determine if the solution is fractional.
    bool is_fractional = false;
    for (Agent a = 0; a < N; ++a)
        for (auto var : agent_vars[a])
        {
            // Get the variable value.
            debug_assert(var);
            const auto var_val = SCIPgetSolVal(scip, sol, var);

            // Store fractional status.
            if (!SCIPisIntegral(scip, var_val))
            {
                is_fractional = true;
                break;
            }
        }

    // Print time horizon.
    fmt::print("                                      ");
    for (Time t = 0; t < makespan; ++t)
    {
        fmt::print("{:10d}", t);
    }
    println("");

    // Print paths.
    for (Agent a = 0; a < N; ++a)
    {
        // Determine if using a dummy variable.
        bool is_using_dummy_path;
        {
            // Get the variable.
            auto var = dummy_vars[a];
            debug_assert(var);

            // Get the variable value.
            const auto var_val = SCIPgetSolVal(scip, sol, var);
            is_using_dummy_path = SCIPisPositive(scip, var_val);
        }

        // Print.
        bool printed = false;
        for (auto var : agent_vars[a])
        {
            // Get the path.
            debug_assert(var);
            auto vardata = SCIPvarGetData(var);
            const auto path_length = SCIPvardataGetPathLength(vardata);
            const auto path = SCIPvardataGetPath(vardata);

            // Get the variable value.
            const auto var_val = SCIPgetSolVal(scip, sol, var);

            // Print.
            if (print_all_paths ||
                is_using_dummy_path ||
                (is_fractional && !SCIPisIntegral(scip, var_val)) ||
                (!is_fractional && SCIPisPositive(scip, var_val))
               )
            {
                if (SCIPisZero(scip, var_val))
                {
                    fmt::print("   ***");
                }
                else if (SCIPisIntegral(scip, var_val))
                {
                    fmt::print("      ");
                }
                else
                {
                    fmt::print("   ---");
                }
                fmt::print("Agent: {:3d}, Val: {:7.4f}, Path: ", a, std::abs(var_val));

                for (Time t = 0; t < path_length; ++t)
                {
                    const auto e = path[t];
                    auto [x, y] = map.get_xy(e.n);
#ifdef REMOVE_PADDING
                    --x;
                    --y;
#endif

                    fmt::terminal_color colour = fmt::terminal_color::black;
                    if (auto it = vertex_times_used.find(NodeTime(e.n, t)); it != vertex_times_used.end())
                    {
                        const auto n = it->second.size();
                        if (n > 1)
                        {
                            colour = fmt::terminal_color::blue;
                        }
                    }
                    if (auto it = edge_times_used.find(EdgeTime(map.get_undirected_edge(path[t]), t)); it != edge_times_used.end())
                    {
                        const auto n = it->second.size();
                        if (n > 1)
                        {
                            colour = fmt::terminal_color::green;
                        }
                    }

                    if (colour != fmt::terminal_color::black)
                    {
                        if (t < path_length - 1 && e.n == path[t+1].n)
                        {
                            fmt::print(fmt::emphasis::bold | fg(colour), "{:>10s}", fmt::format("({},{})", x, y));
                        }
                        else
                        {
                            fmt::print(fg(colour), "{:>10s}", fmt::format("({},{})", x, y));
                        }
                    }
                    else
                    {
                        if (t < path_length - 1 && e.n == path[t+1].n)
                        {
                            fmt::print(fmt::emphasis::bold, "{:>10s}", fmt::format("({},{})", x, y));
                        }
                        else
                        {
                            fmt::print("{:>10s}", fmt::format("({},{})", x, y));
                        }
                    }
                }
                println("");
                printed = true;
            }
        }
        if (is_fractional && printed)
        {
            println("");
        }
    }
}

// Print dual variable values
void print_agent_part_dual(
    SCIP* scip,             // SCIP
    const bool is_farkas    // Indicates if the master problem is infeasible
)
{
    auto probdata = SCIPgetProbData(scip);
    const auto N = SCIPprobdataGetN(probdata);

    const auto& conss = SCIPprobdataGetAgentPartConss(probdata);
    for (Agent a = 0; a < N; ++a)
    {
        auto cons = conss[a];
        if (SCIPconsIsActive(cons))
        {
            const auto dual = is_farkas ?
                              SCIPgetDualfarkasSetppc(scip, cons) :
                              SCIPgetDualsolSetppc(scip, cons);
            if (dual != 0.0)
            {
                println("   Dual of agent_part({}) = {}", a, dual);
            }
        }
    }
}
void print_vertex_conflicts_dual(
    SCIP* scip,             // SCIP
    const bool is_farkas    // Indicates if the master problem is infeasible
)
{
    auto probdata = SCIPgetProbData(scip);
    const auto& conflicts = vertex_conflicts_get_constraints(probdata);
    for (const auto& [nt, vertex_conflict] : conflicts)
    {
        const auto& [row] = vertex_conflict;
        const auto row_dual = SCIProwIsInLP(row) ?
                              (is_farkas ? SCIProwGetDualfarkas(row) : SCIProwGetDualsol(row)) :
                              0.0;
        if (!SCIPisZero(scip, row_dual))
        {
            println("   Dual of {} = {:.4f}", SCIProwGetName(row), row_dual);
        }
    }
}
void print_edge_conflicts_dual(
    SCIP* scip,             // SCIP
    const bool is_farkas    // Indicates if the master problem is infeasible
)
{
    auto probdata = SCIPgetProbData(scip);
    const auto& conflicts = edge_conflicts_get_constraints(probdata);
    for (const auto& [et, edge_conflict] : conflicts)
    {
        const auto& [row, edges, t] = edge_conflict;
        const auto row_dual = SCIProwIsInLP(row) ?
                              (is_farkas ? SCIProwGetDualfarkas(row) : SCIProwGetDualsol(row)) :
                              0.0;
        if (!SCIPisZero(scip, row_dual))
        {
            println("   Dual of {} = {:.4f}", SCIProwGetName(row), row_dual);
        }
    }
}
void print_two_agent_robust_cuts_dual(
    SCIP* scip,             // SCIP
    const bool is_farkas    // Indicates if the master problem is infeasible
)
{
    auto probdata = SCIPgetProbData(scip);
    const auto& two_agent_robust_cuts = SCIPprobdataGetTwoAgentRobustCuts(probdata);
    for (const auto& cut : two_agent_robust_cuts)
    {
        auto row = cut.row();
        const auto row_dual =
            SCIProwIsInLP(row) ?
            (is_farkas ? SCIProwGetDualfarkas(row) : SCIProwGetDualsol(row)) :
            0.0;
        if (!SCIPisZero(scip, row_dual))
        {
            println("   Dual of {} = {:.4f}", SCIProwGetName(row), row_dual);
        }
    }
}
#ifdef USE_RECTANGLE_CLIQUE_CONFLICTS
void print_rectangle_clique_conflicts_dual(
    SCIP* scip,             // SCIP
    const bool is_farkas    // Indicates if the master problem is infeasible
)
{
    auto probdata = SCIPgetProbData(scip);
    const auto& conflicts = rectangle_clique_conflicts_get_constraints(probdata);
    for (const auto& rect_conflict : conflicts)
    {
        auto row = rect_conflict.row;
        const auto row_dual =
            SCIProwIsInLP(row) ?
            (is_farkas ? SCIProwGetDualfarkas(row) : SCIProwGetDualsol(row)) :
            0.0;
        if (!SCIPisZero(scip, row_dual))
        {
            println("   Dual of {} = {:.4f}", SCIProwGetName(row), row_dual);
        }
    }
}
#endif
#ifdef USE_GOAL_CONFLICTS
void print_goal_conflicts_dual(
    SCIP* scip,             // SCIP
    const bool is_farkas    // Indicates if the master problem is infeasible
)
{
    auto probdata = SCIPgetProbData(scip);
    const auto& goal_conflicts = SCIPprobdataGetGoalConflicts(probdata);
    for (const auto& goal_conflict : goal_conflicts)
    {
        auto row = goal_conflict.row;
        const auto row_dual =
            SCIProwIsInLP(row) ?
            (is_farkas ? SCIProwGetDualfarkas(row) : SCIProwGetDualsol(row)) :
            0.0;
        if (!SCIPisZero(scip, row_dual))
        {
            println("   Dual of {} = {:.4f}", SCIProwGetName(row), row_dual);
        }
    }
}
#endif

#ifdef DEBUG
SCIP_Real get_coeff(SCIP_ROW* row, SCIP_VAR* var)
{
    const auto col = SCIPvarGetCol(var);

    const auto n = SCIProwGetNNonz(row);
    const auto cols = SCIProwGetCols(row);
    const auto coeffs = SCIProwGetVals(row);
    for (int i = 0; i < n; ++i)
        if (col == cols[i])
        {
            return coeffs[i];
        }
    return 0.0;
}
#endif
