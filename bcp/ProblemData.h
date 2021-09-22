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

#ifndef MAPF_PROBDATA_H
#define MAPF_PROBDATA_H

#include "Includes.h"
#include "Coordinates.h"
#include "Separator.h"

#include "trufflehog/Instance.h"
#include "trufflehog/AStar.h"

#ifdef USE_GOAL_CONFLICTS
struct GoalConflict
{
    SCIP_ROW* row;    // LP row
    Agent a1;         // Agent of the goal
    Agent a2;         // Agent trying to use the goal vertex
    NodeTime nt;      // Node-time of the conflict
};
#endif

#ifdef USE_PATH_LENGTH_NOGOODS
struct PathLengthNogood
{
    SCIP_ROW* row;                                    // LP row
    Vector<Pair<Agent, Time>> latest_finish_times;    // Latest time for an agent to reach its target
};
#endif

// Create problem data
SCIP_RETCODE SCIPprobdataCreate(
    SCIP* scip,                       // SCIP
    const char* probname,             // Problem name
    SharedPtr<Instance>& instance,    // Instance
    SharedPtr<AStar>& astar           // Search algorithm
);

// Add a new variable for an warm-start solution
SCIP_RETCODE SCIPprobdataAddInitialVar(
    SCIP* scip,                 // SCIP
    SCIP_ProbData* probdata,    // Problem data
    const Agent a,              // Agent
    const Time path_length,     // Path length
    const Edge* const path,     // Path
    SCIP_VAR** var              // Output new variable
);

// Add a new variable from pricing
SCIP_RETCODE SCIPprobdataAddPricedVar(
    SCIP* scip,                 // SCIP
    SCIP_ProbData* probdata,    // Problem data
    const Agent a,              // Agent
    const Time path_length,     // Path length
    const Edge* const path,     // Path
    SCIP_VAR** var              // Output new variable
);

// Add a new two-agent robust cut
SCIP_RETCODE SCIPprobdataAddTwoAgentRobustCut(
    SCIP* scip,                 // SCIP
    SCIP_ProbData* probdata,    // Problem data
    SCIP_SEPA* sepa,            // Separator
    TwoAgentRobustCut&& cut,    // Data for the cut
    const SCIP_Real rhs,        // RHS
    SCIP_RESULT* result,        // Output result
    Int* idx = nullptr          // Output index of the cut
);

// Get array of variables
Vector<SCIP_VAR*>& SCIPprobdataGetVars(
    SCIP_ProbData* probdata    // Problem data
);

// Get array of dummy variables
Vector<SCIP_VAR*>& SCIPprobdataGetDummyVars(
    SCIP_ProbData* probdata    // Problem data
);

// Get array of variables for each agent
Vector<Vector<SCIP_VAR*>>& SCIPprobdataGetAgentVars(
    SCIP_ProbData* probdata    // Problem data
);

// Get agent partition constraints
Vector<SCIP_CONS*>& SCIPprobdataGetAgentPartConss(
    SCIP_ProbData* probdata    // Problem data
);

// Get constraint for vertex conflicts
SCIP_CONS* SCIPprobdataGetVertexConflictsCons(
    SCIP_ProbData* probdata    // Problem data
);

// Get constraint for edge conflicts
SCIP_CONS* SCIPprobdataGetEdgeConflictsCons(
    SCIP_ProbData* probdata    // Problem data
);

// Get array of two-agent robust cuts
Vector<TwoAgentRobustCut>& SCIPprobdataGetTwoAgentRobustCuts(
    SCIP_ProbData* probdata    // Problem data
);

// Get separator for rectangle knapsack conflicts
#ifdef USE_RECTANGLE_KNAPSACK_CONFLICTS
SCIP_SEPA* SCIPprobdataGetRectangleKnapsackConflictsSepa(
    SCIP_ProbData* probdata    // Problem data
);
#endif

// Get separator for rectangle clique conflicts
#ifdef USE_RECTANGLE_CLIQUE_CONFLICTS
SCIP_SEPA* SCIPprobdataGetRectangleCliqueConflictsSepa(
    SCIP_ProbData* probdata    // Problem data
);
#endif

// Get goal conflicts
#ifdef USE_GOAL_CONFLICTS
Vector<GoalConflict>& SCIPprobdataGetGoalConflicts(
SCIP_ProbData* probdata    // Problem data
);
#endif

// Get path length nogoods
#ifdef USE_PATH_LENGTH_NOGOODS
Vector<PathLengthNogood>& SCIPprobdataGetPathLengthNogoods(
    SCIP_ProbData* probdata    // Problem data
);
#endif

// Get array of two-agent robust cuts grouped by agent
Vector<Vector<AgentRobustCut>>& SCIPprobdataGetAgentRobustCuts(
    SCIP_ProbData* probdata    // Problem data
);

// Get array of vertex conflicts at the goal of an agent
Vector<Vector<Pair<Time, SCIP_ROW*>>>& SCIPprobdataGetAgentGoalVertexConflicts(
    SCIP_ProbData* probdata    // Problem data
);

// Get array of edge conflicts at the goal of an agent
#ifdef USE_WAITEDGE_CONFLICTS
Vector<Vector<Pair<Time, SCIP_ROW*>>>& SCIPprobdataGetAgentGoalEdgeConflicts(
    SCIP_ProbData* probdata    // Problem data
);
#endif

// Get the vertices fractionally used by each agent
const Vector<HashTable<NodeTime, SCIP_Real>>& SCIPprobdataGetAgentFractionalVertices(
    SCIP_ProbData* probdata    // Problem data
);

// Get the edges fractionally used by each agent
const Vector<HashTable<EdgeTime, SCIP_Real>>& SCIPprobdataGetAgentFractionalEdges(
    SCIP_ProbData* probdata    // Problem data
);

// Get the non-wait edges fractionally used by each agent
const Vector<HashTable<EdgeTime, SCIP_Real>>& SCIPprobdataGetAgentFractionalEdgesNoWaits(
    SCIP_ProbData* probdata    // Problem data
);

// Get the edges fractionally used by each agent grouped by edge-time
const HashTable<EdgeTime, Vector<SCIP_Real>>& SCIPprobdataGetAgentFractionalEdgesVec(
    SCIP_ProbData* probdata    // Problem data
);

// Update the database of fractional vertices and edges
void update_fractional_vertices_and_edges(
    SCIP* scip    // SCIP
);

// Get pricer data
SCIP_PricerData* SCIPprobdataGetPricerData(
    SCIP_ProbData* probdata    // Problem data
);

// Set pricer data
void SCIPprobdataSetPricerData(
    SCIP_ProbData* probdata,       // Problem data
    SCIP_PricerData* pricerdata    // Pricer data
);

// Get the map
const Map& SCIPprobdataGetMap(
    SCIP_ProbData* probdata    // Problem data
);

// Get the number of agents
Agent SCIPprobdataGetN(
    SCIP_ProbData* probdata    // Problem data
);

// Get the agents data
const AgentsData& SCIPprobdataGetAgentsData(
    SCIP_ProbData* probdata    // Problem data
);

// Get the pricing solver
AStar& SCIPprobdataGetAStar(
    SCIP_ProbData* probdata    // Problem data
);

// Format path
String format_path(
    SCIP_ProbData* probdata,    // Problem data
    const Time path_length,     // Path length
    const Edge* const path      // Path
);

// Format path
String format_path_spaced(
    SCIP_ProbData* probdata,    // Problem data
    const Time path_length,     // Path length
    const Edge* const path      // Path
);

// Write LP relaxation to file
SCIP_RETCODE write_master(
    SCIP* scip    // SCIP
);

// Print map
void print_map(
    SCIP_ProbData* probdata    // Problem data
);

// Print paths with positive value
void print_used_paths(
    SCIP* scip,               // SCIP
    SCIP_SOL* sol = nullptr   // Solution
);

// Print dual variable values
void print_agent_part_dual(
    SCIP* scip,             // SCIP
    const bool is_farkas    // Indicates if the master problem is infeasible
);
void print_vertex_conflicts_dual(
    SCIP* scip,             // SCIP
    const bool is_farkas    // Indicates if the master problem is infeasible
);
void print_edge_conflicts_dual(
    SCIP* scip,             // SCIP
    const bool is_farkas    // Indicates if the master problem is infeasible
);
void print_two_agent_robust_cuts_dual(
    SCIP* scip,             // SCIP
    const bool is_farkas    // Indicates if the master problem is infeasible
);
#ifdef USE_RECTANGLE_CLIQUE_CONFLICTS
void print_rectangle_clique_conflicts_dual(
    SCIP* scip,             // SCIP
    const bool is_farkas    // Indicates if the master problem is infeasible
);
#endif
#ifdef USE_GOAL_CONFLICTS
void print_goal_conflicts_dual(
    SCIP* scip,             // SCIP
    const bool is_farkas    // Indicates if the master problem is infeasible
);
#endif

// Get coefficient of a variable in a constraint
#ifdef DEBUG
SCIP_Real get_coeff(SCIP_ROW* row, SCIP_VAR* var);
#endif

#endif
