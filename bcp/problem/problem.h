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

#pragma once

#include "problem/includes.h"
#include "types/map_types.h"
#include "constraints/separator.h"

#include "problem/instance.h"
#include "pricing/astar.h"

#ifdef USE_GOAL_CONFLICTS
struct GoalConflict
{
    SCIP_ROW* row;    // LP row
    Agent a1;         // Agent of the goal
    Agent a2;         // Agent trying to use the goal vertex
    NodeTime nt;      // Node-time of the conflict
};
#endif

// Create problem data
SCIP_RETCODE SCIPprobdataCreate(
    SCIP* scip,                       // SCIP
    const char* probname,             // Problem name
    SharedPtr<Instance>& instance,    // Instance
    SharedPtr<AStar>& astar           // Search algorithm
);

// Add a new variable from a primal heuristic
SCIP_RETCODE SCIPprobdataAddHeuristicVar(
    SCIP* scip,                 // SCIP
    SCIP_ProbData* probdata,    // Problem data
    const Agent a,              // Agent
    const Time path_length,     // Path length
    const Edge* const path,     // Path
    SCIP_VAR** var              // Output new variable
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

// Get array of dummy variables
Vector<SCIP_VAR*>& SCIPprobdataGetDummyVars(
    SCIP_ProbData* probdata    // Problem data
);

// Get array of variables for all agents
Vector<Pair<SCIP_VAR*, SCIP_Real>>& SCIPprobdataGetVars(
    SCIP_ProbData* probdata    // Problem data
);

// Get array of variables for each agent
Vector<Vector<Pair<SCIP_VAR*, SCIP_Real>>>& SCIPprobdataGetAgentVars(
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

// Get goal conflicts
#ifdef USE_GOAL_CONFLICTS
Vector<GoalConflict>& SCIPprobdataGetGoalConflicts(
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

// Get array of goal conflicts of an agent whose goal is in conflict
#ifdef USE_GOAL_CONFLICTS
Vector<Vector<Pair<Time, SCIP_ROW*>>>& SCIPprobdataGetGoalAgentGoalConflicts(
    SCIP_ProbData* probdata    // Problem data
);
#endif

// Get array of goal conflicts of an agent crossing the goal of another agent
#ifdef USE_GOAL_CONFLICTS
Vector<Vector<Pair<NodeTime, SCIP_ROW*>>>& SCIPprobdataGetCrossingAgentGoalConflicts(
    SCIP_ProbData* probdata    // Problem data
);
#endif

// Get the vertices fractionally used by each agent
const Vector<HashMap<NodeTime, SCIP_Real>>& SCIPprobdataGetFractionalVertices(
    SCIP_ProbData* probdata    // Problem data
);

// Get the edges fractionally used by each agent
const Vector<HashMap<EdgeTime, SCIP_Real>>& SCIPprobdataGetFractionalEdges(
    SCIP_ProbData* probdata    // Problem data
);

// Get the non-wait edges fractionally used by each agent
const Vector<HashMap<EdgeTime, SCIP_Real>>& SCIPprobdataGetFractionalMoveEdges(
    SCIP_ProbData* probdata    // Problem data
);

// Get the non-wait edges used by each agent
const Vector<HashMap<EdgeTime, SCIP_Real>>& SCIPprobdataGetPositiveMoveEdges(
    SCIP_ProbData* probdata    // Problem data
);

// Get the edges fractionally used by each agent, grouped by edge-time
const HashMap<EdgeTime, SCIP_Real*>& SCIPprobdataGetFractionalEdgesVec(
    SCIP_ProbData* probdata    // Problem data
);

// Update the database of fractional vertices and edges
void update_fractional_vertices_and_edges(
    SCIP* scip    // SCIP
);

// Update the arrays of variable values
void update_variable_values(
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

// Get found cuts indicator
bool& SCIPprobdataGetFoundCutsIndicator(
    SCIP_ProbData* probdata    // Problem data
);

// Get the scenario path
const std::filesystem::path& SCIPprobdataGetScenarioPath(
    SCIP_ProbData* probdata    // Problem data
);

// Get the map path
const std::filesystem::path& SCIPprobdataGetMapPath(
    SCIP_ProbData* probdata    // Problem data
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
const Vector<AgentData>& SCIPprobdataGetAgentsData(
    SCIP_ProbData* probdata    // Problem data
);

// Get the pricing solver
AStar& SCIPprobdataGetAStar(
    SCIP_ProbData* probdata    // Problem data
);

// Get the maximum path length
Time SCIPprobdataGetMaxPathLength(
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
