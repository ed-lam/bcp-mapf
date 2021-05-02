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

#include "BranchingRule.h"
#include "Includes.h"
#include "ProblemData.h"
#include "VariableData.h"
#include "Constraint_VertexBranching.h"
#include <iterator>

HashTable<AgentNodeTime, Int>
get_pseudosolution_branch_candidates(
    SCIP* scip    // SCIP
)
{
    // Get candidate variables.
    SCIP_VAR** candidate_vars;
    int nb_candidate_vars;
    scip_assert(SCIPgetPseudoBranchCands(scip, &candidate_vars, nullptr, &nb_candidate_vars));
    debug_assert(nb_candidate_vars > 0);

    // Calculate branching candidates.
    HashTable<AgentNodeTime, Int> candidates;
    for (Int v = 0; v < nb_candidate_vars; ++v)
    {
        // Get the variable.
        auto var = candidate_vars[v];
        debug_assert(var);

        // Proceed if not artificial variable.
        auto vardata = SCIPvarGetData(var);
        if (vardata)
        {
            // Get the path.
            const auto a = SCIPvardataGetAgent(vardata);
            const auto path_length = SCIPvardataGetPathLength(vardata);
            const auto path = SCIPvardataGetPath(vardata);

            // Update candidates data.
            for (Time t = 1; t < path_length - 1; ++t)
            {
                candidates[AgentNodeTime{a, path[t].n, t}]++;
            }
        }
    }
    release_assert(!candidates.empty(), "No candidates for branching in pseudosolution");
    return candidates;
}

SCIP_RETCODE branch_pseudosolution(
    SCIP* scip,            // SCIP
    SCIP_RESULT* result    // Output status
)
{
    // Print.
    debugln("Branching on pseudosolution at node {}, depth {}:",
            SCIPnodeGetNumber(SCIPgetCurrentNode(scip)),
            SCIPgetDepth(scip));

    // Get constraints for vertex branching decisions.
    auto conshdlr = SCIPfindConshdlr(scip, "vertex_branching");
    const auto n_vertex_branching_conss = SCIPconshdlrGetNConss(conshdlr);
    auto vertex_branching_conss = SCIPconshdlrGetConss(conshdlr);
    debug_assert(n_vertex_branching_conss == 0 || vertex_branching_conss);

    // Get branching candidates.
    const auto candidates = get_pseudosolution_branch_candidates(scip);

    // Pick a vertex that hasn't been branched on before.
    for (const auto [ant, count] : candidates)
    {
        // Loop through decisions in ancestors of this node.
        for (Int c = 0; c < n_vertex_branching_conss; ++c)
        {
            // Get the constraint.
            auto cons = vertex_branching_conss[c];
            debug_assert(cons);

            // Ignore constraints that are not active since these are not on the current
            // active path of the search tree.
            if (!SCIPconsIsActive(cons))
            {
                continue;
            }

            // Get the decision.
            const auto a = SCIPgetVertexBranchingAgent(cons);
            const auto nt = SCIPgetVertexBranchingNodeTime(cons);
            const AgentNodeTime decision{.a = a, .n = nt.n, .t = nt.t};

            // Skip the vertex if already branched on.
            if (decision == ant)
            {
                goto NEXT_VERTEX;
            }
        }

        // Branch.
        branch_on_vertex(scip, ant, false);
        *result = SCIP_BRANCHED;
        return SCIP_OKAY;

        // Try the next vertex.
        NEXT_VERTEX:;
    }
    unreachable();
}
