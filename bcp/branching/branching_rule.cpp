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
#include "Constraint_WaitBranching.h"
#include "Constraint_LengthBranching.h"

// Branching rule properties
#define BRANCHRULE_NAME            "mapf"
#define BRANCHRULE_DESC            "MAPF branching rule"
#define BRANCHRULE_PRIORITY        50000
#define BRANCHRULE_MAXDEPTH        -1
#define BRANCHRULE_MAXBOUNDDIST    1.0

// Branching execution method for fractional LP solutions
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_BRANCHEXECLP(branchExeclpMAPF)
{
    // Check.
    debug_assert(scip);
    debug_assert(branchrule);
    debug_assert(strcmp(SCIPbranchruleGetName(branchrule), BRANCHRULE_NAME) == 0);
    debug_assert(result);

    // Branch.
    return branch_lp(scip, result);
}
#pragma GCC diagnostic pop

// Branching execution method for pseudosolutions
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_BRANCHEXECPS(branchExecpsMAPF)
{
    // Check.
    debug_assert(scip);
    debug_assert(branchrule);
    debug_assert(strcmp(SCIPbranchruleGetName(branchrule), BRANCHRULE_NAME) == 0);
    debug_assert(result);

    // Branch.
    return branch_pseudosolution(scip, result);
}
#pragma GCC diagnostic pop

// Unused branching method
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_BRANCHEXECEXT(branchExecextMAPF)
{
    unreachable();
}
#pragma GCC diagnostic pop

// Create the branching rule and include it in SCIP
SCIP_RETCODE SCIPincludeBranchrule(
    SCIP* scip    // SCIP
)
{
    // Include branching rule.
    SCIP_BRANCHRULE* branchrule = nullptr;
    SCIP_CALL(SCIPincludeBranchruleBasic(scip,
                                         &branchrule,
                                         BRANCHRULE_NAME,
                                         BRANCHRULE_DESC,
                                         BRANCHRULE_PRIORITY,
                                         BRANCHRULE_MAXDEPTH,
                                         BRANCHRULE_MAXBOUNDDIST,
                                         nullptr));
    debug_assert(branchrule);

    // Activate branching rule.
    SCIP_CALL(SCIPsetBranchruleExecLp(scip, branchrule, branchExeclpMAPF));
    SCIP_CALL(SCIPsetBranchruleExecPs(scip, branchrule, branchExecpsMAPF));
    SCIP_CALL(SCIPsetBranchruleExecExt(scip, branchrule, branchExecextMAPF));

    // Done.
    return SCIP_OKAY;
}

SCIP_RETCODE branch_on_vertex(
    SCIP* scip,                   // SCIP
    const AgentNodeTime ant,      // Branch decision
    const bool prefer_branch_0    // Preferred branch direction
)
{
    // Print.
#if defined(PRINT_DEBUG) or defined(DEBUG)
    Position x, y;
    {
        auto probdata = SCIPgetProbData(scip);
        const auto& map = SCIPprobdataGetMap(probdata);
        x = map.get_x(ant.n);
        y = map.get_y(ant.n);
        debugln("   Branching on agent {}, vertex (({},{}),{}) at node {}, depth {}",
                ant.a, x, y, ant.t,
                SCIPnodeGetNumber(SCIPgetCurrentNode(scip)),
                SCIPgetDepth(scip));
    }
#endif

    // Check that the decision hasn't been branched on already.
#ifdef DEBUG
    {
        // Get constraints for vertex branching decisions.
        auto conshdlr = SCIPfindConshdlr(scip, "vertex_branching");
        const auto n_vertex_branching_conss = SCIPconshdlrGetNConss(conshdlr);
        auto vertex_branching_conss = SCIPconshdlrGetConss(conshdlr);
        debug_assert(n_vertex_branching_conss == 0 || vertex_branching_conss);

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

            // Check.
            release_assert(decision != ant);
        }
    }
#endif

    // Create children nodes.
    const auto score = SCIPgetLocalTransEstimate(scip);
    SCIP_NODE* branch_1_node;
    SCIP_NODE* branch_0_node;
    SCIP_CALL(SCIPcreateChild(scip,
                              &branch_1_node,
                              1.0 - static_cast<SCIP_Real>(prefer_branch_0),
                              score + (prefer_branch_0 ? 1.0 : 0.0)));
    SCIP_CALL(SCIPcreateChild(scip,
                              &branch_0_node,
                              static_cast<SCIP_Real>(prefer_branch_0),
                              score + (prefer_branch_0 ? 0.0 : 1.0)));

    // Create local constraints that enforce the branching in the children nodes.
    SCIP_CONS* branch_1_cons;
    SCIP_CONS* branch_0_cons;
#ifdef DEBUG
    const auto branch_1_name = fmt::format("branch_use({},({},{}),{})",
                                           ant.a, x, y, ant.t);
    const auto branch_0_name = fmt::format("branch_forbid({},({},{}),{})",
                                           ant.a, x, y, ant.t);
#endif
    SCIP_CALL(SCIPcreateConsVertexBranching(scip,
                                            &branch_1_cons,
#ifdef DEBUG
                                            branch_1_name.c_str(),
#else
                                            "",
#endif
                                            VertexBranchDirection::Use,
                                            ant.a,
                                            NodeTime(ant.n, ant.t),
                                            branch_1_node,
                                            TRUE));
    SCIP_CALL(SCIPcreateConsVertexBranching(scip,
                                            &branch_0_cons,
#ifdef DEBUG
                                            branch_0_name.c_str(),
#else
                                            "",
#endif
                                            VertexBranchDirection::Forbid,
                                            ant.a,
                                            NodeTime(ant.n, ant.t),
                                            branch_0_node,
                                            TRUE));

    // Add the constraints to the nodes.
    SCIP_CALL(SCIPaddConsNode(scip, branch_1_node, branch_1_cons, nullptr));
    SCIP_CALL(SCIPaddConsNode(scip, branch_0_node, branch_0_cons, nullptr));

    // Decrease reference counter of the constraints.
    SCIP_CALL(SCIPreleaseCons(scip, &branch_1_cons));
    SCIP_CALL(SCIPreleaseCons(scip, &branch_0_cons));

    // Return.
    return SCIP_OKAY;
}

//SCIP_RETCODE branch_on_wait(
//    SCIP* scip,                   // SCIP
//    const AgentTime at,           // Branch decision
//    const bool prefer_branch_0    // Preferred branch direction
//)
//{
//    // Print.
//#if defined(PRINT_DEBUG) or defined(DEBUG)
//    debugln("   Branching on agent {}, wait at time {} at node {}, depth {}",
//            at.a, at.t,
//            SCIPnodeGetNumber(SCIPgetCurrentNode(scip)),
//            SCIPgetDepth(scip));
//#endif
//
//    // Check that the decision hasn't been branched on already.
//#ifdef DEBUG
//    {
//        // Get constraints for vertex branching decisions.
//        auto conshdlr = SCIPfindConshdlr(scip, "wait_branching");
//        const auto n_wait_branching_conss = SCIPconshdlrGetNConss(conshdlr);
//        auto wait_branching_conss = SCIPconshdlrGetConss(conshdlr);
//        debug_assert(n_wait_branching_conss == 0 || wait_branching_conss);
//
//        // Loop through decisions in ancestors of this node.
//        for (Count c = 0; c < n_wait_branching_conss; ++c)
//        {
//            // Get the constraint.
//            auto cons = wait_branching_conss[c];
//            debug_assert(cons);
//
//            // Ignore constraints that are not active since these are not on the current
//            // active path of the search tree.
//            if (!SCIPconsIsActive(cons))
//            {
//                continue;
//            }
//
//            // Get the decision.
//            const auto a = SCIPgetWaitBranchingAgent(cons);
//            const auto t = SCIPgetWaitBranchingTime(cons);
//            const AgentTime decision{a, t};
//
//            // Check.
//            release_assert(decision != at);
//        }
//    }
//#endif
//
//    // Create two children nodes.
//    const auto score = SCIPgetLocalTransEstimate(scip);
//    SCIP_NODE* branch_1_node;
//    SCIP_NODE* branch_0_node;
//    SCIP_CALL(SCIPcreateChild(scip,
//                              &branch_1_node,
//                              1.0 - static_cast<SCIP_Real>(prefer_branch_0),
//                              score + (prefer_branch_0 ? 1.0 : 0.0)));
//    SCIP_CALL(SCIPcreateChild(scip,
//                              &branch_0_node,
//                              static_cast<SCIP_Real>(prefer_branch_0),
//                              score + (prefer_branch_0 ? 0.0 : 1.0)));
//
//    // Create local constraints that enforce the branching in the children nodes.
//    SCIP_CONS* branch_1_cons;
//    SCIP_CONS* branch_0_cons;
//#ifdef DEBUG
//    const auto branch_1_name = fmt::format("branch_must_wait({},{})", at.a, at.t);
//    const auto branch_0_name = fmt::format("branch_cannot_wait({},{})", at.a, at.t);
//#endif
//    SCIP_CALL(SCIPcreateConsWaitBranching(scip,
//                                          &branch_1_cons,
//#ifdef DEBUG
//                                          branch_1_name.c_str(),
//#else
//                                          "",
//#endif
//                                          WaitBranchDirection::MustWait,
//                                          at.a,
//                                          at.t,
//                                          branch_1_node,
//                                          TRUE));
//    SCIP_CALL(SCIPcreateConsWaitBranching(scip,
//                                          &branch_0_cons,
//#ifdef DEBUG
//                                          branch_0_name.c_str(),
//#else
//                                          "",
//#endif
//                                          WaitBranchDirection::CannotWait,
//                                          at.a,
//                                          at.t,
//                                          branch_0_node,
//                                          TRUE));
//
//    // Add the constraints to the nodes.
//    SCIP_CALL(SCIPaddConsNode(scip, branch_1_node, branch_1_cons, nullptr));
//    SCIP_CALL(SCIPaddConsNode(scip, branch_0_node, branch_0_cons, nullptr));
//
//    // Decrease reference counter of the constraints.
//    SCIP_CALL(SCIPreleaseCons(scip, &branch_1_cons));
//    SCIP_CALL(SCIPreleaseCons(scip, &branch_0_cons));
//
//    // Return.
//    return SCIP_OKAY;
//}

SCIP_RETCODE branch_on_length(
    SCIP* scip,                   // SCIP
    const AgentNodeTime ant,      // Branch decision
    const bool prefer_branch_0    // Preferred branch direction
)
{
    // Print.
#if defined(PRINT_DEBUG) or defined(DEBUG)
    Position x, y;
    {
        auto probdata = SCIPgetProbData(scip);
        const auto& map = SCIPprobdataGetMap(probdata);
        x = map.get_x(ant.n);
        y = map.get_y(ant.n);
        debugln("   Branching on agent {}, goal time leq {}/geq {} at node {}, depth {}",
                ant.a, ant.t, ant.t + 1,
                SCIPnodeGetNumber(SCIPgetCurrentNode(scip)),
                SCIPgetDepth(scip));
    }
#endif

    // Check that the decision hasn't been branched on already.
#ifdef DEBUG
    {
        // Get constraints for vertex branching decisions.
        auto conshdlr = SCIPfindConshdlr(scip, "length_branching");
        const auto n_length_branching_conss = SCIPconshdlrGetNConss(conshdlr);
        auto length_branching_conss = SCIPconshdlrGetConss(conshdlr);
        debug_assert(n_length_branching_conss == 0 || length_branching_conss);

        // Loop through decisions in ancestors of this node.
        Time earliest_finish = 0;
        Time latest_finish = std::numeric_limits<Time>::max();
        for (Int c = 0; c < n_length_branching_conss; ++c)
        {
            // Get the constraint.
            auto cons = length_branching_conss[c];
            debug_assert(cons);

            // Ignore constraints that are not active since these are not on the current
            // active path of the search tree.
            if (!SCIPconsIsActive(cons))
            {
                continue;
            }

            // Get the decision.
            const auto a = SCIPgetLengthBranchingAgent(cons);
            const auto dir = SCIPgetLengthBranchingDirection(cons);
            const auto t = SCIPgetLengthBranchingNodeTime(cons).t;

            // Store the decision.
            if (a == ant.a)
            {
                if (dir == LengthBranchDirection::LEq && t < latest_finish)
                {
                    latest_finish = t;
                }
                else if (dir == LengthBranchDirection::GEq && t > earliest_finish)
                {
                    earliest_finish = t;
                }
            }
        }
        release_assert(ant.t < latest_finish || ant.t >= earliest_finish);
    }
#endif

    // Create children nodes.
    const auto score = SCIPgetLocalTransEstimate(scip);
    SCIP_NODE* branch_1_node;
    SCIP_NODE* branch_0_node;
    SCIP_CALL(SCIPcreateChild(scip,
                              &branch_1_node,
                              1.0 - static_cast<SCIP_Real>(prefer_branch_0),
                              score + (prefer_branch_0 ? 1.0 : 0.0)));
    SCIP_CALL(SCIPcreateChild(scip,
                              &branch_0_node,
                              static_cast<SCIP_Real>(prefer_branch_0),
                              score + (prefer_branch_0 ? 0.0 : 1.0)));

    // Create local constraints that enforce the branching in the children nodes.
    SCIP_CONS* branch_1_cons;
    SCIP_CONS* branch_0_cons;
#ifdef DEBUG
    const auto branch_1_name = fmt::format("branch_length_geq({},{})",
                                           ant.a, x, y, ant.t + 1);
    const auto branch_0_name = fmt::format("branch_length_leq({},{})",
                                           ant.a, x, y, ant.t);
#endif
    SCIP_CALL(SCIPcreateConsLengthBranching(scip,
                                            &branch_1_cons,
#ifdef DEBUG
                                            branch_1_name.c_str(),
#else
                                            "",
#endif
                                            LengthBranchDirection::GEq,
                                            ant.a,
                                            NodeTime(ant.n, ant.t + 1),
                                            branch_1_node,
                                            TRUE));
    SCIP_CALL(SCIPcreateConsLengthBranching(scip,
                                            &branch_0_cons,
#ifdef DEBUG
                                            branch_0_name.c_str(),
#else
                                            "",
#endif
                                            LengthBranchDirection::LEq,
                                            ant.a,
                                            NodeTime(ant.n, ant.t),
                                            branch_0_node,
                                            TRUE));

    // Add the constraints to the nodes.
    SCIP_CALL(SCIPaddConsNode(scip, branch_1_node, branch_1_cons, nullptr));
    SCIP_CALL(SCIPaddConsNode(scip, branch_0_node, branch_0_cons, nullptr));

    // Decrease reference counter of the constraints.
    SCIP_CALL(SCIPreleaseCons(scip, &branch_1_cons));
    SCIP_CALL(SCIPreleaseCons(scip, &branch_0_cons));

    // Return.
    return SCIP_OKAY;
}
