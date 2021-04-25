cmake_minimum_required(VERSION 3.2)
project(bcp-mapf)

# Verbose.
#set(CMAKE_VERBOSE_MAKEFILE on)

# Set C version.
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED 1)

# Set C++ version.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED 1)

# Set release build.
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

# Set source files.
set(TRUFFLEHOG_SOURCE_FILES
    robin-hood-hashing/src/include/robin_hood.h
    trufflehog/Includes.h
    trufflehog/Debug.h
    trufflehog/Coordinates.h
    trufflehog/Map.h
    trufflehog/AgentsData.h
    trufflehog/Instance.h
    trufflehog/Instance.cpp
    trufflehog/PriorityQueue.h
    trufflehog/LabelPool.h
    trufflehog/LabelPool.cpp
    trufflehog/Heuristic.h
    trufflehog/Heuristic.cpp
    trufflehog/EdgePenalties.h
    trufflehog/Crossings.h
    trufflehog/AStar.h
    trufflehog/AStar.cpp
    trufflehog/ReservationTable.h
    trufflehog/ReservationTable.cpp
    )
set(BCP_MAPF_SOURCE_FILES
    bcp/Main.cpp
    bcp/Includes.h
    bcp/Debug.h
    bcp/Coordinates.h
    bcp/Reader.h
    bcp/Reader.cpp
    bcp/ProblemData.h
    bcp/ProblemData.cpp
    bcp/VariableData.h
    bcp/VariableData.cpp
    bcp/Pricer_TruffleHog.h
    bcp/Pricer_TruffleHog.cpp
    bcp/ConstraintHandler_VertexConflicts.h
    bcp/ConstraintHandler_VertexConflicts.cpp
    bcp/ConstraintHandler_EdgeConflicts.h
    bcp/ConstraintHandler_EdgeConflicts.cpp
    bcp/Separator.h
    bcp/Separator_RectangleConflicts.h
    bcp/Separator_RectangleConflicts.cpp
    bcp/Separator_RectangleKnapsackConflicts.h
    bcp/Separator_RectangleKnapsackConflicts.cpp
    bcp/Separator_RectangleCliqueConflicts.h
    bcp/Separator_RectangleCliqueConflicts.cpp
    bcp/Separator_CorridorConflicts.h
    bcp/Separator_CorridorConflicts.cpp
    bcp/Separator_StepAsideConflicts.h
    bcp/Separator_StepAsideConflicts.cpp
    bcp/Separator_WaitDelayConflicts.h
    bcp/Separator_WaitDelayConflicts.cpp
    bcp/Separator_ExitEntryConflicts.h
    bcp/Separator_ExitEntryConflicts.cpp
    bcp/Separator_TwoEdgeConflicts.h
    bcp/Separator_TwoEdgeConflicts.cpp
    bcp/Separator_ThreeVertexConflicts.h
    bcp/Separator_ThreeVertexConflicts.cpp
    bcp/Separator_FiveEdgeConflicts.h
    bcp/Separator_FiveEdgeConflicts.cpp
    bcp/Separator_SixEdgeConflicts.h
    bcp/Separator_SixEdgeConflicts.cpp
    bcp/Separator_TripleEdgeConflicts.h
    bcp/Separator_TripleEdgeConflicts.cpp
    bcp/Separator_AgentWaitEdgeConflicts.h
    bcp/Separator_AgentWaitEdgeConflicts.cpp
    bcp/Separator_VertexFourEdgeConflicts.h
    bcp/Separator_VertexFourEdgeConflicts.cpp
    bcp/Separator_GoalConflicts.h
    bcp/Separator_GoalConflicts.cpp
    bcp/BranchingRule.h
    bcp/BranchingRule.cpp
    bcp/BranchingRule_Fractional.cpp
    bcp/Constraint_VertexBranching.h
    bcp/Constraint_VertexBranching.cpp
    bcp/Constraint_LengthBranching.h
    bcp/Constraint_LengthBranching.cpp
    bcp/Output.h
    bcp/Output.cpp
    )

# Create target.
add_executable(trufflehog ${TRUFFLEHOG_SOURCE_FILES} trufflehog/Main.cpp)
add_executable(bcp-mapf ${BCP_MAPF_SOURCE_FILES} ${TRUFFLEHOG_SOURCE_FILES})
target_include_directories(trufflehog PUBLIC ./ bcp/)
target_include_directories(bcp-mapf PUBLIC ./ bcp/)

# Include Gurobi.
find_path(GUROBI_INCLUDE_DIR
    NAMES gurobi_c.h
    HINTS "${GUROBI_DIR}/include" "${GUROBI_DIR}/*/include" "$ENV{GUROBI_DIR}/include" "$ENV{GUROBI_DIR}/*/include")
find_library(GUROBI_LIBRARY
    NAMES gurobi91 gurobi90 gurobi80 gurobi81 gurobi75 gurobi70
    HINTS "${GUROBI_DIR}/lib" "${GUROBI_DIR}/*/lib" "$ENV{GUROBI_DIR}/lib" "$ENV{GUROBI_DIR}/*/lib")
if (GUROBI_INCLUDE_DIR AND GUROBI_LIBRARY)
    set(LPS grb CACHE STRING "options for LP solver")
    target_include_directories(bcp-mapf PUBLIC ${GUROBI_INCLUDE_DIR})
    message("Using Gurobi as LP solver")
endif ()

# Include CPLEX.
if (NOT GUROBI_INCLUDE_DIR OR NOT GUROBI_LIBRARY)
    find_path(CPLEX_INCLUDE_DIR
        NAMES cplex.h
        HINTS "${CPLEX_DIR}/include/ilcplex" "$ENV{CPLEX_DIR}/include/ilcplex")
    find_library(CPLEX_LIBRARY
        NAMES cplex
        HINTS "${CPLEX_DIR}/lib/*/static_pic" "$ENV{CPLEX_DIR}/lib/*/static_pic")
    if (CPLEX_INCLUDE_DIR AND CPLEX_LIBRARY)
        set(LPS cpx CACHE STRING "options for LP solver")
        target_include_directories(bcp-mapf PUBLIC ${CPLEX_INCLUDE_DIR})
        message("Using CPLEX as LP solver")
    endif ()
endif ()

# Check for LP solver.
if (NOT LPS)
    message(FATAL_ERROR "Gurobi or CPLEX is required")
endif ()

# Include SCIP.
set(WITH_SCIPDEF on)
set(TPI none)
set(tpisources tpi/tpi_none.c)
set(THREAD_LIBRARIES "")
set(TPI_NONE on)
configure_file(scipoptsuite-7.0.2/scip/src/scip/config.h.in ${CMAKE_BINARY_DIR}/scip/config.h @ONLY)
target_include_directories(bcp-mapf PUBLIC ${CMAKE_BINARY_DIR})
if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -ldl")
endif ()
add_subdirectory(scipoptsuite-7.0.2/scip/ EXCLUDE_FROM_ALL)
target_include_directories(bcp-mapf PRIVATE          scipoptsuite-7.0.2/scip/src/
                                    SYSTEM INTERFACE scipoptsuite-7.0.2/scip/src/)

# Include fmt.
add_subdirectory(fmt EXCLUDE_FROM_ALL)
target_include_directories(bcp-mapf PUBLIC fmt/include/)

# Include cxxopts.
target_include_directories(bcp-mapf PUBLIC cxxopts/include/)

# Include robin-hood-hashing.
target_include_directories(bcp-mapf PUBLIC robin-hood-hashing/src/include/)

# Link to math library.
find_library(LIBM m)
if (NOT LIBM)
    set(LIBM "")
endif ()

# Link to libraries.
target_link_libraries(trufflehog fmt::fmt-header-only)
target_link_libraries(bcp-mapf fmt::fmt-header-only libscip ${LIBM})

# Set separator options.
target_compile_options(bcp-mapf PRIVATE -DUSE_WAITEDGE_CONFLICTS)
target_compile_options(bcp-mapf PRIVATE -DUSE_RECTANGLE_KNAPSACK_CONFLICTS)
#target_compile_options(bcp-mapf PRIVATE -DUSE_RECTANGLE_CLIQUE_CONFLICTS)
target_compile_options(bcp-mapf PRIVATE -DUSE_CORRIDOR_CONFLICTS)
target_compile_options(bcp-mapf PRIVATE -DUSE_LIFTED_CORRIDOR_CONFLICTS)
target_compile_options(bcp-mapf PRIVATE -DUSE_STEPASIDE_CONFLICTS)
target_compile_options(bcp-mapf PRIVATE -DUSE_WAITDELAY_CONFLICTS)
target_compile_options(bcp-mapf PRIVATE -DUSE_EXITENTRY_CONFLICTS)
target_compile_options(bcp-mapf PRIVATE -DUSE_TWOEDGE_CONFLICTS)
target_compile_options(bcp-mapf PRIVATE -DUSE_THREEVERTEX_CONFLICTS)
target_compile_options(bcp-mapf PRIVATE -DUSE_FIVEEDGE_CONFLICTS)
target_compile_options(bcp-mapf PRIVATE -DUSE_SIXEDGE_CONFLICTS)
target_compile_options(bcp-mapf PRIVATE -DUSE_TRIPLEEDGE_CONFLICTS)
target_compile_options(bcp-mapf PRIVATE -DUSE_AGENTWAITEDGE_CONFLICTS)
target_compile_options(bcp-mapf PRIVATE -DUSE_VERTEX_FOUREDGE_CONFLICTS)
target_compile_options(bcp-mapf PRIVATE -DUSE_GOAL_CONFLICTS)

# Set node selection options.
target_compile_options(bcp-mapf PRIVATE -DUSE_BEST_FIRST_NODE_SELECTION)
#target_compile_options(bcp-mapf PRIVATE -DUSE_DEPTH_FIRST_NODE_SELECTION)
#target_compile_options(bcp-mapf PRIVATE -DUSE_BEST_ESTIMATE_NODE_SELECTION)
#target_compile_options(bcp-mapf PRIVATE -DUSE_HYBRID_ESTIMATE_BOUND_NODE_SELECTION)
#target_compile_options(bcp-mapf PRIVATE -DUSE_RESTART_DEPTH_FIRST_NODE_SELECTION)

# Set warnings.
target_compile_options(bcp-mapf PRIVATE -Wall -Wextra -Wignored-qualifiers -Werror=return-type)

# Set flags.
target_compile_options(bcp-mapf PRIVATE -march=native -m64 -mfma)
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(bcp-mapf PRIVATE -DDEBUG -D_GLIBCXX_DEBUG)
    message("Compiled in debug mode")
elseif (CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    target_compile_options(bcp-mapf PRIVATE -Og -DNDEBUG -funroll-loops -fstrict-aliasing)
    message("Compiled in release with debug info mode")
else ()
    target_compile_options(bcp-mapf PRIVATE -O3 -DNDEBUG -funroll-loops -fstrict-aliasing)
    message("Compiled in release mode")
endif ()

# Use address sanitizer.
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer")

# Turn on link-time optimization for Linux.
if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -flto")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -flto")
endif ()
