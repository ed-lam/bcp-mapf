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

# Include Gurobi.
find_path(GUROBI_INCLUDE_DIR
          NAMES gurobi_c.h
          HINTS "${GUROBI_DIR}/include" 
                "${GUROBI_DIR}/*/include" 
                "$ENV{GUROBI_DIR}/include" 
                "$ENV{GUROBI_DIR}/*/include")
find_library(GUROBI_LIBRARY
             NAMES gurobi91 gurobi90 gurobi80 gurobi81 gurobi75 gurobi70
             HINTS "${GUROBI_DIR}/lib"
                   "${GUROBI_DIR}/*/lib" 
                   "$ENV{GUROBI_DIR}/lib"
                   "$ENV{GUROBI_DIR}/*/lib")
if (GUROBI_INCLUDE_DIR AND GUROBI_LIBRARY)
    set(LPS grb CACHE STRING "options for LP solver")
    include_directories(${GUROBI_INCLUDE_DIR})
    message("Using Gurobi as LP solver")
endif ()

# Include CPLEX.
if (NOT GUROBI_INCLUDE_DIR OR NOT GUROBI_LIBRARY)
    find_path(CPLEX_INCLUDE_DIR
              NAMES cplex.h
              HINTS "${CPLEX_DIR}/include/ilcplex" 
                    "$ENV{CPLEX_DIR}/include/ilcplex")
    find_library(CPLEX_LIBRARY
                 NAMES cplex
                 HINTS "${CPLEX_DIR}/lib/*/static_pic" 
                       "$ENV{CPLEX_DIR}/lib/*/static_pic")
    if (CPLEX_INCLUDE_DIR AND CPLEX_LIBRARY)
        set(LPS cpx CACHE STRING "options for LP solver")
        include_directories(${CPLEX_INCLUDE_DIR})
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
configure_file(scipoptsuite-7.0.1/scip/src/scip/config.h.in ${CMAKE_BINARY_DIR}/scip/config.h @ONLY)
include_directories(${CMAKE_BINARY_DIR})
if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -ldl")
endif ()
add_subdirectory(scipoptsuite-7.0.1/scip/ EXCLUDE_FROM_ALL)
include_directories(SYSTEM scipoptsuite-7.0.1/scip/src/)

# Link to math library.
find_library(LIBM m)
if (NOT LIBM)
    set(LIBM "")
endif ()

# Include fmt.
include_directories(SYSTEM fmt/include)
add_subdirectory(fmt EXCLUDE_FROM_ALL)

# Include cxxopts.
include_directories(SYSTEM cxxopts/include)

# Include Truffle Hog.
include_directories(./)
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

# Set source files.
set(MAPF_SOURCE_FILES
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
        bcp/Separator_CorridorConflicts.h
        bcp/Separator_CorridorConflicts.cpp
        bcp/Separator_WaitDelayConflicts.h
        bcp/Separator_WaitDelayConflicts.cpp
        bcp/Separator_ExitEntryConflicts.h
        bcp/Separator_ExitEntryConflicts.cpp
        bcp/Separator_TwoEdgeConflicts.h
        bcp/Separator_TwoEdgeConflicts.cpp
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
include_directories(bcp/)

# Add executable.
add_executable(bcp-mapf ${MAPF_SOURCE_FILES} ${TRUFFLEHOG_SOURCE_FILES})
target_link_libraries(bcp-mapf fmt::fmt-header-only libscip ${LIBM})

# Enable the families of cuts.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_WAITEDGE_CONFLICTS")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_RECTANGLE_KNAPSACK_CONFLICTS")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_CORRIDOR_CONFLICTS")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_WAITDELAY_CONFLICTS")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_EXITENTRY_CONFLICTS")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_TWOEDGE_CONFLICTS")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_GOAL_CONFLICTS")

# Set node selection rule.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_BEST_FIRST_NODE_SELECTION")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_DEPTH_FIRST_NODE_SELECTION")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_BEST_ESTIMATE_NODE_SELECTION")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_HYBRID_ESTIMATE_BOUND_NODE_SELECTION")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_RESTART_DEPTH_FIRST_NODE_SELECTION")

# Set flags.
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -m64")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-ignored-attributes")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG -funroll-loops -fstrict-aliasing")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELEASE} -g")

# Turn on link-time optimization for Linux.
if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -flto")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -flto")
endif ()

# Copy instances into build directory.
#add_custom_command(TARGET bcp-mapf PRE_BUILD
#        COMMAND ${CMAKE_COMMAND} -E copy_directory
#        ${CMAKE_SOURCE_DIR}/2018_instances $<TARGET_FILE_DIR:bcp-mapf>/2018_instances)
#add_custom_command(TARGET bcp-mapf PRE_BUILD
#        COMMAND ${CMAKE_COMMAND} -E copy_directory
#        ${CMAKE_SOURCE_DIR}/2019_instances $<TARGET_FILE_DIR:bcp-mapf>/2019_instances)
