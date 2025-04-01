cmake_minimum_required(VERSION 3.21)
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
set(BCP_MAPF_SOURCE_FILES
    bcp/branching/branching_rule.cpp
    bcp/branching/fractional.cpp
    bcp/branching/length_branching_constraint.cpp
    bcp/branching/pseudosolution.cpp
    bcp/branching/vertex_branching_constraint.cpp
    bcp/constraints/agent_wait_edge.cpp
    bcp/constraints/corridor.cpp
    bcp/constraints/edgetime.cpp
    bcp/constraints/exit_entry.cpp
    bcp/constraints/five_edge.cpp
    bcp/constraints/four_edge.cpp
    bcp/constraints/nodetime.cpp
    bcp/constraints/preprocessing.cpp
    bcp/constraints/rectangle_knapsack.cpp
    bcp/constraints/rectangle.cpp
    bcp/constraints/six_edge.cpp
    bcp/constraints/step_aside.cpp
    bcp/constraints/target.cpp
    bcp/constraints/three_vertex.cpp
    bcp/constraints/two_edge.cpp
    bcp/constraints/two_vertex.cpp
    bcp/constraints/vertex_four_edge.cpp
    bcp/constraints/wait_delay.cpp
    bcp/heuristics/prioritized_planning.cpp
    bcp/main.cpp
    bcp/pricing/astar.cpp
    bcp/pricing/distance_heuristic.cpp
    bcp/pricing/pricer.cpp
    bcp/problem/instance.cpp
    bcp/problem/map.cpp
    bcp/problem/output.cpp
    bcp/problem/problem.cpp
    bcp/problem/reader.cpp
    bcp/problem/variable_data.cpp
    bcp/types/memory_pool.cpp
)

# Create target.
add_executable(bcp-mapf ${BCP_MAPF_SOURCE_FILES})
target_include_directories(bcp-mapf PUBLIC ./ bcp/)

# Include SCIP binaries.
find_path(SCIP_INCLUDE_DIR
          NAMES scip/scip.h
          HINTS "${SCIP_DIR}/scip/src" "$ENV{SCIP_DIR}/scip/src")
find_path(SCIP_CONFIG_DIR
          NAMES scip/config.h
          HINTS "${SCIP_DIR}/build/scip" "$ENV{SCIP_DIR}/build/scip")
find_library(SCIP_LIBRARY
             NAMES scip
             HINTS "${SCIP_DIR}/build/lib" "$ENV{SCIP_DIR}/build/lib")
if (SCIP_INCLUDE_DIR AND SCIP_CONFIG_DIR AND SCIP_LIBRARY)

    target_include_directories(bcp-mapf PUBLIC ${SCIP_INCLUDE_DIR} ${SCIP_CONFIG_DIR})
    message("Using SCIP binary at ${SCIP_LIBRARY}")

else ()

    # Include Gurobi.
    find_path(GUROBI_INCLUDE_DIRS
        NAMES gurobi_c.h
        HINTS "${GUROBI_DIR}/include" "${GUROBI_DIR}/*/include" "$ENV{GUROBI_DIR}/include" "$ENV{GUROBI_DIR}/*/include" "$ENV{GUROBI_HOME}/include")
    find_library(GUROBI_LIBRARY
        NAMES gurobi120 gurobi110 gurobi100 gurobi95 gurobi91 gurobi90 gurobi81 gurobi80 gurobi75 gurobi70
        HINTS "${GUROBI_DIR}/lib" "${GUROBI_DIR}/*/lib" "$ENV{GUROBI_DIR}/lib" "$ENV{GUROBI_DIR}/*/lib" "$ENV{GUROBI_HOME}/lib")
    if (GUROBI_INCLUDE_DIRS AND GUROBI_LIBRARY)
        set(LPS grb CACHE STRING "options for LP solver")
        target_include_directories(bcp-mapf PUBLIC ${GUROBI_INCLUDE_DIRS})
        message("Using Gurobi at ${GUROBI_LIBRARY} as LP solver")
    endif ()

    # Include CPLEX.
    if (NOT GUROBI_INCLUDE_DIRS OR NOT GUROBI_LIBRARY)
        find_path(CPLEX_INCLUDE_DIR
            NAMES cplex.h
            HINTS "${CPLEX_DIR}/include/ilcplex" "$ENV{CPLEX_DIR}/include/ilcplex")
        find_library(CPLEX_LIBRARY
            NAMES cplex
            HINTS "${CPLEX_DIR}/lib/*/static_pic" "$ENV{CPLEX_DIR}/lib/*/static_pic")
        if (CPLEX_INCLUDE_DIR AND CPLEX_LIBRARY)
            set(LPS cpx CACHE STRING "options for LP solver")
            target_include_directories(bcp-mapf PUBLIC ${CPLEX_INCLUDE_DIR})
            message("Using CPLEX at ${CPLEX_LIBRARY} as LP solver")
        endif ()
    endif ()

    # Include SoPlex.
    if ((NOT GUROBI_INCLUDE_DIRS OR NOT GUROBI_LIBRARY) AND (NOT CPLEX_INCLUDE_DIR OR NOT CPLEX_LIBRARY))
        set(LPS spx CACHE STRING "options for LP solver")
        set(CONF_INCLUDE_DIRS ../soplex/src/)
        configure_file(scipoptsuite-9.2.0/soplex/soplex-config.cmake.in
                       "${CMAKE_BINARY_DIR}/soplex/soplex-config.cmake"
                       @ONLY)
        set(SOPLEX_DIR ${CMAKE_BINARY_DIR}/soplex)
        add_subdirectory(scipoptsuite-9.2.0/soplex/ EXCLUDE_FROM_ALL)
        message("Using SoPlex as LP solver ${SOPLEX_INCLUDE_DIRS}")
    endif ()

    # Include SCIP.
    set(WITH_SCIPDEF on)
    set(TPI none)
    set(tpisources tpi/tpi_none.c)
    set(THREAD_LIBRARIES "")
    set(TPI_NONE on)
    set(GMP off)
    set(IPOPT off)
    set(PAPILO off)
    set(ZIMPL off)
    configure_file(scipoptsuite-9.2.0/scip/src/scip/config.h.in "${CMAKE_BINARY_DIR}/scip/config.h" @ONLY)
    target_include_directories(bcp-mapf PUBLIC ${CMAKE_BINARY_DIR})
    if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -ldl")
    endif ()
    add_subdirectory(scipoptsuite-9.2.0/scip/ EXCLUDE_FROM_ALL)
    target_include_directories(bcp-mapf PRIVATE          scipoptsuite-9.2.0/scip/src/
                                        SYSTEM INTERFACE scipoptsuite-9.2.0/scip/src/)
    set(SCIP_LIBRARY libscip)

endif ()

# Find boost.
# find_path(BOOST_INCLUDE_DIR
#     NAMES boost/heap/pairing_heap.hpp
#     HINTS "${BOOST_DIR}" "$ENV{BOOST_DIR}")
# if (NOT BOOST_INCLUDE_DIR)

    # Download boost if not found.
    set(BOOST_VERSION 1.87.0)
    message(STATUS "Downloading Boost libraries")
    include(FetchContent)
    # Set(FETCHCONTENT_QUIET FALSE)
    FetchContent_Declare(boost
        URL https://github.com/boostorg/boost/releases/download/boost-${BOOST_VERSION}/boost-${BOOST_VERSION}-b2-nodocs.tar.gz
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
    FetchContent_MakeAvailable(boost)
    message("Downloaded Boost to ${boost_SOURCE_DIR}")
    SET(BOOST_INCLUDE_DIR "${boost_SOURCE_DIR}")

    # Include boost again.
    find_path(BOOST_INCLUDE_DIR
        NAMES boost/heap/d_ary_heap.hpp
        HINTS "${BOOST_DIR}" "$ENV{BOOST_DIR}" "boost_1_87_0"
        REQUIRED)

# endif ()
message("Found Boost libraries at ${BOOST_INCLUDE_DIR}")

# Include fmt.
add_subdirectory(fmt EXCLUDE_FROM_ALL)
target_include_directories(bcp-mapf PUBLIC fmt/include/)

# Include cxxopts.
target_include_directories(bcp-mapf PUBLIC cxxopts/include/)

# Include robin-hood-hashing.
target_include_directories(bcp-mapf PUBLIC robin-hood-hashing/src/include/)

# Include Boost.
target_include_directories(bcp-mapf PUBLIC ${BOOST_INCLUDE_DIR})

# Link to math library.
find_library(LIBM m)
if (NOT LIBM)
    set(LIBM "")
endif ()

# Link to libraries.
target_link_libraries(bcp-mapf fmt::fmt-header-only ${SCIP_LIBRARY} ${LIBM})

# Set pricer options.
target_compile_options(bcp-mapf PRIVATE -DUSE_RESERVATION_TABLE)
target_compile_options(bcp-mapf PRIVATE -DUSE_ASTAR_SOLUTION_CACHING)

# Set separator options.
target_compile_options(bcp-mapf PRIVATE -DUSE_WAITEDGE_CONFLICTS)
target_compile_options(bcp-mapf PRIVATE -DUSE_RECTANGLE_KNAPSACK_CONFLICTS)
target_compile_options(bcp-mapf PRIVATE -DUSE_CORRIDOR_CONFLICTS)
target_compile_options(bcp-mapf PRIVATE -DUSE_WAITCORRIDOR_CONFLICTS)
#target_compile_options(bcp-mapf PRIVATE -DUSE_STEPASIDE_CONFLICTS)
target_compile_options(bcp-mapf PRIVATE -DUSE_WAITDELAY_CONFLICTS)
target_compile_options(bcp-mapf PRIVATE -DUSE_EXITENTRY_CONFLICTS)
target_compile_options(bcp-mapf PRIVATE -DUSE_TWOEDGE_CONFLICTS)
target_compile_options(bcp-mapf PRIVATE -DUSE_WAITTWOEDGE_CONFLICTS)
target_compile_options(bcp-mapf PRIVATE -DUSE_AGENTWAITEDGE_CONFLICTS)
#target_compile_options(bcp-mapf PRIVATE -DUSE_TWOVERTEX_CONFLICTS)
#target_compile_options(bcp-mapf PRIVATE -DUSE_THREEVERTEX_CONFLICTS)
#target_compile_options(bcp-mapf PRIVATE -DUSE_FOUREDGE_CONFLICTS)
#target_compile_options(bcp-mapf PRIVATE -DUSE_FIVEEDGE_CONFLICTS)
#target_compile_options(bcp-mapf PRIVATE -DUSE_SIXEDGE_CONFLICTS)
#target_compile_options(bcp-mapf PRIVATE -DUSE_VERTEX_FOUREDGE_CONFLICTS)
target_compile_options(bcp-mapf PRIVATE -DUSE_GOAL_CONFLICTS)

# Set primal heuristics options.
# target_compile_options(bcp-mapf PRIVATE -DUSE_PRIORITIZED_PLANNING_PRIMAL_HEURISTIC)

# Set node selection options.
target_compile_options(bcp-mapf PRIVATE -DUSE_BEST_FIRST_NODE_SELECTION)
#target_compile_options(bcp-mapf PRIVATE -DUSE_DEPTH_FIRST_NODE_SELECTION)
#target_compile_options(bcp-mapf PRIVATE -DUSE_BEST_ESTIMATE_NODE_SELECTION)
#target_compile_options(bcp-mapf PRIVATE -DUSE_HYBRID_ESTIMATE_BOUND_NODE_SELECTION)
#target_compile_options(bcp-mapf PRIVATE -DUSE_RESTART_DEPTH_FIRST_NODE_SELECTION)

# Set warnings.
target_compile_options(bcp-mapf PRIVATE -Wall -Wextra -Wignored-qualifiers -Werror=return-type -Wno-sign-compare)

# Set flags.
check_cxx_compiler_flag("-march=native" MARCH_NATIVE)
if (MARCH_NATIVE)
    target_compile_options(bcp-mapf PRIVATE -march=native)
endif ()
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
