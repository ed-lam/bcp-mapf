cmake_minimum_required(VERSION 3.2)
project(trufflehog)

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

# Include fmt.
include_directories(SYSTEM fmt/include)
add_subdirectory(fmt EXCLUDE_FROM_ALL)

# Set source files.
set(TRUFFLEHOG_SOURCE_FILES
        robin-hood-hashing/src/include/robin_hood.h
        Includes.h
        Debug.h
        Coordinates.h
        Map.h
        AgentsData.h
        Instance.h
        Instance.cpp
        PriorityQueue.h
        LabelPool.h
        LabelPool.cpp
        Heuristic.h
        Heuristic.cpp
        EdgePenalties.h
        Crossings.h
        AStar.h
        AStar.cpp
        ReservationTable.h
        ReservationTable.cpp

        main.cpp
        )

# Add executable.
add_executable(trufflehog ${TRUFFLEHOG_SOURCE_FILES})
target_link_libraries(trufflehog fmt::fmt-header-only)

# Enable resources for the families of cuts.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_GOAL_CONFLICTS")

# Set flags.
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -m64")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-ignored-attributes")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG -funroll-loops -fstrict-aliasing")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELEASE}")
if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -ldl")
endif ()

# Turn on link-time optimization for Linux.
if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -flto")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -flto")
endif ()
