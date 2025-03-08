cmake_minimum_required(VERSION 3.5)
project(mujoco-issac-benchmark)

message("")
message("===========================================================================")
message("   _____ ______  _______  _______   __________  ____  ______    ____  __ __")
message("  / ___//  _/  |/  / __ )/ ____/ | / / ____/ / / /  |/  /   |  / __ \\/ //_/")
message("  \\__ \\ / // /|_/ / __  / __/ /  |/ / /   / /_/ / /|_/ / /| | / /_/ / ,<   ")
message(" ___/ // // /  / / /_/ / /___/ /|  / /___/ __  / /  / / ___ |/ _, _/ /| |  ")
message("/____/___/_/  /_/_____/_____/_/ |_/\\____/_/ /_/_/  /_/_/  |_/_/ |_/_/ |_|  ")
message("                                                                           ")
message("===========================================================================")
message("")

add_definitions(-std=c++17)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DRAIDEBUG)
else ()
    remove_definitions(-DRAIDEBUG)
endif()

option(MUJOCO "benchmark MUJOCO" ON)
option(PHYSX "benchmark PHYSX" OFF)

option(BUILD_BENCHMARK "build benchmark tests" ON)
message(STATUS "Benchmark the following physics engines:")

if(MUJOCO)
    message(STATUS "  MUJOCO")
endif(MUJOCO)

if(PHYSX)
    message(STATUS "  PHYSX")
endif(PHYSX)

find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

find_package(Boost COMPONENTS system filesystem program_options REQUIRED timer serialization thread regex)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

add_definitions(-DROOTPATH="${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DRESOURCEPATH="${CMAKE_CURRENT_SOURCE_DIR}/res")
add_definitions(-DDATAPATH="${CMAKE_CURRENT_SOURCE_DIR}/data")

include_directories(sim)
add_subdirectory(sim)

if(BUILD_BENCHMARK)
    add_subdirectory(benchmark)
endif(BUILD_BENCHMARK)

add_custom_target(clean_all
        COMMAND rm -rf *
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
