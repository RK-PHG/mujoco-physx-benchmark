
set(ex_yamlcpp_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../lib/yamlcpp")
set(yamlcpp_include_dir "${ex_yamlcpp_SOURCE_DIR}/include")
set(yamlcpp_lib_dir "${ex_yamlcpp_SOURCE_DIR}/lib")

find_library(yaml-cpp
        NO_DEFAULT_PATH
        NAMES       yaml-cpp
        PATHS       ${yamlcpp_lib_dir})

include_directories(${yamlcpp_include_dir})

set(raicommon_include_dir "${CMAKE_CURRENT_SOURCE_DIR}/../lib/raicommon/include")
include_directories(${raicommon_include_dir})

set(raicommon_lib_dir "${CMAKE_CURRENT_SOURCE_DIR}/../lib/raicommon/lib")

find_library(rai_common_graph_LIB
        NAMES       rai_common_graph
        PATHS       ${raicommon_lib_dir}
        NO_DEFAULT_PATH
        )

find_library(rai_common_logger_LIB
        NAMES       rai_common_logger
        PATHS       ${raicommon_lib_dir}
        NO_DEFAULT_PATH
        )

find_library(rai_common_timer_LIB
        NAMES       rai_common_timer
        PATHS       ${raicommon_lib_dir}
        NO_DEFAULT_PATH
        )

set(benchmarkHeaders
        "include/666Benchmark.hpp"
        "include/AnymalEnergyBenchmark.hpp"
        "include/AnymalMomentumBenchmark.hpp"
        "include/AnymalPDBenchmark.hpp"
        "include/AtlasContactBenchmark.hpp"
        "include/BenchmarkTest.hpp"
        "include/BouncingBenchmark.hpp"
        "include/Elastic666Benchmark.hpp"
        "include/KaplaBenchmark.hpp"
        "include/RollingBenchmark.hpp"
        )

 
include_directories("include")

add_definitions(-DBENCHMARKROOTPATH="${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DBENCHMARKYAMLPATH="${CMAKE_CURRENT_SOURCE_DIR}/res/yaml")

add_library(benchmark INTERFACE)

add_dependencies(benchmark
        Boost
        )

target_include_directories(benchmark
        INTERFACE
        "include"
        ${yamlcpp_include_dir}
        )

link_directories(/usr/lib/x86_64-linux-gnu)        
target_link_libraries(benchmark
        INTERFACE
        ${yaml-cpp}
        boost_filesystem
        ${Boost_LIBRARIES}
        ${rai_common_graph_LIB}
        ${rai_common_logger_LIB}
        ${rai_common_timer_LIB}
        )

file(COPY res/shell/anymal-pd.sh
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(COPY res/shell/rolling.sh
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(COPY res/shell/kapla.sh
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(COPY res/shell/666.sh
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(COPY res/shell/bouncing.sh
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(COPY res/shell/666-elastic.sh
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(COPY res/shell/anymal-momentum.sh
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(COPY res/shell/anymal-energy.sh
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(COPY res/shell/atlas-contact.sh
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(COPY res/shell/selectsim.sh
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

configure_file(sim.sh.in ${CMAKE_CURRENT_BINARY_DIR}/sim.sh @ONLY)

message(STATUS "Build benchmark tests:")

if(TARGET mujocoSim)
    add_subdirectory(mujocoSim)
else()
    message(STATUS "  Cannot find mujocoSim. Stop build mujocoSim benchmark tests")
endif()

if(TARGET physxSim)
    add_subdirectory(physxSim)
else()
    message(STATUS " Cannot find physxSim. Stop build physxSim benchmark tests")
endif()
