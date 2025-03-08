########################################################################################################################
# flags
set(CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -Wl,--no-as-needed -O0 -mtune=native")

set(CMAKE_CXX_FLAGS_RELEASE
        "${CMAKE_CXX_FLAGS_RELEASE} -Wl,--no-as-needed -O3 -mtune=native")


########################################################################################################################
include_directories(${CMAKE_CURRENT_SOURCE_DIR})


########################################################################################################################
# executable

add_executable(physx666Benchmark 666Benchmark.cpp)
target_link_libraries(physx666Benchmark physxSim benchmark)

#add_executable(physx666ElasticBenchmark Elastic666Benchmark.cpp)
#target_link_libraries(physx666ElasticBenchmark physxSim benchmark)
#
#add_executable(physxBouncingBenchmark BouncingBenchmark.cpp)
#target_link_libraries(physxBouncingBenchmark physxSim benchmark)
#
#add_executable(physxKaplaBenchmark KaplaBenchmark.cpp)
#target_link_libraries(physxKaplaBenchmark physxSim benchmark)
#
#add_executable(physxRollingBenchmark RollingBenchmark.cpp)
#target_link_libraries(physxRollingBenchmark physxSim benchmark)
#
#add_executable(physxAnymalEnergyBenchmark AnymalEnergyBenchmark.cpp)
#target_link_libraries(physxAnymalEnergyBenchmark physxSim benchmark)
#
add_executable(physxAnymalMomentumBenchmark AnymalMomentumBenchmark.cpp)
target_link_libraries(physxAnymalMomentumBenchmark physxSim benchmark)
#
#add_executable(physxssssAnymalPDBenchmark AnymalPDBenchmark.cpp)
#target_link_libraries(physxAnymalPDBenchmark physxSim benchmark)
#
#add_executable(physxAtlasContactBenchmark AtlasContactBenchmark.cpp)
#target_link_libraries(physxAtlasContactBenchmark physxSim benchmark)