########################################################################################################################
# flags
set(CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -Wl,--no-as-needed -O0 -mtune=native -march=native")

set(CMAKE_CXX_FLAGS_RELEASE
        "-Wl,--no-as-needed -O3 -mtune=native -march=native")


########################################################################################################################
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

########################################################################################################################
# executable

#add_executable(mujocoAnymalPDBenchmark AnymalPDBenchmark.cpp)
#target_link_libraries(mujocoAnymalPDBenchmark mujocoSim benchmark)

add_executable(mujocoAnymalMomentumBenchmark AnymalMomentumBenchmark.cpp)
target_link_libraries(mujocoAnymalMomentumBenchmark mujocoSim benchmark)

# add_executable(mujocoAnymalEnergyBenchmark AnymalEnergyBenchmark.cpp)
# target_link_libraries(mujocoAnymalEnergyBenchmark mujocoSim benchmark)

#add_executable(mujocoRollingBenchmark RollingBenchmark.cpp)
#link_directories(/usr/lib/x86_64-linux-gnu)
#target_link_libraries(mujocoRollingBenchmark mujocoSim benchmark boost_filesystem boost_system)

# add_executable(mujocoKaplaBenchmark KaplaBenchmark.cpp)
# target_link_libraries(mujocoKaplaBenchmark mujocoSim benchmark)

#add_executable(mujoco666Benchmark 666Benchmark.cpp)
#target_link_libraries(mujoco666Benchmark mujocoSim benchmark)

# add_executable(mujocoBouncingBenchmark BouncingBenchmark.cpp)
# target_link_libraries(mujocoBouncingBenchmark mujocoSim benchmark)

# add_executable(mujocoAtlasContactBenchmark AtlasContactBenchmark.cpp)
# target_link_libraries(mujocoAtlasContactBenchmark mujocoSim benchmark)

# add_executable(AtlasDemo AtlasDemo.cpp)
# target_link_libraries(AtlasDemo mujocoSim benchmark)