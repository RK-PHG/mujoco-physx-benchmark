
set(CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -Wl,--no-as-needed -O0 -mtune=native -march=native")

set(CMAKE_CXX_FLAGS_RELEASE        "-Wl,--no-as-needed -O3 -mtune=native -march=native")


include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_executable(mujoco666Benchmark 666Benchmark.cpp)
target_link_libraries(mujoco666Benchmark mujocoSim benchmark)

add_executable(mujocoRollingBenchmark RollingBenchmark.cpp)
target_link_libraries(mujocoRollingBenchmark mujocoSim benchmark)

add_executable(mujocoSlidersBenchmark SlidersBenchmark.cpp)
target_link_libraries(mujocoSlidersBenchmark mujocoSim benchmark)

add_executable(mujocoBouncingBenchmark BouncingBenchmark.cpp)
target_link_libraries(mujocoBouncingBenchmark mujocoSim benchmark)