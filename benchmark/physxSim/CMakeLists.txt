set(CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -Wl,--no-as-needed -O0 -mtune=native")

set(CMAKE_CXX_FLAGS_RELEASE
        "${CMAKE_CXX_FLAGS_RELEASE} -Wl,--no-as-needed -O3 -mtune=native")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# 落石场景
add_executable(physx666Benchmark 666Benchmark.cpp)
target_link_libraries(physx666Benchmark physxSim benchmark)
