message(STATUS "  mujocoSim for MuJoCo")

set(ex_mujoco_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../lib/mujoco")
set(mujoco_include_dir "${ex_mujoco_SOURCE_DIR}/include/")
set(mujoco_lib_dir "${ex_mujoco_SOURCE_DIR}/lib/")

set(ex_raigraphics_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../lib/raigraphics")
set(raigraphics_include_dir "${ex_raigraphics_SOURCE_DIR}/include/")
set(raigraphics_lib_dir "${ex_raigraphics_SOURCE_DIR}/lib/")

find_library(mujoco_LIB
        NAMES       mujoco
        PATHS       ${mujoco_lib_dir}
        NO_DEFAULT_PATH
        )
 
if(NOT TARGET commonSim)
    message(WARNING "Cannot find dependency. Stop build mujocoSim")
    return()
endif()

set(CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -Wl,--no-as-needed -O0 -mtune=native -march=native")

set(CMAKE_CXX_FLAGS_RELEASE
        "-Wl,--no-as-needed -O3 -mtune=native -march=native")

include_directories(src)

set(mjcSimSrc
        src/MjcWorld.cpp
        src/MjcWorld.hpp
        src/MjcSim.cpp
        src/MjcSim.hpp
        src/UserHandle.hpp
        src/object/MjcSingleBodyObject.cpp
        src/object/MjcSingleBodyObject.hpp
        src/object/MjcSphere.cpp
        src/object/MjcSphere.hpp
        src/object/MjcBox.cpp
        src/object/MjcBox.hpp
        src/object/MjcCapsule.cpp
        src/object/MjcCapsule.hpp
        src/object/MjcCheckerBoard.cpp
        src/object/MjcCheckerBoard.hpp
        src/object/MjcCylinder.cpp
        src/object/MjcCylinder.hpp)

add_library(mujocoSim
        STATIC
        ${mjcSimSrc})

add_dependencies(mujocoSim
        commonSim
        )

target_include_directories(mujocoSim
        PUBLIC
        src
        ${mujoco_include_dir}
        ${raigraphics_include_dir}
        )

find_library(RAIGRAPHICS_LIB
        NAMES       raigraphics
        PATHS       ${raigraphics_lib_dir}
        )

if(NOT RAIGRAPHICS_LIB)
        message(FATAL_ERROR "Could not find raigraphics library.")
endif()        

target_link_libraries(mujocoSim
        ${RAIGRAPHICS_LIB}
        ${mujoco_LIB}
        ${Boost_FILESYSTEM_LIBRARY}
        ${Boost_SYSTEM_LIBRARY}
        commonSim
        OpenGL GLEW SDL2 SDL2_ttf assimp SOIL pthread ncurses
        )
