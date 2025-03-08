message(STATUS "  dartSim for DART")

########################################################################################################################
# dependencies
set(ex_dart_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../lib/dart")
set(dart_include_dir "${ex_dart_SOURCE_DIR}/include/")
set(dart_lib_dir "${ex_dart_SOURCE_DIR}/lib/")

#find_library(DART
#        NAMES       libdart.so
#        PATHS       ${dart_lib_dir}
#        NO_DEFAULT_PATH
#        )
#
#find_library(DART_IO
#        NAMES     libdart-io.so
#        PATHS     ${dart_lib_dir}
#        NO_DEFAULT_PATH
#        )
#
#find_library(DART_IO_URDF
#        NAMES     libdart-io-urdf.so
#        PATHS     ${dart_lib_dir}
#        NO_DEFAULT_PATH
#        )
#
#find_library(DART_COLLISION_BULLET
#        NAMES     libdart-collision-bullet.so
#        PATHS     ${dart_lib_dir}
#        NO_DEFAULT_PATH
#        )
#
#find_library(DART_COLLISION_ODE
#        NAMES     libdart-collision-ode.so
#        PATHS     ${dart_lib_dir}
#        NO_DEFAULT_PATH
#        )

#find_package(DART
#        CONFIG
#        PATHS           "${dart_lib_dir}"
#        COMPONENTS
#        io
#        io-urdf
#        collision-bullet
#        collision-ode
#        NO_DEFAULT_PATH
#        )


# raigraphics
set(ex_raigraphics_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../lib/raigraphics")
set(raigraphics_include_dir "${ex_raigraphics_SOURCE_DIR}/include/")
set(raigraphics_lib_dir "${ex_raigraphics_SOURCE_DIR}/lib/")

set(ex_bullet_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../lib/bullet")
set(bullet_include_dir "${ex_bullet_SOURCE_DIR}/include/")
set(bullet_lib_dir "${ex_bullet_SOURCE_DIR}/lib/")

if(NOT TARGET commonSim)
    message(WARNING "----------------Cannot find dependency. Stop build dartSim-----------------------------------------------------------------------------------------------------------------------")
    return()
endif()


########################################################################################################################
# flags
set(CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -Wl,--no-as-needed -O0 -mtune=native")

set(CMAKE_CXX_FLAGS_RELEASE
        "${CMAKE_CXX_FLAGS_RELEASE} -Wl,--no-as-needed -O3 -mtune=native")


########################################################################################################################
# source
set(DartSim_SRCS
        src/DartWorld.cpp
        src/DartWorld.hpp
        src/DartSim.cpp
        src/DartSim.hpp
        src/UserHandle.hpp
        src/object/DartSingleBodyObject.cpp
        src/object/DartSingleBodyObject.hpp
        src/object/DartSphere.cpp
        src/object/DartSphere.hpp
        src/object/DartBox.cpp
        src/object/DartBox.hpp
        src/object/DartCapsule.cpp
        src/object/DartCapsule.hpp
        src/object/DartCheckerBoard.cpp
        src/object/DartCheckerBoard.hpp
        src/object/DartObject.cpp
        src/object/DartObject.hpp
        src/object/DartCylinder.cpp
        src/object/DartCylinder.hpp
        src/object/DartArticulatedSystem.cpp
        src/object/DartArticulatedSystem.hpp
        )


########################################################################################################################
# library
add_library(dartSim
        STATIC
        ${DartSim_SRCS}
        )

add_dependencies(dartSim
        commonSim
        )

find_library(RAIGRAPHICS_LIB
        NAMES       raigraphics
        PATHS       ${raigraphics_lib_dir}
        )

target_include_directories(dartSim
        PUBLIC
        "src"
        ${dart_include_dir}
        ${bullet_include_dir}
        ${raigraphics_include_dir}
        )

find_package(console_bridge REQUIRED)
target_link_libraries(dartSim
        "commonSim"
        ${RAIGRAPHICS_LIB}
        ${dart_lib_dir}/libdart.so
        ${dart_lib_dir}/libdart-io.so
        ${dart_lib_dir}/libdart-collision-bullet.so
        ${dart_lib_dir}/libdart-collision-ode.so
        ${dart_lib_dir}/libdart-external-odelcpsolver.so.7.0
        ${dart_lib_dir}/libdart-io-urdf.so
        ${bullet_lib_dir}/libBulletInverseDynamics.a
        ${bullet_lib_dir}/libBulletInverseDynamicsUtils.a
        ${bullet_lib_dir}/libLinearMath.a
        lz4
        OpenGL GLEW SDL2 SDL2_ttf assimp SOIL pthread ncurses console_bridge::console_bridge ccd fcl
        )


########################################################################################################################
# sub-directories
