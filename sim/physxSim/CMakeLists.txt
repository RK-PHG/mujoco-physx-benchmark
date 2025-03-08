message(STATUS "  physXSim for physX")

set(ex_physx_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../lib/physx")
set(physx_include_dir "${ex_physx_SOURCE_DIR}/include/")
set(physx_lib_dir "${ex_physx_SOURCE_DIR}/lib/")

set(ex_raigraphics_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../lib/raigraphics")
set(raigraphics_include_dir "${ex_raigraphics_SOURCE_DIR}/include/")
set(raigraphics_lib_dir "${ex_raigraphics_SOURCE_DIR}/lib/")

#physx
find_library(physx_LIB
        NAMES       PhysXGpu_64
        PATHS       ${physx_lib_dir}
        NO_DEFAULT_PATH
        )

find_library(PhysX_static_64_LIB
        NAMES       PhysX_static_64
        PATHS       ${physx_lib_dir}
        NO_DEFAULT_PATH
        )

find_library(PhysXExtensions_static_64_LIB
        NAMES       PhysXExtensions_static_64
        PATHS       ${physx_lib_dir}
        NO_DEFAULT_PATH
        )

find_library(PhysXPvdSDK_static_64_LIB
        NAMES       PhysXPvdSDK_static_64
        PATHS       ${physx_lib_dir}
        NO_DEFAULT_PATH
        )

find_library(PhysXCommon_static_64_LIB
        NAMES       PhysXCommon_static_64
        PATHS       ${physx_lib_dir}
        NO_DEFAULT_PATH
        )

find_library(PhysXFoundation_static_64_LIB
        NAMES       PhysXFoundation_static_64
        PATHS       ${physx_lib_dir}
        NO_DEFAULT_PATH
        )

find_library(PVDRuntime_64_LIB
        NAMES       PVDRuntime_64
        PATHS       ${physx_lib_dir}
        NO_DEFAULT_PATH
        )

find_library(PhysXPvdSDK_static_64_LIB
        NAMES       PhysXPvdSDK_static_64
        PATHS       ${physx_lib_dir}
        NO_DEFAULT_PATH
        )

if(NOT TARGET commonSim)
    message(WARNING "Cannot find dependency. Stop build physxSim")
    return()
endif()

set(CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -Wl,--no-as-needed -O0 -mtune=native -march=native")

set(CMAKE_CXX_FLAGS_RELEASE
        "-Wl,--no-as-needed -O3 -mtune=native -march=native")

include_directories(src)

set(pyXSimSrc
        src/PyXWorld.cpp
        src/PyXWorld.hpp
        src/PyXSim.cpp
        src/PyXSim.hpp
        src/UserHandle.hpp
        src/object/PyXArticulatedSystem.hpp
        src/object/PyXArticulatedSystem.cpp
        src/object/PyXSingleBodyObject.hpp
        src/object/PyXSingleBodyObject.cpp
        src/object/PyXSphere.cpp
        src/object/PyXSphere.hpp
        src/object/PyXBox.cpp
        src/object/PyXBox.hpp
        src/object/PyXCapsule.cpp
        src/object/PyXCapsule.hpp
        src/object/PyXCheckerBoard.cpp
        src/object/PyXCheckerBoard.hpp
        src/object/PyXCylinder.cpp
        src/object/PyXCylinder.hpp
        )

add_library(physxSim
        STATIC
        ${pyXSimSrc})

add_dependencies(physxSim
        commonSim
        )

target_include_directories(physxSim
        PUBLIC
        src
        ${physx_include_dir}
        ${raigraphics_include_dir}
        )

find_library(RAIGRAPHICS_LIB
        NAMES       raigraphics
        PATHS       ${raigraphics_lib_dir}
        )

if(NOT RAIGRAPHICS_LIB)
    message(FATAL_ERROR "Could not find raigraphics library.")
endif()

target_link_libraries(physxSim
        ${RAIGRAPHICS_LIB}
        ${physx_LIB}
        ${PhysX_static_64_LIB}
        ${PhysXExtensions_static_64_LIB}
        ${PhysXPvdSDK_static_64_LIB}
        ${PhysXCommon_static_64_LIB}
        ${PhysXFoundation_static_64_LIB}
        ${PVDRuntime_64_LIB}
        ${PhysXPvdSDK_static_64_LIB}
        commonSim
        OpenGL GLEW SDL2 SDL2_ttf assimp SOIL pthread ncurses cuda GL glut GLU tinyxml2 dl
        )
