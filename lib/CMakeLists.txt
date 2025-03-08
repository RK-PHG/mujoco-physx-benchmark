##[[
#
#This CMakeLists.txt is for installing dependencies.
#
#"externalproject_add" will download each dependency into build directory, build and install.
#The path convention is
#
#<CMAKE BUILD DIR>/lib/<PROJECT-NAME>-prefix/
#<CMAKE BUILD DIR>/lib/<PROJECT-NAME>-prefix/src
#<CMAKE BUILD DIR>/lib/<PROJECT-NAME>-prefix/include
#<CMAKE BUILD DIR>/lib/<PROJECT-NAME>-prefix/lib
#<CMAKE BUILD DIR>/lib/<PROJECT-NAME>-prefix/bin
#...
#
#If this is first build, build "ex_all" first to download all dependencies, build and install.
#Then cmake will find each dependencies from the <CMAKE BUILD DIR>.
#
#]]#
#
## cmake external project
#include(ExternalProject)
##
## yaml-cpp
#externalproject_add(ex_yamlcpp
#        GIT_REPOSITORY      "https://github.com/jbeder/yaml-cpp.git"
#        GIT_TAG             "yaml-cpp-0.6.2"
#        CMAKE_ARGS
#        "-DBUILD_SHARED_LIBS=OFF"
#        "-DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>"
#        )
#
## raicommon
#externalproject_add(ex_raicommon
#        GIT_REPOSITORY      "https://github.com/eastskykang/raicommon.git"
#        GIT_TAG             "master"
#        CMAKE_ARGS
#        "-DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>"
#        )
#
## raigraphics
#externalproject_add(ex_raigraphics
#        GIT_REPOSITORY      "https://github.com/eastskykang/raigraphics.git"
#        GIT_TAG             "master"
#        CMAKE_ARGS
#        "-DINSTALL_LIBS=ON"
#        "-DBUILD_DEMOS=OFF"
#        "-DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>"
#        )
#
#externalproject_get_property(ex_raigraphics INSTALL_DIR)
#set(raigraphics_install_dir ${INSTALL_DIR})
#
## bullet
#if(BULLET)
#        externalproject_add(ex_bullet
#                GIT_REPOSITORY      "https://github.com/eastskykang/bullet3.git"
#                CMAKE_ARGS
#                "-DUSE_DOUBLE_PRECISION=ON"
#                "-DBUILD_SHARED_LIBS=OFF"
#                "-DBULLET2_MULTITHREADING=OFF"
#                "-DBUILD_BULLET3=ON"
#                "-DBUILD_PYBULLET=OFF"
#                "-DBUILD_BULLET2_DEMOS=OFF"
#                "-DBUILD_EXTRAS=ON"
#                "-DBUILD_UNIT_TESTS=OFF"
#                "-DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>"
#                )
#endif(BULLET)
#
## dart
#if(DART)
#        externalproject_add(ex_dart
#                GIT_REPOSITORY      "https://github.com/eastskykang/dart.git"
#                CMAKE_ARGS
#                "-DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>"
#                )
#endif(DART)
#
## mujoco
#if(MUJOCO)
#    externalproject_add(ex_mujoco
#        URL                 "https://github.com/google-deepmind/mujoco/releases/download/3.2.3/mujoco-3.2.3-linux-x86_64.tar.gz"
#        CONFIGURE_COMMAND   ""
#        BUILD_COMMAND       ""
#        INSTALL_COMMAND     ""
#        )
#endif(MUJOCO)
#
## physx
#if(PHYSX)
#    externalproject_add(ex_physx
#            URL                 "https://github.com/NVIDIA-Omniverse/PhysX/archive/refs/tags/106.1-physx-5.4.2.tar.gz"
#            CONFIGURE_COMMAND   ""
#            BUILD_COMMAND       ""
#            INSTALL_COMMAND     ""
#    )
#endif(PHYSX)
#
########################################################################################################################
## add ex_all as build target
#message(STATUS "The project depends on the following external libraries")
#message(STATUS "Build ex_all and cmake again if this is first build:")
#message(STATUS "  ex_raigraphics")
#
#
#
#
#
#
## common libs
#list(APPEND ex_projects ex_yamlcpp)
#list(APPEND ex_projects ex_raicommon)
#list(APPEND ex_projects ex_raigraphics)
#
## simulators
#if(BULLET)
#    message(STATUS "  ex_bullet")
#    list(APPEND ex_projects ex_bullet)
#endif(BULLET)
#
#if(MUJOCO)
#    message(STATUS "  ex_mujoco")
#    list(APPEND ex_projects ex_mujoco)
#endif(MUJOCO)
#
#if(DART)
#    message(STATUS "  ex_dart")
#    list(APPEND ex_projects ex_dart)
#endif(DART)
#
#if(PHYSX)
#    message(STATUS " ex_physx")
#    list(APPEND ex_projects ex_physx)
#endif(PHYSX)
#
## build ex_all
#add_custom_target(ex_all
#        DEPENDS
#        ${ex_projects}
#        )