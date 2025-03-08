message(STATUS "  commonSim for common parts")

set(ex_raigraphics_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../lib/raigraphics")
set(raigraphics_include_dir "${ex_raigraphics_SOURCE_DIR}/include/")
set(raigraphics_lib_dir "${ex_raigraphics_SOURCE_DIR}/lib/")

set(ex_raicommon_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../lib/raicommon")
set(raicommon_include_dir "${ex_raicommon_SOURCE_DIR}/include/")
set(raicommon_lib_dir "${ex_raicommon_SOURCE_DIR}/lib/")


set(CommonSrc
        UserHandle.hpp
        WorldRG.hpp
        WorldRG.cpp
        Configure.hpp
        math.hpp
        interface/CheckerboardInterface.hpp
        interface/ArticulatedSystemInterface.hpp
        interface/WorldInterface.hpp
        interface/SingleBodyObjectInterface.hpp
        )

find_library(RAIGRAPHICS_LIB
        NAMES raigraphics
        PATHS ${raigraphics_lib_dir}
        )

add_library(commonSim ${CommonSrc})

target_include_directories(commonSim
        PUBLIC
        ${raigraphics_include_dir}
        ${raicommon_include_dir}
        )

target_link_libraries(commonSim
        ${RAIGRAPHICS_LIB}
        )
