message(STATUS "Build Sim libraries for the following engines:")

add_subdirectory(common)

if(MUJOCO)
    add_subdirectory(mujocoSim)
endif(MUJOCO)

if(PHYSX)
    add_subdirectory(physxSim)
endif(PHYSX)