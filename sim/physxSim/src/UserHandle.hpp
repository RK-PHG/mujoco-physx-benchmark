
#ifndef SIMBENCHMARK_USERHANDLE_HPP
#define SIMBENCHMARK_USERHANDLE_HPP

namespace physx_sim {

    typedef benchmark::UserObjectHandle<physx_sim::object::PyXSingleBodyObject> SingleBodyHandle;
    typedef benchmark::UserObjectHandle<physx_sim::object::PyXArticulatedSystem> ArticulatedSystemHandle;

} // benchmark

#endif //SIMBENCHMARK_USERHANDLE_HPP