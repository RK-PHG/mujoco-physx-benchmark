#ifndef SIMBENCHMARK_PYXCHECKERBOARD_HPP
#define SIMBENCHMARK_PYXCHECKERBOARD_HPP

#include "PyXSingleBodyObject.hpp"

namespace physx_sim {
namespace object {

    class PyXCheckerBoard: public PyXSingleBodyObject {

    public:
        PyXCheckerBoard(physx::PxRigidStatic* actor,
                        physx::PxMaterial* material);

    };
}
}


#endif //SIMBENCHMARK_PYXCHECKERBOARD_HPP
