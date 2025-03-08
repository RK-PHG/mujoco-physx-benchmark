/** PyxCheckerBoard */

#ifndef SIMBENCHMARK_PYXCHECKERBOARD_HPP
#define SIMBENCHMARK_PYXCHECKERBOARD_HPP

#include "PyXSingleBodyObject.hpp"

namespace physx_sim {
namespace object {
    /** 地面类型 */
    class PyXCheckerBoard: public PyXSingleBodyObject {

    public:
        PyXCheckerBoard(physx::PxRigidStatic* actor,
                        physx::PxMaterial* material);

    };
}
}


#endif //SIMBENCHMARK_PYXCHECKERBOARD_HPP
