/** PyxCheckerBoard */

#include "PyXCheckerBoard.hpp"

/** 平面 */
physx_sim::object::PyXCheckerBoard::PyXCheckerBoard(physx::PxRigidStatic* actor,
                                                    physx::PxMaterial* material){
    this->staticActor_ = actor;
    this->material_ = material;
}
