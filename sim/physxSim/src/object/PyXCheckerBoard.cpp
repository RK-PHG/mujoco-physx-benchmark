#include "PyXCheckerBoard.hpp"

physx_sim::object::PyXCheckerBoard::PyXCheckerBoard(physx::PxRigidStatic* actor,
                                                    physx::PxMaterial* material){
    this->staticActor_ = actor;
    this->material_ = material;
}
