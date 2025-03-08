/** PyXCapsule */

#include "PyXCapsule.hpp"

/** 胶囊体 */
physx_sim::object::PyXCapsule::PyXCapsule(double radius,
                                          double height,
                                          double mass,
                                          physx::PxRigidDynamic *actor,
                                          physx::PxMaterial* material
                                          )
                      : PyXSingleBodyObject(actor,material),radius_(radius),height_(height){
    this->mass_ = mass;
}

