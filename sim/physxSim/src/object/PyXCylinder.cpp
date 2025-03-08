/** PyXCylinder */

#include "PyXCylinder.hpp"

/** 圆柱体类型 (physx默认不支持) */
physx_sim::object::PyXCylinder::PyXCylinder(double radius,
                                            double height,
                                            physx::PxRigidDynamic *actor,
                                            physx::PxMaterial* material
                                            ):radius_(radius),height_(height){
    this->actor_ = actor;
}