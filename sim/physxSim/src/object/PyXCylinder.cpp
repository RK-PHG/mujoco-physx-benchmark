#include "PyXCylinder.hpp"

physx_sim::object::PyXCylinder::PyXCylinder(double radius,
                                            double height,
                                            physx::PxRigidDynamic *actor,
                                            physx::PxMaterial* material
                                            ):radius_(radius),height_(height){
    this->actor_ = actor;
}