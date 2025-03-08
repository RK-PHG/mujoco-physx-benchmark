/** PyXSphere */

#include "PyXSphere.hpp"

/** 球体类型 */
physx_sim::object::PyXSphere::PyXSphere(double radius,
                                        double mass,
                                        physx::PxRigidDynamic *actor,
                                        physx::PxMaterial* material)
: PyXSingleBodyObject(actor,material),radius_(radius){
    this->mass_  =  mass;
}