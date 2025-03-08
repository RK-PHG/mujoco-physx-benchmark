/** PyXBox */

#include "PyXBox.hpp"

/** 盒体 */
physx_sim::object::PyXBox::PyXBox(double xLength,
                                  double yLength,
                                  double zLength,
                                  double mass,
                                  physx::PxRigidDynamic *actor,
                                  physx::PxMaterial* material)
                                  : PyXSingleBodyObject(actor,material),xLength_(xLength),yLength_(yLength),zLength_(zLength){
    this->mass_ = mass;
}