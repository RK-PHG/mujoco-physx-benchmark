#ifndef SIMBENCHMARK_PYXCYLINDER_HPP
#define SIMBENCHMARK_PYXCYLINDER_HPP

#include "PyXSingleBodyObject.hpp"

namespace physx_sim {
namespace object {

    class PyXCylinder:public PyXSingleBodyObject {
    public:
        PyXCylinder(double radius,
                    double height,
                    physx::PxRigidDynamic* actor,
                    physx::PxMaterial* material);
    private:
        double radius_;
        double height_;
    };

}
}


#endif //SIMBENCHMARK_PYXCYLINDER_HPP
