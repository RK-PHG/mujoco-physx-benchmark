/**  PyXCylinder */

#ifndef SIMBENCHMARK_PYXCYLINDER_HPP
#define SIMBENCHMARK_PYXCYLINDER_HPP

#include "PyXSingleBodyObject.hpp"

namespace physx_sim {
namespace object {

    /** 圆柱体类型 */
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
