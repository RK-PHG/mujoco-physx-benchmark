/** phXSphere的定义 */

#ifndef SIMBENCHMARK_PYXSPHERE_HPP
#define SIMBENCHMARK_PYXSPHERE_HPP

#include "PyXSingleBodyObject.hpp"


namespace physx_sim{
namespace object{

    /** 球体类型 */
    class PyXSphere: public PyXSingleBodyObject {

    public:
        PyXSphere(double radius,
                  double mass,
                  physx::PxRigidDynamic* actor,
                  physx::PxMaterial* material
                  );

    private:
        double radius_;
    };
}
}

#endif //SIMBENCHMARK_PYXSPHERE_HPP
