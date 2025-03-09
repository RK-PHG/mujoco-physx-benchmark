#ifndef SIMBENCHMARK_PYXSPHERE_HPP
#define SIMBENCHMARK_PYXSPHERE_HPP

#include "PyXSingleBodyObject.hpp"


namespace physx_sim{
namespace object{

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
