#ifndef SIMBENCHMARK_PYXCAPSULE_HPP
#define SIMBENCHMARK_PYXCAPSULE_HPP

#include "PyXSingleBodyObject.hpp"


namespace physx_sim{
namespace object{

    class PyXCapsule: public PyXSingleBodyObject{

    public:
        PyXCapsule(double radius,
                   double height,
                   double mass,
                   physx::PxRigidDynamic* actor,
                   physx::PxMaterial* material);

    private:
        double radius_;
        double height_;

    };

}
}


#endif //SIMBENCHMARK_PYXCAPSULE_HPP
