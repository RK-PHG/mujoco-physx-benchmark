#ifndef SIMBENCHMARK_PYXBOX_HPP
#define SIMBENCHMARK_PYXBOX_HPP

#include "PyXSingleBodyObject.hpp"

namespace physx_sim {
namespace object {

    class PyXBox : public PyXSingleBodyObject{

    public:
        PyXBox(double xLength,
               double yLength,
               double zLength,
               double mass,
               physx::PxRigidDynamic* actor,
               physx::PxMaterial* material
               );
    private:
        double xLength_;
        double yLength_;
        double zLength_;
    };

}
}


#endif //SIMBENCHMARK_PYXBOX_HPP
