#ifndef SIMBENCHMARK_PYXSINGLEBODYOBJECT_HPP
#define SIMBENCHMARK_PYXSINGLEBODYOBJECT_HPP

#include <Eigen/Geometry>
#include <common/UserHandle.hpp>
#include <PxPhysicsAPI.h>

#include "common/math.hpp"
#include "common/interface/SingleBodyObjectInterface.hpp"

namespace physx_sim {

    namespace object {

    class PyXSingleBodyObject: public benchmark::object::SingleBodyObjectInterface{

    public:

        PyXSingleBodyObject(physx::PxRigidDynamic* actor, physx::PxMaterial* material);

        PyXSingleBodyObject();

        const benchmark::eQuaternion getQuaternion() override ;

        void getQuaternion(benchmark::Vec<4>& quat) override ;

        const benchmark::eRotationMat getRotationMatrix() override ;

        void getRotationMatrix(benchmark::Mat<3,3>& rotation) override ;

        const benchmark::eVector3 getPosition() override ;

        const benchmark::eVector3 getComPosition() override ;

        const benchmark::eVector3 getLinearVelocity() override ;

        const benchmark::eVector3 getAngularVelocity() override ;

        void getPosition_W(benchmark::Vec<3>& pos_w) override ;

        double getKineticEnergy() override ;

        double getPotentialEnergy(const benchmark::Vec<3> &gravity) override ;

        double getEnergy(const benchmark::Vec<3> &gravity) override ;

        double getMass(){
           return mass_;
        }

         bool isMovable(){
             return this->isMovable_;
         }

        const benchmark::eVector3 getLinearMomentum() override;

        void setExternalForce(Eigen::Vector3d force) override ;

        void setExternalTorque(Eigen::Vector3d torque) override ;

        void setGeomFriction(benchmark::Vec<3> friction);

        void setFrictionCoefficient(double friction) override ;

        void setNoSlipCoefficient(double friction);

        void setPosition(Eigen::Vector3d originPosition) override ;

        void setPosition(double x, double y, double z) override ;

        void setOrientation(Eigen::Quaterniond quaternion) override ;

        void setOrientation(double w, double x, double y, double z) override ;

        void setOrientation(Eigen::Matrix3d rotationMatrix) override ;

        void setOrientationRandom() override ;

        void setPose(Eigen::Vector3d originPosition, Eigen::Quaterniond quaternion) override ;

        void setPose(Eigen::Vector3d originPosition, Eigen::Matrix3d rotationMatrix) override ;

        void setVelocity(Eigen::Vector3d linearVelocity, Eigen::Vector3d angularVelocity) override ;

        void setVelocity(double dx, double dy, double dz, double wx, double wy, double wz) override;

        void setRestitutionCoefficient(double restitution) override;

        bool isVisualizeFramesAndCom() const override {};


    protected:

        physx::PxRigidDynamic* actor_;

        physx::PxMaterial* material_;

        benchmark::Vec<4> quatTemp_ = {1.0, 0.0, 0.0, 0.0};

        benchmark::Mat<3, 3> rotMatTemp_;

        benchmark::Vec<3> posTemp_ = {0.0, 0.0, 0.0};

        benchmark::Vec<3> linVelTemp_ = {0.0, 0.0, 0.0};

        benchmark::Vec<3> angVelTemp_ = {0.0, 0.0, 0.0};

        benchmark::Vec<3> linearMomentum_ = {0, 0, 0};

        bool isMovable_ = true;

        double mass_ = 0;

        [[maybe_unused]] physx::PxRigidStatic* staticActor_;

    };

}   // object
}   // physx_sim


#endif //SIMBENCHMARK_PYXSINGLEBODYOBJECT_HPP
