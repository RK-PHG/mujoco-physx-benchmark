#ifndef SIM_BENCHMARK_PYXARTICULATEDSYSTEM_HPP
#define SIM_BENCHMARK_PYXARTICULATEDSYSTEM_HPP

#include <Eigen/Geometry>
#include <common/UserHandle.hpp>
#include <PxPhysicsAPI.h>

#include "common/math.hpp"
#include "common/interface/SingleBodyObjectInterface.hpp"

namespace physx_sim{

    namespace object {

        typedef Eigen::Map<Eigen::Matrix<double,-1,1>> EigenVec;

        class PyXArticulatedSystem: public benchmark::object::ArticulatedSystemInterface {

        public:
            explicit PyXArticulatedSystem(physx::PxArticulationReducedCoordinate *articulation_);

            ~PyXArticulatedSystem();

            const EigenVec getGeneralizedCoordinate() override;

            const EigenVec getGeneralizedVelocity() override;

            void setGeneralizedCoordinate(const Eigen::VectorXd &jointState) override;

            void setGeneralizedVelocity(const Eigen::VectorXd &jointVel) override;

            void setGeneralizedCoordinate(std::initializer_list<double> jointState) override;

            void setGeneralizedVelocity(std::initializer_list<double> jointVel) override;

            void setGeneralizedForce(std::initializer_list<double> tau) override;

            void getState(Eigen::VectorXd &genco, Eigen::VectorXd &genvel) override;

            void setState(const Eigen::VectorXd &genco, const Eigen::VectorXd &genvel) override;

            void setGeneralizedForce(const Eigen::VectorXd &tau) override;

            const EigenVec getGeneralizedForce() override;

            int getDOF() override ;

            int getStateDimension() override ;

            void setColor(Eigen::Vector4d color)  override ;

            const Eigen::Map<Eigen::Matrix<double, 3, 1>> getLinearMomentumInCartesianSpace() override;

            double getTotalMass() override;

            double getEnergy(const benchmark::Vec<3> &gravity) override;

            double getKineticEnergy();

            double getPotentialEnergy(const benchmark::Vec<3> &gravity);

            void updateVisuals();

            void initVisual();

        private:

            benchmark::Vec<3> linearMomentum_;

            physx::PxArticulationReducedCoordinate* articulation_;

            physx::PxMaterial* material_;

            int dof_ = 0;

            int stateDimension_ = 0;

            bool isFixed_ = false;

        };
    }
}



#endif //SIM_BENCHMARK_PYXARTICULATEDSYSTEM_HPP
