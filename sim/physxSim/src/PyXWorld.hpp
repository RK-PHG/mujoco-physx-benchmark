/** PyXWorld */

#ifndef SIMBENCHMARK_PYXWORLD_HPP
#define SIMBENCHMARK_PYXWORLD_HPP

#include<PxPhysicsAPI.h>

#include "common/Configure.hpp"
#include "common/interface/WorldInterface.hpp"

#include "object/PyXSphere.hpp"
#include "object/PyXBox.hpp"
#include "object/PyXCapsule.hpp"
#include "object/PyXCheckerBoard.hpp"
#include "object/PyXCylinder.hpp"
#include "object/PyXArticulatedSystem.hpp"
#include "tinyxml2.h"

#include <unordered_map>

using namespace physx;

namespace physx_sim {

    enum SolverOption {
        SOLVER_PGS,
        SOLVER_CG,
        SOLVER_NEWTON
    };

    typedef Eigen::Map<Eigen::Matrix<double,-1,1>> EigenVec;
    typedef Eigen::Map<Eigen::Matrix<double,-1,-1>> EigenMat;

    struct Single3DContactProblem {
        Single3DContactProblem(const double x, const double y, const double z) {
            point_ = {x, y, z};
        };
        Eigen::Vector3d point_;
        Eigen::Vector3d normal_;
        double force_;
    };


    class PyXWorld : benchmark::WorldInterface {

        friend class PyXSim;
        friend class CollisionCallback;

    public:

        PyXWorld();

        virtual ~PyXWorld();

        object::PyXArticulatedSystem* loadModel(std::string modelPath);

        object::PyXSphere *addSphere(double radius, double mass, benchmark::Vec<3> pos = {0.0, 0.0, 0.0});

        object::PyXBox *addBox(double xLength,
                               double yLength,
                               double zLength,
                               double mass,
                               benchmark::Vec<3> pos = {0.0, 0.0, 0.0});

        object::PyXCheckerBoard *addCheckerboard(benchmark::Vec<3> pos={0.0,1.0,0.0});

        object::PyXCapsule *addCapsule(double radius,
                                       double height,
                                       double mass,
                                       benchmark::Vec<3> pos={0.0,0.0,0.0});


        object::PyXCheckerBoard *addCheckerboard(double gridSize,
                                               double xLength,
                                               double yLength,
                                               double reflectanceI,
                                               bo::CheckerboardShape shape,
                                               benchmark::CollisionGroupType collisionGroup,
                                               benchmark::CollisionGroupType collisionMask) override {
            return addCheckerboard();
        }

        object::PyXSphere *addSphere(double radius,
                                     double mass,
                                     benchmark::CollisionGroupType collisionGroup,
                                     benchmark::CollisionGroupType collisionMask) override {
            return addSphere(radius,mass);
        }

        object::PyXBox *addBox(double xLength,
                              double yLength,
                              double zLength,
                              double mass,
                              benchmark::CollisionGroupType collisionGroup,
                              benchmark::CollisionGroupType collisionMask) override {
            return addBox(xLength,yLength,zLength,mass);
        }

        object::PyXCapsule *addCapsule(double radius,
                                      double height,
                                      double mass,
                                      benchmark::CollisionGroupType collisionGroup,
                                      benchmark::CollisionGroupType collisionMask) override {
            return addCapsule(radius,height,mass);
        }


        object::PyXCylinder *addCylinder(double radius,
                                       double height,
                                       double mass,
                                       benchmark::CollisionGroupType collisionGroup=1,
                                       benchmark::CollisionGroupType collisionMask=-1) override {
            return nullptr;
        }

        void setGravity(const benchmark::Vec<3> &gravity) override;

        int getNumObject() override;

        void integrate(double dt) override {}

        void integrate1(double dt) override {}

        void integrate2(double dt) override {}

        double getKineticEnergy();

        double getPotentialEnergy(const benchmark::Vec<3> &gravity);

        double getEnergy(const benchmark::Vec<3> &gravity);

        void setNoSlipParameter(double friction);

        void setTimeStep(double timeStep);

        const Eigen::Map<Eigen::Matrix<double, 3, 1>> getLinearMomentumInCartesianSpace();

        double getTotalMass();

        void integrate();

        int getWorldNumContacts();

        PxArticulationLink *addLink(PxArticulationReducedCoordinate *articulation, PxArticulationLink *parent,
                                    tinyxml2::XMLElement *linkElement,
                                    std::unordered_map <std::string, std::vector<float>> &materials);

    public:

        PxPvd *pvd_;
        PxDefaultAllocator allocator_;
        PxDefaultErrorCallback errorCallback_;
        PxFoundation *foundation_;
        physx::PxPhysics* physics_;
        physx::PxScene * scene_;
        physx::PxDefaultCpuDispatcher* dispatcher_;
        std::vector<object::PyXArticulatedSystem*> articulatedSystemList_;
        std::vector<object::PyXSingleBodyObject*> objectList_;
        std::vector<Single3DContactProblem> contactProblemList_;
        benchmark::VecDyn generalizedCoordinate_;
        benchmark::VecDyn generalizedVelocity_;
        benchmark::VecDyn generalizedForce_;
        benchmark::Vec<3> linearMomentum_;
        double timeStep_;
    };

    class CollisionCallback : public PxSimulationEventCallback {
    public:
        CollisionCallback(physx_sim::PyXWorld* world):world_(world){}
        void onContact(const PxContactPairHeader& pairHeader, const PxContactPair* pairs, PxU32 nbPairs) override {
            world_->contactProblemList_.clear();
            for (PxU32 i = 0; i < nbPairs; ++i) {
                const PxContactPair& contactPair = pairs[i];
                PxContactPairPoint contactPoints[16];
                PxU32 extractedPoints = contactPair.extractContacts(contactPoints, 16);
                for (PxU32 j = 0; j < extractedPoints; ++j) {
                    const PxVec3& position = contactPoints[j].position;
                    Single3DContactProblem contact(position.x, position.y, position.z);
                    contact.normal_ = Eigen::Vector3d(contactPoints[j].normal.x, contactPoints[j].normal.y, contactPoints[j].normal.z);
                    contact.force_ = contactPoints[j].separation;
                    world_->contactProblemList_.push_back(contact);
                }
            }
        }

        void onTrigger(PxTriggerPair* pairs, PxU32 count) override {}
        void onConstraintBreak(PxConstraintInfo* constraints, PxU32 count) override {}
        void onWake(PxActor** actors, PxU32 count) override {}
        void onSleep(PxActor** actors, PxU32 count) override {}
        void onAdvance(const PxRigidBody*const* bodyBuffer, const PxTransform* poseBuffer, const PxU32 count) override {}
    private:
        physx_sim::PyXWorld* world_;

    };

}

#endif //SIMBENCHMARK_PYXWORLD_HPP
