#ifndef SIMBENCHMARK_PYXSIM_HPP
#define SIMBENCHMARK_PYXSIM_HPP

#include <raiGraphics/RAI_graphics.hpp>

#include "common/WorldRG.hpp"
#include "PyXWorld.hpp"
#include "UserHandle.hpp"

namespace physx_sim {

class PyXSim : public benchmark::WorldRG {
public:

    PyXSim(int windowWidth,
            int windowHeight,
            float cms,
            int flags = 0);

    PyXSim();

    PyXSim(const std::string &modelPath);

    virtual ~PyXSim() override;

    void updateFrame() override;

    void integrate(double dt) override {}

    void integrate1(double dt) override {}

    void integrate2(double dt) override {}

    void setERP(double erp, double erp2, double frictionErp) override {};

    void setGravity(Eigen::Vector3d gravity) override ;

    void loop(double dt, double realTimeFactor = 1.0) override;

    int getWorldNumContacts() override ;

    int getNumObject() override ;

    void setNoSlipParameter(double friction);

    void integrate();

    void setTimeStep(double timeStep);

    benchmark::SingleBodyHandle getSingleBodyHandle(int index);

    const Eigen::Map<Eigen::Matrix<double, 3, 1>> getLinearMomentumInCartesianSpace();

    double getTotalMass();

    double getEnergy(const benchmark::Vec<3> &gravity);

    double getKineticEnergy();

    double getPotentialEnergy(const benchmark::Vec<3> &gravity);


public:

    physx_sim::PyXWorld* world_;

    benchmark::SingleBodyHandle addSphere(double radius, double mass);

    benchmark::SingleBodyHandle addBox(double xLength, double yLength, double zLength, double mass);

    benchmark::SingleBodyHandle addCylinder(double radius, double height, double mass);

    benchmark::SingleBodyHandle addCheckerboard(double gridSize, double xLength, double yLength);

    benchmark::SingleBodyHandle addCapsule(double radius, double height, double mass);

    // 不直接调用
    benchmark::SingleBodyHandle addSphere(double radius,
                                          double mass,
                                          int bodyId,
                                          int geomId) override ;

    // 不直接调用
    benchmark::SingleBodyHandle addBox(double xLength,
                                       double yLength,
                                       double zLength,
                                       double mass,
                                       int bodyId,
                                       int geomId) override ;

    // 不直接调用
    benchmark::SingleBodyHandle addCylinder(double radius,
                                            double height,
                                            double mass,
                                            int bodyId,
                                            int geomId) override {
    }

    // 不直接调用
    benchmark::SingleBodyHandle addCheckerboard(double gridSize,
                                                double xLength,
                                                double yLength,
                                                double reflectanceI,
                                                bo::CheckerboardShape shape,
                                                int bodyId,
                                                int geomId,
                                                int flags = 0) override ;

    // 不直接调用
    benchmark::SingleBodyHandle addCapsule(double radius,
                                           double height,
                                           double mass,
                                           int bodyId,
                                           int geomid) override ;

    ArticulatedSystemHandle addArticulatedSystem(physx_sim::object::PyXArticulatedSystem* articulatedSystem);

    std::vector<physx_sim::ArticulatedSystemHandle> asHandles_;

};
}


#endif //SIMBENCHMARK_PYXSIM_HPP
