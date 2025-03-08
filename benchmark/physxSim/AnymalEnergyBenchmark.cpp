#include "PyXSim.hpp"

#include "AnymalEnergyBenchmark.hpp"
#include "PyXBenchmark.hpp"

physx_sim::PyXSim *sim;
po::options_description desc;
std::vector<physx_sim::object::PyXArticulatedSystem*> anymals;

void setupSimulation() {
    sim = new physx_sim::PyXSim();
//    sim = new physx_sim::PyXSim(benchmark::anymal::freedrop::getURDFpath());
    sim->setTimeStep(benchmark::anymal::freedrop::options.dt);
}

void setupWorld() {

    // 添加一个平面
    auto checkerboard = sim->addCheckerboard(5.0, 200.0,
                                             200.0, 0.1, bo::BOX_SHAPE, 1, -1, bo::GRID);

    // 添加一个关节系统，同时设置广义坐标
    physx_sim::object::PyXArticulatedSystem* anymal = sim->world_->loadModel(benchmark::anymal::freedrop::getURDFpath());

    // 设置广义坐标
    anymal->setGeneralizedCoordinate({0,
                                           0,
                                           benchmark::anymal::freedrop::params.H,
                                           1.0, 0.0, 0.0, 0.0,
                                           0.03, 0.4, -0.8,
                                           0.03, -0.4, +0.8,
                                           -0.03, 0.4, -0.8,
                                           -0.03, -0.4, 0.8});


    // 设置广义力矩
    anymal->setGeneralizedForce(Eigen::VectorXd::Zero(anymal->getDOF()));

    // 将机器人添加到列表
    anymals.push_back(anymal);

    // 设置重力
    sim->setGravity({0,  0, benchmark::anymal::freedrop::params.g});

    // 获取总质量
    benchmark::anymal::freedrop::params.M = sim->world_->getTotalMass();

    // 施加一个力
    benchmark::anymal::freedrop::params.F =
            benchmark::anymal::freedrop::params.M * (-benchmark::anymal::freedrop::params.g) * 0.05;
}

double simulationLoop(bool timer = true, bool error = true) {

    // reserve error vector
    if(error)
        benchmark::anymal::freedrop::data.setN(
                unsigned(benchmark::anymal::freedrop::params.T2 / benchmark::anymal::freedrop::options.dt)
        );

    // timer start
    StopWatch watch;
    if(timer)
        watch.start();

    {
        // step1: applying force
        for (int t = 0;
             t < (int) (benchmark::anymal::freedrop::params.T1 / benchmark::anymal::freedrop::options.dt); t++) {
            anymals[0]->setGeneralizedForce({0, 0, benchmark::anymal::freedrop::params.F,
                                              0, 0, 0,
                                              0, 0, 0,
                                              0, 0, 0,
                                              0, 0, 0,
                                              0, 0, 0});
            sim->integrate();
        }
    }

    {
        // step2: freedrop
        for (int t = 0; t < (int) (benchmark::anymal::freedrop::params.T2 / benchmark::anymal::freedrop::options.dt); t++) {
            anymals[0]->setGeneralizedForce({0, 0, 0,
                                             0, 0, 0,
                                             0, 0, 0,
                                             0, 0, 0,
                                             0, 0, 0,
                                             0, 0, 0});

            if(error) {
                if(t==0)
                    benchmark::anymal::freedrop::data.E0 = anymals[0]->getEnergy({0, 0,
                                                                                                       benchmark::anymal::freedrop::params.g});
                double k = anymals[0]->getKineticEnergy();
                double p = anymals[0]->getPotentialEnergy({0,0,benchmark::anymal::freedrop::params.g});

                benchmark::anymal::freedrop::data.kineticE.push_back(k);
                benchmark::anymal::freedrop::data.potentialE.push_back(p);
            }
            sim->integrate();
        }
    }

    double time = 0;
    if(timer)
        time = watch.measure();
    return time;
}

int main(int argc, const char* argv[]) {

    benchmark::anymal::freedrop::addDescToOption(desc);
    benchmark::anymal::freedrop::getOptionsFromArg(argc, argv, desc);


    benchmark::anymal::freedrop::getParamsFromYAML(benchmark::anymal::freedrop::getYamlpath().c_str(),
                                                   benchmark::DART);

    RAIINFO(
            std::endl << "=======================" << std::endl
                      << "Simulator  : " << "Physx" << std::endl
//                      << "GUI        : " << benchmark::anymal::freedrop::options.gui << std::endl
//                      << "Solver     : " << "solver" << std::endl
//                      << "Integrator : " << "Integrator" << std::endl
                      << "Timestep   : " << benchmark::anymal::freedrop::options.dt << std::endl
                      << "-----------------------"
    )

    // trial1: get Error
    setupSimulation();
    setupWorld();
    double time = simulationLoop(true, true);
    double error = benchmark::anymal::freedrop::data.computeError();

    if(benchmark::anymal::freedrop::options.csv)
        benchmark::anymal::freedrop::printCSV(benchmark::anymal::freedrop::getCSVpath(),
                                              "t-SimName",
                                              "t-SolverName",
                                              "t-detectorName",
                                              "t-integratorName",
                                              time,
                                              error);

    RAIINFO(
            std::endl << "Timer : " << time << std::endl
                      << "Mean Error: " << error << std::endl
                      << "======================="
    )

    return 0;
}