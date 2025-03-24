//
// Created by kangd on 05.05.18.
//

#include "PyXSim.hpp"

#include "RollingBenchmark.hpp"
#include "PyXBenchmark.hpp"

// sim
physx_sim::PyXSim *sim;

// objects
std::vector<benchmark::SingleBodyHandle> objList;

// options description
po::options_description desc;

void setupSimulation() {

    if (benchmark::rolling::options.gui)
        sim = new physx_sim::PyXSim(800, 600, 0.5);    else
        sim = new physx_sim::PyXSim();

    // timestep
    sim->setTimeStep(benchmark::rolling::options.dt);

    /// no erp for dart
    if(benchmark::rolling::options.erpYN)
    RAIFATAL("erp is not supported for dart")
}

void setupWorld() {

    // add objects
    auto checkerboard = sim->addCheckerboard(5.0, 100.0, 100.0, 0.1, bo::BOX_SHAPE, 1, -1, bo::GRID);
    checkerboard->setFrictionCoefficient(benchmark::rolling::params.dartGroundMu);
    checkerboard->setRestitutionCoefficient(1.0);

    auto box = sim->addBox(20, 20, 1, 0.5, 0, 0);
    box->setPosition(0, 0, 2);
    box->setFrictionCoefficient(benchmark::rolling::params.dartBoxMu);
    objList.push_back(box);

    if(benchmark::rolling::options.gui)
        box.visual()[0]->setColor({0,1.0,0});

    for(int i = 0; i < benchmark::rolling::params.n; i++) {
        for(int j = 0; j < benchmark::rolling::params.n; j++) {
            auto ball = sim->addSphere(0.5, 1, 0, 0);
            ball->setPosition(i * 2.0 - 4.0,
                              j * 2.0 - 4.0,
                              2.5);
            ball->setFrictionCoefficient(benchmark::rolling::params.dartBallMu);
            ball->setRestitutionCoefficient(1.0);
            objList.push_back(ball);

            if(benchmark::rolling::options.gui)
                box.visual()[0]->setColor({0,1.0,0});
        }
    }

    // gravity
    sim->setGravity({0, 0, benchmark::rolling::params.g});

    if(benchmark::rolling::options.gui) {
        sim->setLightPosition((float)benchmark::rolling::params.lightPosition[0],
                              (float)benchmark::rolling::params.lightPosition[1],
                              (float)benchmark::rolling::params.lightPosition[2]);
        sim->cameraFollowObject(checkerboard, {30, 0, 15});
    }
}

double simulationLoop(bool timer = true, bool error = true) {

    // force
    Eigen::Vector3d force;
    if(benchmark::rolling::options.forceDirection == benchmark::rolling::FORCE_Y)
        force = {0,
                 benchmark::rolling::params.F,
                 0};
    else if(benchmark::rolling::options.forceDirection == benchmark::rolling::FORCE_XY)
        force = {benchmark::rolling::params.F * 0.5 * 80,
                 benchmark::rolling::params.F * 0.866025403784439 * 80,
                 0};

    // resever error vector
    benchmark::rolling::data.setN(unsigned(benchmark::rolling::params.T / benchmark::rolling::options.dt));

    // timer start
    StopWatch watch;
    if(timer)
        watch.start();

    for(int i = 0; i < (int) (benchmark::rolling::params.T / benchmark::rolling::options.dt); i++) {
        // gui
        if(benchmark::rolling::options.gui && !sim->visualizerLoop(benchmark::rolling::options.dt))
            break;

        // set force to box
        objList[0]->setExternalForce(force);

        // data save
        if(error) {
            benchmark::rolling::data.boxVel.push_back(objList[0]->getLinearVelocity());
            benchmark::rolling::data.boxPos.push_back(objList[0]->getPosition());
            benchmark::rolling::data.ballVel.push_back(objList[1]->getLinearVelocity());
            benchmark::rolling::data.ballPos.push_back(objList[1]->getPosition());
        }

        // step
        sim->integrate();
    }

    double time = 0;
    if(timer)
        time = watch.measure();
    return time;
}

int main(int argc, const char* argv[]) {

//    benchmark::rolling::addDescToOption(desc);
//    benchmark::physx::addDescToOption(desc);
//
//    benchmark::rolling::getOptionsFromArg(argc, argv, desc);
//    benchmark::dart::getOptionsFromArg(argc, argv, desc);

    benchmark::rolling::getParamsFromYAML(benchmark::rolling::getYamlpath().c_str(),
                                          benchmark::DART);

    RAIINFO(
            std::endl << "=======================" << std::endl
                      << "Simulator: DART" << std::endl
                      << "GUI      : " << benchmark::rolling::options.gui << std::endl
                      << "ERP      : " << benchmark::rolling::options.erpYN << std::endl
                      << "Force    : " << benchmark::rolling::options.forceDirection << std::endl
                      << "Timestep : " << benchmark::rolling::options.dt << std::endl
                      << "Solver   : "
                      << "Num iter : " << benchmark::rolling::options.numSolverIter << std::endl
                      << "-----------------------"
    )

    // trial1: get Error
    setupSimulation();
    setupWorld();
    double time = simulationLoop(false, true);
    double error = benchmark::rolling::data.computeError();

    if(benchmark::rolling::options.csv)
        benchmark::rolling::printCSV(benchmark::rolling::getCSVpath(),
                                     "simName",
                                     "solverName",
                                     "detectorName",
                                     "integratorName",
                                     time,
                                     error);

    RAIINFO(
            std::endl << "CPU time   : " << time << std::endl
                      << "mean error : " << error << std::endl
                      << "speed (Hz) : " << benchmark::rolling::params.T / benchmark::rolling::options.dt / time << std::endl
                      << "=======================" << std::endl
    )

    delete sim;
    return 0;
}