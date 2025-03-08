/** 小球弹跳 */

#include "PyXSim.hpp"

#include "PyXBenchmark.hpp"
#include "SnippetRender.h"
#include "SnippetUtils.h"
#include "SnippetCamera.h"
#include "BouncingBenchmark.hpp"

physx_sim::PyXSim *sim;
po::options_description desc;
Snippets::Camera* sCamera;
std::vector<benchmark::SingleBodyHandle> objList;


/** 初始化模拟 */
void setupSimulation() {

    sim = new physx_sim::PyXSim();

    /** timeStep and max contact */
    sim->setTimeStep(benchmark::bouncing::options.dt);

}


void setupWorld() {
    auto checkerboard = sim->addCheckerboard(5.0, 100.0, 100.0, 0.1, bo::BOX_SHAPE, 1, -1, bo::GRID);
    checkerboard->setFrictionCoefficient(benchmark::bouncing::params.mu_ground);
    checkerboard->setRestitutionCoefficient(1.0);

    for(int i = 0; i < benchmark::bouncing::params.n; i++) {
        for(int j = 0; j < benchmark::bouncing::params.n; j++) {
            auto ball = sim->addSphere(benchmark::bouncing::params.R, benchmark::bouncing::params.m,0,0);
            ball->setPosition(i * 2.0, benchmark::bouncing::params.H, j * 2.0);
            ball->setFrictionCoefficient(benchmark::bouncing::params.mu_ball);
            ball->setRestitutionCoefficient(benchmark::bouncing::options.e);
        }
    }

    // gravity
    sim->setGravity({0, benchmark::bouncing::params.g, 0});
}

double simulationLoop(bool timer = true, bool error = true) {

    // resever error vector
    benchmark::bouncing::data.setN(unsigned(benchmark::bouncing::params.T / benchmark::bouncing::options.dt));

    // timer start
    StopWatch watch;
    if(timer)
        watch.start();

    for(int i = 0; i < (int) (benchmark::bouncing::params.T / benchmark::bouncing::options.dt); i++) {

        // data save
        if (error) {
            double E = 0;
            for(int j = 0; j < objList.size(); j++) {
                E += objList[j]->getEnergy({0, 0, benchmark::bouncing::params.g});
            }
            benchmark::bouncing::data.ballEnergy.push_back(E);
        }

        sim->integrate();
    }

    double time = 0;
    if(timer)
        time = watch.measure();
    return time;
}

int main(int argc, const char* argv[]) {

    benchmark::bouncing::addDescToOption(desc);
    benchmark::bouncing::getOptionsFromArg(argc, argv, desc);

    benchmark::bouncing::getParamsFromYAML(benchmark::bouncing::getYamlPath().c_str(),
                                           benchmark::PHYSX);

    RAIINFO(
            std::endl << "=======================" << std::endl
                      << "Simulator: DART" << std::endl
                      << "GUI      : " << benchmark::bouncing::options.gui << std::endl
                      << "ERP      : " << benchmark::bouncing::options.erpYN << std::endl
                      << "Res Coef : " << benchmark::bouncing::options.e << std::endl
                      << "Timestep : " << benchmark::bouncing::options.dt << std::endl
                      << "Solver   : " << "Solver" << std::endl
                      << "-----------------------"
    )

    // trial1: get Error
    setupSimulation();
    setupWorld();
    double time = simulationLoop(true, true);
    double error = benchmark::bouncing::data.computeError();


    if(benchmark::bouncing::options.csv)
        benchmark::bouncing::printCSV(benchmark::bouncing::getCSVpath(),
                                      "simName",
                                      "solverName",
                                      "detectorName",
                                      "integratorName",
                                      time,
                                      error);

    RAIINFO(
            std::endl << "CPU time   : " << time << std::endl
                      << "mean error : " << error << std::endl
                      << "=======================" << std::endl
    )

    delete sim;
    return 0;
}