//
// Created by kangd on 15.02.18.
//

#include <PyXSim.hpp>

#include "BouncingBenchmark.hpp"
#include "PyXBenchmark.hpp"

physx_sim::PyXSim *sim;
std::vector<benchmark::SingleBodyHandle> objList;
po::options_description desc;
std::vector<double> energy_list;

void setupSimulation() {
    if (benchmark::bouncing::options.gui)
        sim = new physx_sim::PyXSim(800, 600, 0.5);
    else
        sim = new physx_sim::PyXSim();

    // timestep
    sim->setTimeStep(benchmark::bouncing::options.dt);

    /// no erp for dart
    if(benchmark::bouncing::options.erpYN)
        RAIFATAL("erp is not supported for dart")
}

void setupWorld() {
    auto checkerboard = sim->addCheckerboard(5.0, 100.0, 100.0, 0.1, bo::BOX_SHAPE, 1, -1, bo::GRID);
    checkerboard->setFrictionCoefficient(benchmark::bouncing::params.mu_ground);
    checkerboard->setRestitutionCoefficient(1.0);

    for(int i = 0; i < benchmark::bouncing::params.n; i++) {
        for(int j = 0; j < benchmark::bouncing::params.n; j++) {
            auto ball = sim->addSphere(benchmark::bouncing::params.R, benchmark::bouncing::params.m);
            ball->setPosition(i * 2.0, j * 2.0, benchmark::bouncing::params.H);
            ball->setFrictionCoefficient(benchmark::bouncing::params.mu_ball);
            ball->setVelocity(0,5,25,0,0,0);
            ball->setRestitutionCoefficient(benchmark::bouncing::options.e);
            objList.push_back(ball);
        }
    }

    // gravity
    sim->setGravity({0, 0, benchmark::bouncing::params.g});

    if(benchmark::bouncing::options.gui) {
        sim->setLightPosition((float)benchmark::bouncing::params.lightPosition[0],
                              (float)benchmark::bouncing::params.lightPosition[1],
                              (float)benchmark::bouncing::params.lightPosition[2]);
        sim->cameraFollowObject(objList[20], {10, 0, 10});
    }
}

double simulationLoop(bool timer = true, bool error = true) {
    if(benchmark::bouncing::options.saveVideo)
        sim->startRecordingVideo("/tmp", "bullet-bouncing");

    // resever error vector
    benchmark::bouncing::data.setN(unsigned(benchmark::bouncing::params.T / benchmark::bouncing::options.dt));

    // timer start
    StopWatch watch;
    if(timer)
        watch.start();

    for(int i = 0; i < (int) (benchmark::bouncing::params.T / benchmark::bouncing::options.dt); i++) {
        // gui
        if (benchmark::bouncing::options.gui && !sim->visualizerLoop(benchmark::bouncing::options.dt))
            break;

        // data save
        if (error) {
            double E = 0;
            for(int j = 0; j < objList.size(); j++) {
               E += objList[j]->getEnergy({0, 0, benchmark::bouncing::params.g});

            }
            std::cout << E << std::endl;
            benchmark::bouncing::data.ballEnergy.push_back(E);
            energy_list.push_back(E);
        }

        sim->integrate();
    }

    if(benchmark::bouncing::options.saveVideo)
        sim->stopRecordingVideo();

    double time = 0;
    if(timer)
        time = watch.measure();
    return time;
}

int main(int argc, const char* argv[]) {

//    benchmark::bouncing::addDescToOption(desc);
//    benchmark::dart::addDescToOption(desc);
//
//    benchmark::bouncing::getOptionsFromArg(argc, argv, desc);
//    benchmark::dart::getOptionsFromArg(argc, argv, desc);

//    benchmark::bouncing::getParamsFromYAML(benchmark::bouncing::getYamlPath().c_str(),
//                                           benchmark::DART);

//    RAIINFO(
//            std::endl << "=======================" << std::endl
//                      << "Simulator: DART" << std::endl
//                      << "GUI      : " << benchmark::bouncing::options.gui << std::endl
//                      << "ERP      : " << benchmark::bouncing::options.erpYN << std::endl
//                      << "Res Coef : " << benchmark::bouncing::options.e << std::endl
//                      << "Timestep : " << benchmark::bouncing::options.dt << std::endl
//                      << "Solver   : " << benchmark::dart::options.solverName << std::endl
//                      << "-----------------------"
//    )

    // trial1: get Error
    setupSimulation();
    setupWorld();
    simulationLoop(false, true);
//    double error = benchmark::bouncing::data.computeError();

//    // reset
//    objList.clear();
//    delete sim;
//
//    // trial2: get CPU time
//    setupSimulation();
//    setupWorld();
//    double time = simulationLoop(true, false);

//    if(benchmark::bouncing::options.csv)
//        benchmark::bouncing::printCSV(benchmark::bouncing::getCSVpath(),
//                                      benchmark::dart::options.simName,
//                                      benchmark::dart::options.solverName,
//                                      benchmark::dart::options.detectorName,
//                                      benchmark::dart::options.integratorName,
//                                      time,
//                                      error);

    std::ofstream outFile("physx_free_drop_test_result.csv");
    if (!outFile.is_open()) {
        std::cerr << "无法创建文件: " << "physx_free_drop_test_result.csv" << std::endl;
    }

    // 写入 CSV 标题
    outFile << "energy\n";

    // 设置精度（保留6位小数）
    outFile << std::fixed << std::setprecision(6);

    for(size_t i = 0; i < energy_list.size(); ++i){
        outFile << energy_list[i] << "\n";
    }

    outFile.close();
    return 0;
}