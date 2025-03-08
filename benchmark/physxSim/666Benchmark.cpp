/** 此文件是 mujoco 666 测试的入口文件 */

#include "PyXSim.hpp"
#include "666Benchmark.hpp"

physx_sim::PyXSim *sim;
std::vector<benchmark::SingleBodyHandle> objList;
po::options_description desc;

/**
 * 检查物体间的穿透情况
 * 返回计算的误差值
 */
double penetrationCheck() {

    /** 记录穿透误差的数量 */
    double error = 0;
    int numObj = sim->getNumObject()-1;

    for (int i = 0; i < numObj; i++) {
        for (int j = i + 1; j < numObj; j++) {

            /** 获取方向 */
            sim->getSingleBodyHandle(i+1)->getPosition();

            double dist = (sim->getSingleBodyHandle(i+1)->getPosition() - sim->getSingleBodyHandle(j+1)->getPosition()).norm();

            /** 如果两个球体之间的距离小于他们两个半径之和，则出现误差 */
            if (dist < benchmark::sixsixsix::params.ballR * 2)
                error += (benchmark::sixsixsix::params.ballR * 2 - dist) * (benchmark::sixsixsix::params.ballR * 2 - dist);
        }

        /** 判断球体与地面之间的穿透误差，首先获取z轴坐标，然后判断球心到地面的距离 */
        if (abs(sim->getSingleBodyHandle(i+1)->getPosition()[1]) < benchmark::sixsixsix::params.ballR) {
            error +=
                    (benchmark::sixsixsix::params.ballR - abs(sim->getSingleBodyHandle(i+1)->getPosition()[1]))*
                    (benchmark::sixsixsix::params.ballR - abs(sim->getSingleBodyHandle(i+1)->getPosition()[1]));
        }
    }

    return error;
}

/** 初始化模拟 */
void setupSimulation(){

    sim = new physx_sim::PyXSim(800, 600, 0.5,
                                benchmark::NO_BACKGROUND);

    /** timeStep and max contact */
    sim->setTimeStep(0.1);

}

/** 初始化场景 */
void setupWorld() {

    /** 手动构建场景 */
    sim->setGravity({0,0, benchmark::sixsixsix::params.g});

    rai::RandomNumberGenerator<double> rand;
    rand.seed(benchmark::sixsixsix::params.randomSeed);

    auto checkerboard = sim->addCheckerboard(5.0, 200.0, 200.0, 0.1, bo::BOX_SHAPE, 1, -1, bo::GRID);

    checkerboard->setFrictionCoefficient(0);

    if(benchmark::sixsixsix::options.elasticCollision)
        checkerboard->setRestitutionCoefficient(1.0);

    for(int i = 0; i < benchmark::sixsixsix::params.n; i++) {
        for(int j = 0; j < benchmark::sixsixsix::params.n; j++) {
            for(int k = 0; k < benchmark::sixsixsix::params.n; k++) {

                // add object
                auto obj = sim->addSphere(benchmark::sixsixsix::params.ballR,
                                          benchmark::sixsixsix::params.ballM,0,0);
                // set position
                double x =
                        double(i) * benchmark::sixsixsix::params.gap
                        + rand.sampleUniform01() * benchmark::sixsixsix::params.perturbation;
                double y =
                        double(j) * benchmark::sixsixsix::params.gap
                        + rand.sampleUniform01() * benchmark::sixsixsix::params.perturbation;
                double z =
                        double(k) * benchmark::sixsixsix::params.gap
                        + rand.sampleUniform01() * benchmark::sixsixsix::params.perturbation
                        + benchmark::sixsixsix::params.H;

                obj->setPosition(x, y, z);
                obj->setFrictionCoefficient(0);
                if(benchmark::sixsixsix::options.elasticCollision)
                    obj->setRestitutionCoefficient(1.0);



                if(benchmark::sixsixsix::options.gui) {
                    if((i + j + k) % 3 == 0) {
                        obj.visual()[0]->setColor({0,1,0});
                    }
                    else if((i + j + k) % 3 == 1) {
                        obj.visual()[0]->setColor({1,0,0});
                    }
                    else if((i + j + k) % 3 == 2) {
                        obj.visual()[0]->setColor({0,0,1});
                    }
                }

                objList.push_back(obj);
            }
        }
    }

    if(benchmark::sixsixsix::options.gui) {
        sim->setLightPosition((float)benchmark::sixsixsix::params.lightPosition[0],
                              (float)benchmark::sixsixsix::params.lightPosition[1],
                              (float)benchmark::sixsixsix::params.lightPosition[2]);
        sim->cameraFollowObject(objList[objList.size() / 2], {0, 5, 2});
    }
}

/** 模拟循环 */
double simulationLoop(bool timer = true, bool error = true) {

    // 为错误向量预留时间
    benchmark::sixsixsix::data.setN(unsigned(benchmark::sixsixsix::options.T / benchmark::sixsixsix::options.dt));

    // 计时器
    StopWatch watch;
    if(timer)
        watch.start();

    // 循环步数为总时间除以每步的时长
    for(int i = 0; i < (int) (benchmark::sixsixsix::options.T / benchmark::sixsixsix::options.dt); i++) {

        if (benchmark::sixsixsix::options.gui && !sim->visualizerLoop(benchmark::sixsixsix::options.dt))
            break;

        // 保存错误
        if (error) {
            if (benchmark::sixsixsix::options.elasticCollision) {
                RAIFATAL("elastic collision is not supported for mujoco")
            }
            else {
                double error = penetrationCheck();
                benchmark::sixsixsix::data.error.push_back(error);
            }
        }

        sim->integrate();
    }

    double time = 0;
    if(timer)
        time = watch.measure();
    return time;
}

int main(int argc, const char* argv[]) {

    benchmark::sixsixsix::getParamsFromYAML(benchmark::sixsixsix::getYamlpath().c_str(),
                                            benchmark::PHYSX);

    if(benchmark::sixsixsix::options.elasticCollision)
        RAIFATAL("Elastic 666 test is not available for Physx")


    RAIINFO(
            std::endl << "=======================" << std::endl
                      << "Simulator: Physx" << std::endl
                      << "GUI      : " << benchmark::sixsixsix::options.gui << std::endl
                      << "ERP      : " << benchmark::sixsixsix::options.erpYN << std::endl
                      << "Timetep : " << benchmark::sixsixsix::options.dt << std::endl
                      << "-----------------------"
    )

    setupSimulation();

    setupWorld();

    double time = simulationLoop(true, true);

    double error = benchmark::sixsixsix::data.computeError();

    if(benchmark::sixsixsix::options.csv)
    benchmark::sixsixsix::printCSV(benchmark::sixsixsix::getCSVpath(),
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

    return 0;
}




