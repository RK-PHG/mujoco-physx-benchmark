/** 此文件是 mujoco 666 测试的入口文件 */

#include "PyXSim.hpp"

#include "PyXBenchmark.hpp"
#include "SnippetRender.h"
#include "SnippetUtils.h"
#include "SnippetCamera.h"
#include "Elastic666Benchmark.hpp"

physx_sim::PyXSim *sim;
po::options_description desc;
Snippets::Camera* sCamera;
std::vector<benchmark::SingleBodyHandle> objList;

/**
 * 检查物体间的穿透情况
 * 返回计算的误差值
 */
double computeEnergy() {
    double energy = 0;
    for(int j = 0; j < objList.size(); j++)
        energy += objList[j]->getEnergy({0.0,  benchmark::elasticsixsixsix::params.g,0.0});
    return energy;
}

/** 初始化模拟 */
void setupSimulation() {

    sim = new physx_sim::PyXSim();

    /** timeStep and max contact */
    sim->setTimeStep(benchmark::elasticsixsixsix::options.dt);

}

/** 初始化场景 */
void setupWorld() {

    /** 手动构建场景 */
    sim->setGravity({0,benchmark::elasticsixsixsix::params.g,0.0});

    rai::RandomNumberGenerator<double> rand;
    rand.seed(benchmark::elasticsixsixsix::params.randomSeed);

    auto checkerboard = sim->addCheckerboard(5.0, 200.0, 200.0, 0.1, bo::BOX_SHAPE, 1, -1, bo::GRID);

    checkerboard->setFrictionCoefficient(0);

    checkerboard->setRestitutionCoefficient(1.0);

    for(int i = 0; i < benchmark::elasticsixsixsix::params.n; i++) {
        for(int j = 0; j < benchmark::elasticsixsixsix::params.n; j++) {
            for(int k = 0; k < benchmark::elasticsixsixsix::params.n; k++) {

                // add object
                auto obj = sim->addSphere(benchmark::elasticsixsixsix::params.ballR,
                                          benchmark::elasticsixsixsix::params.ballM,0,0);
                // set position
                double x =
                        double(i) * benchmark::elasticsixsixsix::params.gap
                        + rand.sampleUniform01() * benchmark::elasticsixsixsix::params.perturbation;
                double y =
                        double(j) * benchmark::elasticsixsixsix::params.gap
                        + rand.sampleUniform01() * benchmark::elasticsixsixsix::params.perturbation;
                double z =
                        double(k) * benchmark::elasticsixsixsix::params.gap
                        + rand.sampleUniform01() * benchmark::elasticsixsixsix::params.perturbation
                        + benchmark::elasticsixsixsix::params.H;

                obj->setPosition(x, z, y);
                obj->setFrictionCoefficient(0);
                obj->setRestitutionCoefficient(1.0);
                objList.push_back(obj);
            }
        }
    }
}

/** 模拟循环 */
double simulationLoop(bool timer = true, bool error = true) {

    // 为错误向量预留时间
    benchmark::elasticsixsixsix::data.setN(unsigned(benchmark::elasticsixsixsix::options.T / benchmark::elasticsixsixsix::options.dt));

    // 计时器
    StopWatch watch;
    if(timer)
        watch.start();

    // 循环步数为总时间除以每步的时长
    for(int i = 0; i < (int) (benchmark::elasticsixsixsix::options.T / benchmark::elasticsixsixsix::options.dt); i++) {
        if (error) {
            static double E0 = 0;
            if(i==0)
                E0 = computeEnergy();

            double error = pow(computeEnergy() - E0, 2);
            benchmark::elasticsixsixsix::data.error.push_back(error);
        }

        sim->integrate();
    }

    double time = 0;
    if(timer)
        time = watch.measure();
    return time;
}

void renderCallback()
{
    Snippets::startRender(sCamera);

    PxScene* scene;
    PxGetPhysics().getScenes(&scene,1);
    PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
    if(nbActors)
    {
        std::vector<PxRigidActor*> actors(nbActors);
        scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<PxActor**>(&actors[0]), nbActors);
        Snippets::renderActors(&actors[0], static_cast<PxU32>(actors.size()), true);
    }

    Snippets::finishRender();
}

void exitCallback(void)
{
    delete sCamera;
}


int main(int argc, const char* argv[]) {

    benchmark::elasticsixsixsix::getParamsFromYAML(benchmark::elasticsixsixsix::getYamlpath().c_str(),
                                            benchmark::PHYSX);


    RAIINFO(
            std::endl << "=======================" << std::endl
                      << "Simulator: Physx" << std::endl
                      << "GUI      : " << benchmark::elasticsixsixsix::options.gui << std::endl
                      << "ERP      : " << benchmark::elasticsixsixsix::options.erpYN << std::endl
                      << "Timetep : " << benchmark::elasticsixsixsix::options.dt << std::endl
                      << "-----------------------"
    )

    setupSimulation();

    setupWorld();

    double time = simulationLoop(true, true);

    double error = benchmark::elasticsixsixsix::data.computeError();

    if(benchmark::elasticsixsixsix::options.csv)
        benchmark::elasticsixsixsix::printCSV(benchmark::elasticsixsixsix::getCSVpath(),
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

//    glutMainLoop();

    return 0;
}




