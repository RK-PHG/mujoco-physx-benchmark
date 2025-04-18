#include "PyXSim.hpp"
#include "RollingBenchmark.hpp"

physx_sim::PyXSim *sim;

// 单体列表
std::vector<benchmark::SingleBodyHandle> objList;


// 初始化仿真环境
void setupSimulation() {

    // 支持GUI的仿真选项
    if (benchmark::rolling::options.gui)
        sim = new physx_sim::PyXSim(800, 600, 0.5); // 这三个参数都是窗口相关的，不用管
    else
        sim = new physx_sim::PyXSim(); // 无窗口世界接口

    // 设置时间步长
    sim->setTimeStep(benchmark::rolling::options.dt);
}

// 初始化场景
void setupWorld() {

    // 地面
    auto checkerboard = sim->addCheckerboard(5.0, 100.0, 100.0);
    checkerboard->setFrictionCoefficient(benchmark::rolling::params.mjcGroundMu);

    // 下面的平板滑动
    auto box = sim->addBox(20, 20, 1, 10);
    box->setPosition(0, 0, 0.5);
    box->setFrictionCoefficient(benchmark::rolling::params.mjcBoxMu);
    objList.push_back(box);

    // 逐个添加平板上的小球
    for(int i = 0; i < benchmark::rolling::params.n; i++) {
        for(int j = 0; j < benchmark::rolling::params.n; j++) {
            auto ball = sim->addSphere(0.5, 1);
            ball->setPosition(i * 2.0 - 4.0,
                              j * 2.0 - 4.0,
                              1.5);
            ball->setFrictionCoefficient(benchmark::rolling::params.mjcBallMu);
            ball->setRestitutionCoefficient(1.0);
            objList.push_back(ball);
        }
    }

    // 重力加速度
    sim->setGravity({0, 0, benchmark::rolling::params.g});

    // 相机设置
    if(benchmark::rolling::options.gui) {
        sim->setLightPosition((float)benchmark::rolling::params.lightPosition[0],
                              (float)benchmark::rolling::params.lightPosition[1],
                              (float)benchmark::rolling::params.lightPosition[2]);
        sim->cameraFollowObject(checkerboard, {30, 0, 15});
    }
}

// 仿真循环
double simulationLoop(bool timer = true, bool error = true) {

    // 在每个循环中对小球施加力
    Eigen::Vector3d force;
    if(benchmark::rolling::options.forceDirection == benchmark::rolling::FORCE_Y)
        force = {0, benchmark::rolling::params.F, 0};
    else if (benchmark::rolling::options.forceDirection == benchmark::rolling::FORCE_XY)
        force = {benchmark::rolling::params.F * 0.5,
                 benchmark::rolling::params.F * 0.866025403784439,
                 0};

    // 预留错误向量
    benchmark::rolling::data.setN(unsigned(benchmark::rolling::params.T / benchmark::rolling::options.dt));

    // 计时开始
    StopWatch watch;
    if(timer)
        watch.start();

    for(int i = 0; i < (int) (benchmark::rolling::params.T / benchmark::rolling::options.dt); i++) {

        // gui
        if(benchmark::rolling::options.gui && !sim->visualizerLoop(benchmark::rolling::options.dt))
            break;

        // 对下面的盒体施加力
        objList[0]->setExternalForce(force);

        // 计算误差
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

int main() {

    setupSimulation();
    setupWorld();
    double time = simulationLoop(true, true);
    double error = benchmark::rolling::data.computeError();
    RAIINFO(
            std::endl << "CPU time   : " << time << std::endl
                      << "mean error : " << error << std::endl
                      << "speed (Hz) : " << benchmark::rolling::params.T / benchmark::rolling::options.dt / time << std::endl
                      << "=======================" << std::endl
    )

    return 0;
}