#include <MjcSim.hpp>

#include "MjcBenchmark.hpp"
#include "SlidersBenchmark.hpp"

mujoco_sim::MjcSim *sim;

// 用于保存仿真过程中滑块速度
std::vector<double> v1_list;    // 下面
std::vector<double> v2_list;    // 上面

// 用于保存仿真过程中滑块运动距离
std::vector<double> s1_list;    // 下面
std::vector<double> s2_list;    // 上面


void setupSimulation() {
    if (benchmark::slopeSlider::options.gui)
        sim = new mujoco_sim::MjcSim(800, 600, 0.5,
                                     benchmark::slopeSlider::getMujocoXMLpath().c_str(),
                                     benchmark::mujoco::getKeypath().c_str(),
                                     benchmark::NO_BACKGROUND,
                                     benchmark::mujoco::options.solverOption,
                                     benchmark::mujoco::options.integratorOption);
    else
        sim = new mujoco_sim::MjcSim(benchmark::slopeSlider::getMujocoXMLpath().c_str(),
                                     benchmark::mujoco::getKeypath().c_str(),
                                     benchmark::mujoco::options.solverOption,
                                     benchmark::mujoco::options.integratorOption);

    sim->setTimeStep(benchmark::slopeSlider::options.dt);
}


void setupWorld() {

    // 重力加速度
    sim->setGravity({0, 0, benchmark::slopeSlider::params.g});

    /// Note. for mujoco (frictional coefficient A-B) = max(coeff of A, coeff of B)
    sim->getSingleBodyHandle(0)->setFrictionCoefficient(benchmark::slopeSlider::params.groundMu);
    sim->getSingleBodyHandle(1)->setFrictionCoefficient(benchmark::slopeSlider::params.boxMu);
    sim->getSingleBodyHandle(2)->setFrictionCoefficient(benchmark::slopeSlider::params.boxMu);

    Eigen::VectorXd genCoord(14);
    genCoord << 0, 0, 1.0,
            0, 0, 0,
            0, 0, 0,
            3.0, 0, 0,
            0, 0;


    Eigen::VectorXd genVelocity(12);
    genVelocity << 0, 10, 0,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0;

    sim->setGeneralizedCoordinate(genCoord);
    sim->setGeneralizedVelocity(genVelocity);

    // 设置相机跟随
    if(benchmark::slopeSlider::options.gui) {
        sim->setLightPosition((float)benchmark::slopeSlider::params.lightPosition[0],
                              (float)benchmark::slopeSlider::params.lightPosition[1],
                              (float)benchmark::slopeSlider::params.lightPosition[2]);
        sim->cameraFollowObject(sim->getSingleBodyHandle(1), {30, 0, 15});
    }
}


double simulationLoop(bool timer = true, bool error = true) {

    // timer start
    StopWatch watch;
    if(timer)
        watch.start();

    for(int i = 0; i < (int) (benchmark::slopeSlider::params.T / benchmark::slopeSlider::options.dt); i++) {
        // gui
        if(benchmark::slopeSlider::options.gui && !sim->visualizerLoop(benchmark::slopeSlider::options.dt))
            break;

        std::cout << sim->getStateDimension() << std::endl;

        // s
        double s1 = sim->getSingleBodyHandle(1)->getPosition()[1];
        double s2 = sim->getSingleBodyHandle(2)->getPosition()[1];

        s1_list.push_back(s1);
        s2_list.push_back(s2);

        // velocity
        double v1 = sim->getSingleBodyHandle(1)->getLinearVelocity()[1];  //上面滑块速度
        double v2 = sim->getSingleBodyHandle(2)->getLinearVelocity()[1];  //下面滑块速度

        v1_list.push_back(v1);
        v2_list.push_back(v2);

        // integrate step2
        sim->integrate();
    }

    double time = 0;
    if(timer)
        time = watch.measure();
    return time;
}

int main() {

    // trial1: get Error
    setupSimulation();

    setupWorld();

    simulationLoop(false, false);

    // 相关参数计算
    double boxMu = benchmark::slopeSlider::params.boxMu;
    double groundMu = benchmark::slopeSlider::params.groundMu;
    double mu1 = boxMu;
    double mu2 = (boxMu + groundMu) / 2;
    double M = 8;
    double m = 1;
    double v = benchmark::slopeSlider::params.v;
    double g = - benchmark::slopeSlider::params.g;
    double p1 = 1 + m/M;
    double p2 = mu1 + mu2;
    double p3 = mu1 / mu2;

    // 算出上面的滑块理论应该滑动的距离
    double l = 0.5 * v * v / (p1 * p1) * p3 / p2 / g;

    std::ofstream outFile("sliders_test_result.csv");
    if (!outFile.is_open()) {
        std::cerr << "无法创建文件: " << "sliders_test_result.csv" << std::endl;
    }

    // 写入 CSV 标题
    outFile << "v1,v2,s1,s2\n";

    // 设置精度（保留6位小数）
    outFile << std::fixed << std::setprecision(6);

    // 逐行写入数据
    double s1_0 = s1_list[0];
    double s2_0 = s2_list[0];
    for (size_t i = 0; i < v1_list.size(); ++i) {
        outFile << v1_list[i] << ","
                << v2_list[i] << ","
                << s1_list[i] - s1_0 << ","
                << s2_list[i] - s2_0 << "\n";
    }

    outFile.close();
    std::cout << "数据已保存到: " << "sliders_test_result.csv" << std::endl;


    RAIINFO(
            std::endl << "上面滑块理论滑动距离:   " << l  << std::endl
                      << "上面滑块实际滑动距离:   " << s2_list[s2_list.size()-1] - s2_list[0] << std::endl
                      << "=======================" << std::endl
    )

    delete sim;
    return 0;

}