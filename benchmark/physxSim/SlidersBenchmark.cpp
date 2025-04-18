#include "PyXSim.hpp"
#include "SlidersBenchmark.hpp"
#include <fstream>

physx_sim::PyXSim *sim;

std::vector<benchmark::SingleBodyHandle> objList;  //按照添加顺序 0-大滑块 1-小滑块

// 用于保存仿真过程中滑块速度
std::vector<double> v1_list;    // 下面
std::vector<double> v2_list;    // 上面

// 用于保存仿真过程中滑块运动距离
std::vector<double> s1_list;    // 下面
std::vector<double> s2_list;    // 上面


// 初始化仿真器
void setupSimulation() {

    // 初始化仿真器
    if (benchmark::slopeSlider::options.gui)
        sim = new physx_sim::PyXSim(800, 600, 0.5);
    else
        sim = new physx_sim::PyXSim();  // 无GUI版本

    sim->setTimeStep(benchmark::slopeSlider::options.dt);
}

void setupWorld() {

    // 地面
    auto checkerboard = sim->addCheckerboard(5.0, 100.0, 100.0, 0.1, bo::BOX_SHAPE, 1, -1, bo::GRID);
    checkerboard->setFrictionCoefficient(benchmark::slopeSlider::params.groundMu);

    // 下面的滑块
    auto box = sim->addBox(10, 40, 2, 8, 0, 0);
    box->setPosition(0, 0, 1);
    box->setVelocity(0,benchmark::slopeSlider::params.v,0,0,0,0);
    box->setFrictionCoefficient(benchmark::slopeSlider::params.boxMu);
    objList.push_back(box);

    // 上面的滑块
    auto box2 = sim->addBox(10, 5, 2, 1, 0, 0);
    box2->setPosition(0, 10, 3);
    box2->setFrictionCoefficient(benchmark::slopeSlider::params.boxMu);
    objList.push_back(box2);

    // 设置重力
    sim->setGravity({0, 0, benchmark::slopeSlider::params.g});

    // 场景摄像机
    if(benchmark::slopeSlider::options.gui) {
        sim->setLightPosition((float)benchmark::slopeSlider::params.lightPosition[0],
                              (float)benchmark::slopeSlider::params.lightPosition[1],
                              (float)benchmark::slopeSlider::params.lightPosition[2]);
        sim->cameraFollowObject(checkerboard, {30, 0, 15});
    }
}

double simulationLoop(bool timer = true, bool error = true) {

    // timer start
    StopWatch watch;
    if(timer)
        watch.start();

    for(int i = 0; i < (int) (benchmark::slopeSlider::params.T / benchmark::slopeSlider::options.dt); i++) {

        // 更新gui选项
        if(benchmark::slopeSlider::options.gui && !sim->visualizerLoop(benchmark::slopeSlider::options.dt))
            break;

        // s
        double s1 = objList[0]->getPosition()[1];
        double s2 = objList[1]->getPosition()[1];

        s1_list.push_back(s1);
        s2_list.push_back(s2);

        // velocity
        double v1 = objList[0]->getLinearVelocity()[1];  //上面滑块速度
        double v2 = objList[1]->getLinearVelocity()[1];  //下面滑块速度

        v1_list.push_back(v1);
        v2_list.push_back(v2);

        // step
        sim->integrate();
    }

    double time = 0;
    if(timer)
        time = watch.measure();
    return time;
}

int main() {

    // 初始化仿真场景
    setupSimulation();
    setupWorld();
    simulationLoop(false, true);

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

    return 0;
}