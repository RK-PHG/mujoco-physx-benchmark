/** 此文件是 mujoco 666 测试的入口文件 */

#include "MjcSim.hpp"

#include "666Benchmark.hpp"
#include "MjcBenchmark.hpp"

mujoco_sim::MjcSim *sim;
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
      double dist = (sim->getSingleBodyHandle(i+1)->getPosition() - sim->getSingleBodyHandle(j+1)->getPosition()).norm();

      /** 如果两个球体之间的距离小于他们两个半径之和，则出现误差 */
      if (dist < benchmark::sixsixsix::params.ballR * 2)
        error += (benchmark::sixsixsix::params.ballR * 2 - dist) * (benchmark::sixsixsix::params.ballR * 2 - dist);
    }

    /** 判断球体与地面之间的穿透误差，首先获取z轴坐标，然后判断球心到地面的距离 */
    if (sim->getSingleBodyHandle(i+1)->getPosition()[2] < benchmark::sixsixsix::params.ballR) {
      error +=
          (benchmark::sixsixsix::params.ballR - sim->getSingleBodyHandle(i+1)->getPosition()[2]) *
              (benchmark::sixsixsix::params.ballR - sim->getSingleBodyHandle(i+1)->getPosition()[2]);
    }
  }
  return error;
}

/** 初始化模拟 */
void setupSimulation() {

  if (benchmark::sixsixsix::options.gui)
    sim = new mujoco_sim::MjcSim(800, 600, 0.5,
                                      benchmark::sixsixsix::getMujocoXMLpath().c_str(),
                                      benchmark::mujoco::getKeypath().c_str(),
                                      benchmark::NO_BACKGROUND,
                                      benchmark::mujoco::options.solverOption,
                                      benchmark::mujoco::options.integratorOption);
  else
    sim = new mujoco_sim::MjcSim(benchmark::sixsixsix::getMujocoXMLpath().c_str(),
                                      benchmark::mujoco::getKeypath().c_str(),
                                      benchmark::mujoco::options.solverOption,
                                      benchmark::mujoco::options.integratorOption);

  /** timestep and max contact */
  sim->setTimeStep(benchmark::sixsixsix::options.dt);
}

/** 重置模拟环境 */
void resetWorld() {
  sim->resetSimulation();
}


/** 初始化场景 */
void setupWorld() {

  /** 初始化重力加速度 */
  sim->setGravity({0, 0, benchmark::sixsixsix::params.g});

  const int n = benchmark::sixsixsix::params.n;

  /** 设置球体颜色，仅用于支持图形界面的初始化 */
  for(int i = 0; i < n; i++) {
    for(int j = 0; j < n; j++) {
      for(int k = 0; k < n; k++) {
        if(benchmark::sixsixsix::options.gui) {
          if((i + j + k) % 3 == 0) {
            sim->getSingleBodyHandle(i * n * n + j * n + k + 1).visual()[0]->setColor(
                {benchmark::mujoco::color[0],
                 benchmark::mujoco::color[1],
                 benchmark::mujoco::color[2]});
          }
          else if((i + j + k) % 3 == 1) {
            sim->getSingleBodyHandle(i * n * n + j * n + k + 1).visual()[0]->setColor(
                {benchmark::mujoco::color[0],
                 benchmark::mujoco::color[1],
                 benchmark::mujoco::color[2]});
          }
          else if((i + j + k) % 3 == 2) {
            sim->getSingleBodyHandle(i * n * n + j * n + k + 1).visual()[0]->setColor(
                {benchmark::mujoco::color[0],
                 benchmark::mujoco::color[1],
                 benchmark::mujoco::color[2]});
          }
        }
      }
    }
  }

  if(benchmark::sixsixsix::options.gui) {
    sim->setLightPosition((float)benchmark::sixsixsix::params.lightPosition[0],
                          (float)benchmark::sixsixsix::params.lightPosition[1],
                          (float)benchmark::sixsixsix::params.lightPosition[2]);
    sim->cameraFollowObject(sim->getSingleBodyHandle(
        (sim->getNumObject() - 1) / 2), {0, 5, 2});
  }
}

/** 模拟循环 */
double simulationLoop(bool timer = true, bool error = true) {

  // 录制屏幕选项
  if(benchmark::sixsixsix::options.gui && benchmark::sixsixsix::options.saveVideo)
    sim->startRecordingVideo("/tmp", "mujoco-666");

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

  if(benchmark::sixsixsix::options.saveVideo)
    sim->stopRecordingVideo();

  double time = 0;
  if(timer)
    time = watch.measure();
  return time;
}

int main(int argc, const char* argv[]) {

  benchmark::sixsixsix::addDescToOption(desc);
  benchmark::mujoco::addDescToOption(desc);

  benchmark::sixsixsix::getOptionsFromArg(argc, argv, desc);
  benchmark::mujoco::getOptionsFromArg(argc, argv, desc);

  benchmark::sixsixsix::getParamsFromYAML(benchmark::sixsixsix::getYamlpath().c_str(),
                                         benchmark::MUJOCO);

  if(benchmark::sixsixsix::options.elasticCollision)
    RAIFATAL("Elastic 666 test is not available for MUJOCO")

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: MUJOCO" << std::endl
                << "GUI      : " << benchmark::sixsixsix::options.gui << std::endl
                << "ERP      : " << benchmark::sixsixsix::options.erpYN << std::endl
                << "Timestep : " << benchmark::sixsixsix::options.dt << std::endl
                << "Solver   : " << benchmark::mujoco::options.solverName << std::endl
                << "-----------------------"
  )

  // trial1: get Error
  setupSimulation();
  setupWorld();
  simulationLoop(false, true);
  double error = benchmark::sixsixsix::data.computeError();

  double time = 0;

  if(benchmark::sixsixsix::options.csv)
    benchmark::sixsixsix::printCSV(benchmark::sixsixsix::getCSVpath(),
                                   benchmark::mujoco::options.simName,
                                   benchmark::mujoco::options.solverName,
                                   benchmark::mujoco::options.detectorName,
                                   benchmark::mujoco::options.integratorName,
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
