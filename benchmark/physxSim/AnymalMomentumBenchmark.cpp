#include "PyXSim.hpp"

#include "AnymalMomentumBenchmark.hpp"
#include "PyXBenchmark.hpp"

physx_sim::PyXSim *sim;
po::options_description desc;
std::vector<benchmark::SingleBodyHandle> balls;
std::vector<physx_sim::object::PyXArticulatedSystem*> anymals;

void setupSimulation() {

    sim = new physx_sim::PyXSim(800, 600, 0.5,
                                benchmark::NO_BACKGROUND);

    sim->setTimeStep(0.1);

}

void setupWorld() {
  // 添加平面
  auto checkerboard = sim->addCheckerboard(2, 100, 100, 0.1, bo::BOX_SHAPE, 1, -1, bo::GRID);

  // 添加球
  auto ball = sim->addSphere(0.2, benchmark::anymal::zerogravity::params.m, 0, 0);
  ball->setPosition(0,
                    benchmark::anymal::zerogravity::params.x0,
                    benchmark::anymal::zerogravity::params.H);
  ball->setVelocity(0, benchmark::anymal::zerogravity::params.v0/8, 0, 0, 0, 0);
  balls.push_back(ball);

  // 添加关节系统
  physx_sim::object::PyXArticulatedSystem* anymal = sim->world_->loadModel(benchmark::anymal::zerogravity::getURDFpath());
  anymals.push_back(anymal);

  // 设置关节初始位置
  anymals[0]->setGeneralizedCoordinate({0,
                                    0,
                                    benchmark::anymal::zerogravity::params.H,
                                    1.0, 0.0, 0.0, 0.0,
                                    0.03, 0.4, -0.8,
                                    0.03, -0.4, +0.8,
                                    -0.03, 0.4, -0.8,
                                    -0.03, -0.4, 0.8});

  // gravity
  sim->setGravity({0, 0, 0});

  // mass
  benchmark::anymal::zerogravity::params.M = anymals[0]->getTotalMass();
  if(benchmark::anymal::zerogravity::options.gui)
    sim->cameraFollowObject(checkerboard, {10.0, 0.0, 1.0});
}

double simulationLoop(bool timer = true, bool error = true) {

  // resever error vector
  if(error)
    benchmark::anymal::zerogravity::data.setN(
        unsigned(benchmark::anymal::zerogravity::params.T / benchmark::anymal::zerogravity::options.dt)
    );

  // timer start
  StopWatch watch;
  if(timer)
    watch.start();
  for(int i = 0; i < (int) (benchmark::anymal::zerogravity::params.T / benchmark::anymal::zerogravity::options.dt); i++) {
  // gui
  if(benchmark::anymal::zerogravity::options.gui && !sim->visualizerLoop(benchmark::anymal::zerogravity::options.dt))
      break;


      // data save
    if(error) {
      benchmark::anymal::zerogravity::data.ballMomentum.push_back(
          balls[0]->getLinearMomentum()
      );
      benchmark::anymal::zerogravity::data.anymalMomentum.push_back(
          anymals[0]->getLinearMomentumInCartesianSpace()
      );
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

    benchmark::anymal::zerogravity::addDescToOption(desc);
    benchmark::anymal::zerogravity::getOptionsFromArg(argc, argv, desc);


    benchmark::anymal::zerogravity::getParamsFromYAML(benchmark::anymal::zerogravity::getYamlpath().c_str(),
                                                   benchmark::DART);

    RAIINFO(
            std::endl << "=======================" << std::endl
                      << "Simulator  : " << "Physx" << std::endl
                      << "-----------------------"
    )

    // trial1: get Error
    setupSimulation();
    setupWorld();
    double time = simulationLoop(true, true);
    double error = benchmark::anymal::zerogravity::data.computeError();

    if(benchmark::anymal::zerogravity::options.csv)
        benchmark::anymal::zerogravity::printCSV(benchmark::anymal::zerogravity::getCSVpath(),
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