#include "PyXSim.hpp"

#include "PyXBenchmark.hpp"
#include "SnippetRender.h"
#include "SnippetUtils.h"
#include "SnippetCamera.h"
#include "KaplaBenchmark.hpp"


physx_sim::PyXSim *sim;
po::options_description desc;
Snippets::Camera* sCamera;
std::vector<benchmark::SingleBodyHandle> objList;

/** 初始化模拟 */
void setupSimulation() {

    sim = new physx_sim::PyXSim();

    /** timeStep and max contact */
    sim->setTimeStep(benchmark::building::params.dt);

}

void setupWorld() {

  // add objects
  auto checkerboard = sim->addCheckerboard(10.0, 400.0, 400.0, 0.1, bo::BOX_SHAPE, 1, -1, bo::GRID);

  // block size
  const float shortLen = benchmark::building::params.shortLen;
  const float longLen = benchmark::building::params.longLen;
  const float heightLen = benchmark::building::params.heightLen;

  // num of blocks
  // numFloor x numBase + numFloor x (numWall x 2 + 1)
  const int numFloor = benchmark::building::params.numFloor;
  const int numBase = benchmark::building::params.numBase;
  const int numWall = numBase / 2;

  for(int i = 0; i < numFloor; i++) {
    // i floor
    for(int j = 0; j < numBase; j++) {
      // base
      auto base = sim->addBox(shortLen, longLen + 0.05, heightLen, 10.0,0,0);
      base->setPosition(j * longLen, i * heightLen * 2 + 0.05, 0);
      objList.push_back(base);
    }

    for(int j = 0; j < numWall; j++) {
      // right wall
      auto wall = sim->addBox(longLen, shortLen, heightLen, 10.0,0,0);
      wall->setPosition(j * longLen * 2 + 0.1, i * heightLen * 2 + 0.15, -0.5 * longLen);
      objList.push_back(wall);
    }

    for(int j = 0; j < numWall - 1; j++) {
      // left wall
      auto wall = sim->addBox(longLen, shortLen, heightLen, 10.0,0,0);
      wall->setPosition(j * longLen * 2 + 0.3, i * heightLen * 2 + 0.15, 0.5 * longLen);
      objList.push_back(wall);
    }

    // first wall on left
    auto wall1 = sim->addBox(longLen, shortLen, heightLen, 10.0,0,0);
    wall1->setPosition(0.1, i * heightLen * 2 + 0.15, 0.5 * longLen);
    objList.push_back(wall1);

    // last wall on left
    auto wall2 = sim->addBox(longLen, shortLen, heightLen, 10.0,0,0);
    wall2->setPosition((numWall - 1) * longLen * 2 + 0.1, i * heightLen * 2 + 0.15, 0.5 * longLen);
    objList.push_back(wall2);
  }

  // gravity
  sim->setGravity({0,  benchmark::building::params.g,0});
}

benchmark::building::Data simulationLoop() {

  // data
  benchmark::building::Data data;
  data.setN(unsigned(benchmark::building::params.T / benchmark::building::params.dt));

  // timer start
  StopWatch watch;
  watch.start();

  int i;
  for(i = 0; i < (int) (benchmark::building::params.T / benchmark::building::params.dt); i++) {
    // num contacts
    data.numContacts.push_back(sim->getWorldNumContacts());

    if(benchmark::building::options.collapse && objList.back()->getPosition()[2] <
        benchmark::building::params.heightLen * (benchmark::building::params.numFloor - 1) * 2) {
      // break if the building collapses
      RAIINFO("building collapsed after " << i << " steps = " << i * benchmark::building::params.dt << " sec!")
      break;
    }

    sim->integrate();
  }

  data.time = watch.measure();
  data.step = i;
  return data;
}

int main(int argc, const char* argv[]) {

  benchmark::building::addDescToOption(desc);
  benchmark::building::getOptionsFromArg(argc, argv, desc);
  benchmark::building::getParamsFromYAML(benchmark::building::getYamlpath().c_str(),
                                         benchmark::PHYSX);

  setupSimulation();
  setupWorld();

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: RAI" << std::endl
                << "GUI      : " << benchmark::building::options.gui << std::endl
                << "ERP      : " << benchmark::building::options.erpYN << std::endl
                << "Timestep : " << benchmark::building::params.dt << std::endl
                << "Num block: " << objList.size() << std::endl
                << "-----------------------"
  )

  benchmark::building::Data data = simulationLoop();

  if(benchmark::building::options.csv)
    benchmark::building::printCSV(benchmark::building::getCSVpath(),
                                  "simName",
                                  "solverName",
                                  "detectorName",
                                  "integratorName",
                                  data.time,
                                  data.step,
                                  data.computeMeanContacts());

  RAIINFO(
      std::endl << "Avg. Num Contacts : " << data.computeMeanContacts() << std::endl
                << "CPU time          : " << data.time << std::endl
                << "num steps         : " << data.step << std::endl
                << "speed             : " << data.step / data.time << std::endl
                << "=======================" << std::endl
  )
  return 0;
}
