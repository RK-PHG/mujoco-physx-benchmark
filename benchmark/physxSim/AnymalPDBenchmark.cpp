#include <PyXSim.hpp>

#include "AnymalPDBenchmark.hpp"
#include "AnymalEnergyBenchmark.hpp"
#include "raiCommon/utils/StopWatch.hpp"

physx_sim::PyXSim *sim;
po::options_description desc;
std::vector<physx_sim::object::PyXArticulatedSystem*> anymals;

void setupSimulation() {
    sim = new physx_sim::PyXSim();
    sim->setTimeStep(benchmark::anymal::params.dt);
}

void resetWorld() {
  auto checkerboard = sim->addCheckerboard(2, 100, 100, 0.1, bo::BOX_SHAPE, 1, -1, bo::GRID);
  checkerboard->setFrictionCoefficient(0.8);

  for(int i = 0; i < benchmark::anymal::options.numRow; i++) {
    for(int j = 0; j < benchmark::anymal::options.numRow; j++) {
      auto anymal = sim->world_->loadModel(
          benchmark::anymal::freedrop::getURDFpath()
      );
      anymal->setGeneralizedCoordinate(
          {i * 2.0,
           j * 2.0,
           benchmark::anymal::params.H,
           benchmark::anymal::params.baseQuat[0],
           benchmark::anymal::params.baseQuat[1],
           benchmark::anymal::params.baseQuat[2],
           benchmark::anymal::params.baseQuat[3],
           benchmark::anymal::params.dartjointPos[0],
           benchmark::anymal::params.dartjointPos[1],
           benchmark::anymal::params.dartjointPos[2],
           benchmark::anymal::params.dartjointPos[3],
           benchmark::anymal::params.dartjointPos[4],
           benchmark::anymal::params.dartjointPos[5],
           benchmark::anymal::params.dartjointPos[6],
           benchmark::anymal::params.dartjointPos[7],
           benchmark::anymal::params.dartjointPos[8],
           benchmark::anymal::params.dartjointPos[9],
           benchmark::anymal::params.dartjointPos[10],
           benchmark::anymal::params.dartjointPos[11]
          });
      anymal->setGeneralizedVelocity(Eigen::VectorXd::Zero(anymal->getDOF()));
      anymal->setGeneralizedForce(Eigen::VectorXd::Zero(anymal->getDOF()));
      anymals.push_back(anymal);
    }
  }

  sim->setGravity({0, 0, benchmark::anymal::params.g});
}

void simulationLoop() {
  Eigen::VectorXd jointNominalConfig(19);
  Eigen::VectorXd jointState(18), jointVel(18), jointForce(18);
  const double kp = benchmark::anymal::params.kp;
  const double kd = benchmark::anymal::params.kd;

  jointNominalConfig
      <<
      0,
      0,
      benchmark::anymal::params.H,
      benchmark::anymal::params.baseQuat[0],
      benchmark::anymal::params.baseQuat[1],
      benchmark::anymal::params.baseQuat[2],
      benchmark::anymal::params.baseQuat[3],
      benchmark::anymal::params.dartjointPos[0],
      benchmark::anymal::params.dartjointPos[1],
      benchmark::anymal::params.dartjointPos[2],
      benchmark::anymal::params.dartjointPos[3],
      benchmark::anymal::params.dartjointPos[4],
      benchmark::anymal::params.dartjointPos[5],
      benchmark::anymal::params.dartjointPos[6],
      benchmark::anymal::params.dartjointPos[7],
      benchmark::anymal::params.dartjointPos[8],
      benchmark::anymal::params.dartjointPos[9],
      benchmark::anymal::params.dartjointPos[10],
      benchmark::anymal::params.dartjointPos[11];

    // no gui
    StopWatch watch;
    watch.start();
    for(int t = 0; t < (int)(benchmark::anymal::params.T / benchmark::anymal::params.dt); t++) {
      for(int i = 0; i < anymals.size(); i++) {
        jointState = anymals[i]->getGeneralizedCoordinate();
        jointVel = anymals[i]->getGeneralizedVelocity();
        jointForce = anymals[i]->getGeneralizedForce();

        jointForce = kp * (jointNominalConfig - jointState).tail(18) - kd * jointVel;
        jointForce.head(6).setZero();
        anymals[i]->setGeneralizedForce(jointForce);
      }
      sim->integrate();
    }

    double time = watch.measure();

    // print to screen
    std::cout<<"time taken for "
             << (int) (benchmark::anymal::params.T / benchmark::anymal::params.dt)
             << " steps "<< time <<"s \n";

    if(benchmark::anymal::options.csv)
      benchmark::anymal::printCSV(benchmark::anymal::getCSVpath(benchmark::anymal::options.feedback),
                                  "simName",
                                  "solverName",
                                  "detectorName",
                                  "integratorName",
                                  benchmark::anymal::options.numRow,
                                  time);

}

int main(int argc, const char* argv[]) {

  benchmark::anymal::addDescToOption(desc);
  benchmark::anymal::getOptionsFromArg(argc, argv, desc);


  benchmark::anymal::getParamsFromYAML(benchmark::anymal::getYamlpath().c_str(),
                                       benchmark::DART);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: Physx" << std::endl
//                << "GUI      : " << benchmark::anymal::options.gui << std::endl
//                << "Row      : " << benchmark::anymal::options.numRow << std::endl
//                << "Feedback : " << benchmark::anymal::options.feedback << std::endl
//                << "Solver   : " << "soverOption"<< std::endl
                << "-----------------------"
  )

  setupSimulation();
  resetWorld();
  simulationLoop();

  RAIINFO(
      std::endl << "-----------------------" << std::endl
                << "Contacts : " << sim->getWorldNumContacts() << std::endl
                << "======================="
  )
  return 0;
}