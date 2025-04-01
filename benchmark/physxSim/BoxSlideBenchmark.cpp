#include "PyXSim.hpp"
#include "BoxSlideBenchmark.hpp"

// sim
physx_sim::PyXSim *sim;

// objects
std::vector<benchmark::SingleBodyHandle> objList;

double slideLength = 0;


void setupSimulation() {

    if (benchmark::boxSlide::options.gui)
        sim = new physx_sim::PyXSim(800, 600, 0.5);
    else
        sim = new physx_sim::PyXSim();

    // timestep
    sim->setTimeStep(benchmark::boxSlide::options.dt);
}

void setupWorld() {

    // add objects
    auto checkerboard = sim->addCheckerboard(5.0, 100.0, 100.0, 0.1, bo::BOX_SHAPE, 1, -1, bo::GRID);
    checkerboard->setFrictionCoefficient(benchmark::boxSlide::params.groundMu);

    auto box = sim->addBox(5, 5, 3, 0.8, 0, 0);
    box->setPosition(0, 0, 3);
    box->setVelocity(0,benchmark::boxSlide::params.v,0,0,0,0);
    box->setFrictionCoefficient(benchmark::boxSlide::params.boxMu);
    objList.push_back(box);

    if(benchmark::boxSlide::options.gui)
        box.visual()[0]->setColor({0,1.0,0});

    // gravity
    sim->setGravity({0, 0, benchmark::boxSlide::params.g});

    if(benchmark::boxSlide::options.gui) {
        sim->setLightPosition((float)benchmark::boxSlide::params.lightPosition[0],
                              (float)benchmark::boxSlide::params.lightPosition[1],
                              (float)benchmark::boxSlide::params.lightPosition[2]);
        sim->cameraFollowObject(checkerboard, {30, 0, 15});
    }
}

double simulationLoop(bool timer = true, bool error = true) {

    // timer start
    StopWatch watch;
    if(timer)
        watch.start();

    for(int i = 0; i < (int) (benchmark::boxSlide::params.T / benchmark::boxSlide::options.dt); i++) {

        // gui
        if(benchmark::boxSlide::options.gui && !sim->visualizerLoop(benchmark::boxSlide::options.dt))
            break;

        slideLength =  objList[0]->getPosition()[1];

        // step
        sim->integrate();
    }

    double time = 0;
    if(timer)
        time = watch.measure();
    return time;
}

int main(int argc, const char* argv[]) {

    // trial1: get Error
    setupSimulation();
    setupWorld();
    simulationLoop(false, true);

    double boxMu = benchmark::boxSlide::params.boxMu;
    double groundMu = benchmark::boxSlide::params.groundMu;
    double v = benchmark::boxSlide::params.v;
    double g = - benchmark::boxSlide::params.g;


    RAIINFO(
            std::endl << "理论滑动距离:   " << v*v / (boxMu+groundMu) / g  << std::endl
                      << "实际滑动距离:   " << slideLength << std::endl
                      << "=======================" << std::endl
    )

    delete sim;
    return 0;
}