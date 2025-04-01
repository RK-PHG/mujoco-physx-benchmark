#include "PyXSim.hpp"
#include "SlopeSliderBenchmark.hpp"

// sim
physx_sim::PyXSim *sim;

// objects
std::vector<benchmark::SingleBodyHandle> objList;

double slideLength = 0;


void setupSimulation() {

    if (benchmark::slopeSlider::options.gui)
        sim = new physx_sim::PyXSim(800, 600, 0.5);
    else
        sim = new physx_sim::PyXSim();

    // timestep
    sim->setTimeStep(benchmark::slopeSlider::options.dt);
}

void setupWorld() {

    // add objects
    auto checkerboard = sim->addCheckerboard(5.0, 100.0, 100.0, 0.1, bo::BOX_SHAPE, 1, -1, bo::GRID);
    checkerboard->setFrictionCoefficient(benchmark::slopeSlider::params.groundMu);

    // box1
    auto box = sim->addBox(10, 40, 2, 0.8, 0, 0);
    box->setPosition(0, 0, 2);
    box->setVelocity(0,benchmark::slopeSlider::params.v,0,0,0,0);
    box->setFrictionCoefficient(benchmark::slopeSlider::params.boxMu);
    objList.push_back(box);

    // box2
    auto box2 = sim->addBox(10, 5, 2, 0.8, 0, 0);
    box2->setPosition(0, 10, 6);
    box2->setFrictionCoefficient(benchmark::slopeSlider::params.boxMu);
    objList.push_back(box2);

    if(benchmark::slopeSlider::options.gui)
        box.visual()[0]->setColor({0,1.0,0});

    // gravity
    sim->setGravity({0, 0, benchmark::slopeSlider::params.g});

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

        // gui
        if(benchmark::slopeSlider::options.gui && !sim->visualizerLoop(benchmark::slopeSlider::options.dt))
            break;

        slideLength =  objList[1]->getPosition()[1];

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

    double l = 0.5 * v * v / (p1 * p1) * p3 / p2 / (-benchmark::slopeSlider::params.g);

    RAIINFO(
            std::endl << "理论滑动距离:   " << l  << std::endl
                      << "实际滑动距离:   " << slideLength - 10 << std::endl
                      << "=======================" << std::endl
    )

    delete sim;
    return 0;
}