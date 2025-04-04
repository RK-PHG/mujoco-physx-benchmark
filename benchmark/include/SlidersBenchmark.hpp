#ifndef MUJOCO_ISSAC_BENCHMARK_BOXSLIDEBENCHMARK_HPP
#define MUJOCO_ISSAC_BENCHMARK_BOXSLIDEBENCHMARK_HPP

#include "raiGraphics/RAI_graphics.hpp"
#include <yaml-cpp/yaml.h>
#include <boost/program_options.hpp>

#include "BenchmarkTest.hpp"

namespace po = boost::program_options;
namespace ru = rai::Utils;

namespace benchmark::slopeSlider {

    std::string getYamlPath();

    struct Option: benchmark::Option {

        double dt = 0.01;


    } options;

    struct Parameter {

        double lightPosition[3] = {30.0, 0, 10.0};

        double T = 5.0;

        double erp = 0.2;

        double mass = 1;

        double g = -9.81;

        double v = 10.0; // m,s

        double f = 0.2;

        double groundMu = 0.2;

        double boxMu = 0.2;

    } params;


} // benchmark::rolling


#endif //MUJOCO_ISSAC_BENCHMARK_BOXSLIDEBENCHMARK_HPP
