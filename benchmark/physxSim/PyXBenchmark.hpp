/** 此文件用于配置 physx 的 benchmark 环境, 解析命令行环境  */

#ifndef SIM_BENCHMARK_PYXBENCHMARK_HPP
#define SIM_BENCHMARK_PYXBENCHMARK_HPP

#include <raiCommon/rai_utils.hpp>
#include <boost/program_options.hpp>
#include "physxSim/src/PyXWorld.hpp"
#include "BenchmarkTest.hpp"

namespace po = boost::program_options;

struct Option{

    const benchmark::Simulator simulator = benchmark::PHYSX;
    physx_sim::SolverOption solverOption = physx_sim::SOLVER_PGS;
    const std::string simName = "PHYSX";
    std::string solverName = "DANTZIG";
    std::string detectorName = "BULLET";
    std::string integratorName = "PHYSX";

};

Option options;

void addDescToOption(po::options_description &desc) {
    desc.add_options()
            ("solver", po::value<std::string>(), "constraint solver type (pgs / cg / newton)")
            ("integrator", po::value<std::string>(), "integrator type (euler / rk4)")
            ("noslip", "no-slip solver")
            ;
}

/**
 * get option or parameter from arguments
 *
 * @param argc
 * @param argv
 * @param desc
 */
void getOptionsFromArg(int argc, const char **argv, po::options_description &desc) {

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

}

#endif //SIM_BENCHMARK_PYXBENCHMARK_HPP



