#ifndef BENCHMARK_THOUSANDBENCHMARK_HPP
#define BENCHMARK_THOUSANDBENCHMARK_HPP

#include <yaml-cpp/yaml.h>
#include <boost/program_options.hpp>

#include "BenchmarkTest.hpp"

/**
 * Thousand test is for testing hard-contact constraint.
 * The test focuses on:
 *
 * 1. compliance of hard-contact constraint
 *
 * Please read docs for more details
 */

/** 两个简称 */
namespace po = boost::program_options;
namespace ru = rai::Utils;

namespace benchmark::sixsixsix {

    struct Option: benchmark::Option {

        bool erpYN = false;

        double dt = 0.01;

        double T = 10.0;

        bool elasticCollision = false;
    };
    Option options;

    struct Parameter {

        // sim properties
        double lightPosition[3] = {30.0, 0, 10.0};

        // solver parameters
        double erp = 0.2;
        double erp2 = 0.1;              // for bullet
        double erpFriction = 0.01;      // for bullet

        // simulation parameters
        /// (num of object) = n * n * n
        int n = 8;

        double gap = 0.21;
        double perturbation = 0.001;
        double H = 1.101;

        double ballR = 0.1;     // radius of ball
        double ballM = 10;    // mass of ball

        int randomSeed = 42;

        double g = -9.81;
    };
    Parameter params;

    struct Data {
        void setN(int n) {
            Data::n = n;
            error.reserve(n);
        }

        double computeError() {
            Eigen::MatrixXd errorSq(n, 1); // energy error or penetration error

            for(int i = 0; i < n; i++) {
                errorSq(i, 0) = error[i];
            }

            if(options.plot) {
                Eigen::MatrixXd tdata(n, 1);        // time

                for(int i = 0; i < n; i++) {
                    tdata(i, 0) = i * benchmark::sixsixsix::options.dt;
                }

                rai::Utils::Graph::FigProp2D figure1properties("time", "squared error", "squared error");
                rai::Utils::graph->figure(1, figure1properties);
                rai::Utils::graph->appendData(1, tdata.data(), errorSq.data(), n, "error sq");
                rai::Utils::graph->drawFigure(1);
                rai::Utils::graph->waitForEnter();
            }

            return errorSq.mean();
        }

        // data list
        std::vector<double> error;

        // num data
        int n = 0;
    };
    Data data;

/**
 * get XML file path for Mujoco
 *
 * @return urdf path in string
 */
    std::string getBulletBallPath() {

        std::string ballpath(RESOURCEPATH);
        ballpath += "/benchmark/666-benchmark/bullet/ball.urdf";

        return ballpath;
    }

/**
 * get XML file path for Mujoco
 *
 * @return urdf path in string
 */
    std::string getBulletPlanePath() {

        std::string planepath(RESOURCEPATH);
        planepath += "/benchmark/666-benchmark/bullet/plane.urdf";

        return planepath;
    }

/**
 * get XML file path for Mujoco
 *
 * @return xml path in string
 */
    std::string getMujocoXMLpath() {

        int ncubic = params.n * params.n * params.n;

        std::string xmlPath(RESOURCEPATH);
        xmlPath += "/benchmark/666-benchmark/mujoco/sphere" + std::to_string(ncubic) + ".xml";

        return xmlPath;
    }

/**
 * get YAML file path for parameter
 *
 * @return yaml path in string
 */
    std::string getYamlpath() {

        std::string yamlPath(BENCHMARKYAMLPATH);

        if(options.elasticCollision)
            yamlPath += "/666-elastic.yaml";
        else
            yamlPath += "/666.yaml";

        return yamlPath;
    }

/**
 * get log file directory path of test result
 *
 * @param erp
 * @return log directory path in string
 */
    std::string getCSVpath() {
        std::string logPath(DATAPATH);
        logPath += "/666/" + options.csvName;
        return logPath;
    }

/**
 * add options to desc
 *
 * @param desc
 */
    void addDescToOption(po::options_description &desc) {
        benchmark::addDescToOption(desc);
        desc.add_options()
                ("erp-on", po::value<bool>(), "erp on (true / false)")
                ("dt", po::value<double>(), "time step for simulation (e.g. 0.01)")
                ("T", po::value<double>(), "simulation time (e.g. 15)")
                ("plot", "plot energy and pen error")
                ("elastic", "if elastic collision")
                ;
    }

/**
 * get options from arguments
 *
 * @param argc
 * @param argv
 * @param desc
 */
    void getOptionsFromArg(int argc, const char *argv[], po::options_description &desc) {

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);

        // help option
        if(vm.count("help")) {
            exit(0);
        }

        // csv
        if(vm.count("csv")) {
            options.csv = true;
            options.csvName = vm["csv"].as<std::string>();
        }

        // nogui option
        if(vm.count("nogui")) {
            options.gui = false;
        }

        // save video
        if(vm.count("video")) {
            options.saveVideo = true;
        }

        // dt option
        if(vm.count("dt")) {
            options.dt = vm["dt"].as<double>();
        }

        // T option
        if(vm.count("T")) {
            options.T = vm["T"].as<double>();
        }

        // erp
        if(vm.count("erp-on")) {
            if(vm["erp-on"].as<bool>()) {
                options.erpYN = true;
            } else {
                options.erpYN = false;
            }
        }

        // elastic collision
        if(vm.count("elastic")) {
            options.elasticCollision = true;
        }

        // plot
        if(vm.count("plot")) {
            options.plot = true;
        }
    }

/**
 * get params from YAML
 *
 * @param yamlfile
 */
    void getParamsFromYAML(const char *yamlfile, benchmark::Simulator simulator) {

        /// parameters from yaml
        YAML::Node yaml = YAML::LoadFile(yamlfile);

        // sim properties
        YAML::Node props = yaml["sim_properties"];
        params.lightPosition[0] = props["light_position"].as<std::vector<double>>()[0];
        params.lightPosition[1] = props["light_position"].as<std::vector<double>>()[1];
        params.lightPosition[2] = props["light_position"].as<std::vector<double>>()[2];

        // simulation constants
        YAML::Node constant = yaml["constant"];
        params.ballR = constant["ballR"].as<double>();
        params.ballM = constant["ballM"].as<double>();
        params.gap = constant["gap"].as<double>();
        params.perturbation = constant["perturbation"].as<double>();
        params.H = constant["H"].as<double>();
        params.n = constant["n"].as<int>();
        params.g = constant["g"].as<double>();

        // solver parameters
        YAML::Node solver_params = yaml["solver_params"];

        switch (simulator) {
            case benchmark::RAI:
                params.erp = solver_params["raiSim"]["erp"].as<double>();
                break;
            case benchmark::BULLET:
                params.erp = solver_params["bullet"]["erp"].as<double>();
                params.erp2 = solver_params["bullet"]["erp2"].as<double>();
                params.erpFriction = solver_params["bullet"]["erp_friction"].as<double>();
                break;
            case benchmark::ODE:
                params.erp = solver_params["ode"]["erp"].as<double>();
                break;
            case benchmark::MUJOCO:
                break;
            case benchmark::DART:
                break;
            case benchmark::PHYSX:
                break;
            default:
            RAIFATAL("invalid simulator value")
        }
    }

    void printCSV(std::string filePath,
                  std::string sim,
                  std::string solver,
                  std::string detector,
                  std::string integrator,
                  double time,
                  double error) {
        std::ofstream myfile;
        myfile.open (filePath, std::ios_base::app);

        myfile
                << sim << ","
                << solver << ","
                << detector << ","
                << integrator << ","
                << options.erpYN << ","
                << options.elasticCollision << ","
                << options.dt << ","
                << error << ","
                << time
                << std::endl;

        myfile.close();
    }

} // benchmark::sixsixsix


#endif //BENCHMARK_THOUSANDBENCHMARK_HPP
