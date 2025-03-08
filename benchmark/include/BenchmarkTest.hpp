/** 这个文件定义了 benchmarkTest 的一些基本属性 */

#ifndef BENCHMARK_BENCHMARK_HPP
#define BENCHMARK_BENCHMARK_HPP

#include <string>
#include <Eigen/Geometry>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

namespace benchmark {

/** simulators */
enum Simulator {
  RAI     = (1 << 0),
  BULLET  = (1 << 1),
  ODE     = (1 << 2),
  MUJOCO  = (1 << 3),
  DART    = (1 << 4),
  PHYSX   = (1 << 5)
};

/**
 * options for benchmark test
 */
struct Option {
  // gui on/off
  bool gui = true;

  // save video
  bool saveVideo = false;

  // print log on/off
  bool log = false;

  // print csv on/off
  bool csv = true;
  std::string csvName = "log.csv";

  // plot (show plot if exists)
  bool plot = false;
};

/**
 *  定义了运行测试的一些选项
 */
void addDescToOption(po::options_description &desc) {
  desc.add_options()
      ("help", "produce help message")  // help
      ("nogui", "no visualization")     // gui 选项
      ("log", "create log files")       // log 选项
      ("video", "save a video file (only available when gui is on)")    // 录制视频
      ("csv", po::value<std::string>(), "name of csv file (save csv when this option is given)") // 打印csv
      ;
}

} // benchmark

#endif //BENCHMARK_BENCHMARK_HPP
