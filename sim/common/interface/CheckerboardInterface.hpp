/** 检查板，地面参考系 */

#ifndef BENCHMARK_CHECKERBOARDINTERFACE_HPP
#define BENCHMARK_CHECKERBOARDINTERFACE_HPP

namespace benchmark {
namespace object {

/** 棋盘类型 */
enum CheckerboardShape {
  PLANE_SHAPE,
  BOX_SHAPE
};

enum CheckerBoardOption {
  GRID = 1<<(1),
};

class CheckerboardInterface {

 protected:
  CheckerboardShape checkerboardShape = PLANE_SHAPE;

};

}
}
#endif
