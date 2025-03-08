/** 此文件用于实现 WorldInterface 接口 */

#ifndef BENCHMARK_WORLD_HPP
#define BENCHMARK_WORLD_HPP

#include <mujoco/mujoco.h>

#include "common/Configure.hpp"
#include "common/interface/WorldInterface.hpp"

#include "object/MjcSphere.hpp"
#include "object/MjcBox.hpp"
#include "object/MjcCapsule.hpp"
#include "object/MjcCheckerBoard.hpp"
#include "object/MjcCylinder.hpp"


/** mujoco_sim namespace */
namespace mujoco_sim {

/** 列向量 */
typedef Eigen::Map<Eigen::Matrix<double, -1, 1> > EigenVec;

/** 矩阵 */
typedef Eigen::Map<Eigen::Matrix<double, -1, -1> > EigenMat;

/** 求解器选项 */
enum SolverOption {
  SOLVER_PGS,
  SOLVER_CG,
  SOLVER_NEWTON
};

/** 积分器选项 */
enum IntegratorOption {
  /** 欧拉积分法，计算速度快，但精度较低 */
  INTEGRATOR_EULER,
  /** 四阶龙格-库塔法，精度较高，但计算代价较大 */
  INTEGRATOR_RK4
};

/** 接触点信息 */
struct Single3DContactProblem {
  Single3DContactProblem(const double x, const double y, const double z) {
    point_ = {x, y, z};
  };
  Eigen::Vector3d point_;
  Eigen::Vector3d normal_;
  double force_;
};

/** class MjcWorld */
class MjcWorld: public benchmark::WorldInterface {

  friend class MjcSim;
 public:
  MjcWorld(const char *modelPath,
           const char *keyPath,
           SolverOption solver,
           IntegratorOption integrator);
  virtual ~MjcWorld();


  /**
   * Getter for collision problems.
   * @return    std vector pointer contains collision problems.
   */
  const std::vector<Single3DContactProblem> *getCollisionProblem() const;

  /// note: use last two parameters as bodyId and geomId rather than collisionGroup and collisionMask
  object::MjcSphere *addSphere(double radius,
                               double mass,
                               int bodyId,
                               int geomId) override ;

  /** 添加盒体 */
  object::MjcBox *addBox(double xLength,
                         double yLength,
                         double zLength,
                         double mass,
                         int bodyId,
                         int geomId) override;

  /** 添加棋盘 */
  object::MjcCheckerBoard *addCheckerboard(double gridSize,
                                           double xLength,
                                           double yLength,
                                           double reflectanceI,
                                           bo::CheckerboardShape shape,
                                           int bodyId,
                                           int geomId) override;

    /**
   * @brief 向模拟中添加一个胶囊体（Capsule）对象。
   *
   * @param radius 胶囊体的半径。
   * @param height 胶囊体的高度。
   * @param mass 胶囊体的质量。
   * @param bodyId 与胶囊体关联的刚体 ID。
   * @param geomId 与胶囊体关联的几何体 ID。
   *
   * @return 返回指向添加的 `MjcCapsule` 对象的指针。
   */
  object::MjcCapsule *addCapsule(double radius,
                                 double height,
                                 double mass,
                                 int bodyId,
                                 int geomId) override;

    /**
   * @brief 向模拟中添加一个圆柱体（Cylinder）对象。
   *
   * @param radius 圆柱体的半径。
   * @param height 圆柱体的高度。
   * @param mass 圆柱体的质量。
   * @param bodyId 与圆柱体关联的刚体 ID。
   * @param geomId 与圆柱体关联的几何体 ID。
   *
   * @return 返回指向添加的 `MjcCylinder` 对象的指针。
   */
  object::MjcCylinder *addCylinder(double radius,
                                   double height,
                                   double mass,
                                   int bodyId,
                                   int geomId) override;
    /**
   * @brief 设置模拟中的重力加速度。
   *
   * @param gravity 重力加速度向量，类型为 `benchmark::Vec<3>`。
   */
  void setGravity(const benchmark::Vec<3> &gravity) override ;

    /**
   * @brief 设置无滑参数（通常用于接触模拟中的摩擦力求解）。
   *
   * @param maxIter 最大迭代次数，用于无滑条件下的接触求解。
   */
  void setNoSlipParameter(int maxIter);

    /**
   * @brief 设置模拟的时间步长。
   *
   * @param timeStep 每步模拟的时间步长（秒）。
   */
  void setTimeStep(double timeStep);

    /**
    * @brief 获取模拟世界的模型数据。
    *
    * @return 返回指向 `mjModel` 的指针，表示当前模拟世界的模型数据。
    */
  mjModel *getWorldModel() const;

    /**
   * @brief 获取模拟世界的动态数据。
   *
   * @return 返回指向 `mjData` 的指针，表示当前模拟世界的动态数据。
   */
  mjData *getWorldData() const;

    /**
    * @brief 获取当前模拟世界中的接触数量。
    *
    * @return 返回当前模拟世界中的接触点数量。
    */
  int getWorldNumContacts();

    /**
    * @brief 获取模拟世界中物体的数量。
    *
    * @return 返回模拟世界中的物体数量。
    */
  int getNumObject() override ;

    /**
     * @brief 获取系统在笛卡尔空间中的线性动量。
     *
     * @return 返回 `Eigen::Map<Eigen::Matrix<double, 3, 1>>`，表示系统的线性动量。
     */
  const Eigen::Map<Eigen::Matrix<double, 3, 1>> getLinearMomentumInCartesianSpace();

    /**
     * @brief 获取系统的总质量。
     *
     * @return 返回系统的总质量。
     */
  double getTotalMass();

    /**
     * @brief 执行系统的正向运动学计算，更新所有关节的位姿。
     */
  void forwardKinematics();

    /**
     * @brief 执行模拟的积分步骤。
     */
  void integrate();

    /**
   * @brief 执行模拟的第一个积分步骤，通常在 `integrate2` 之前调用。
   */
  void integrate1();

    /**
   * @brief 执行模拟的第二个积分步骤，通常在 `integrate1` 之后调用。
   */
  void integrate2();

  /**
   * reset mujoco data
   */
  void resetSimulation();

  /// the functions below are articulated system related.

    /**
    * @brief 获取广义坐标（Generalized Coordinate）。
    *
    * 对于浮动基机器人，返回 [线性位置_W, 基座方向四元数, 关节角度]。
    * 对于固定基机器人，返回 [关节角度]。
    * @return 广义坐标的 Eigen 向量。
    */
  const EigenVec getGeneralizedCoordinate();

    /**
  * @brief 获取广义速度（Generalized Velocity）。
  *
  * 对于浮动基机器人，返回 [线性速度_W, 角速度_W, 关节速度]。
  * @return 广义速度的 Eigen 向量。
  */
  const EigenVec getGeneralizedVelocity();

    /**
   * @brief 设置广义坐标（Generalized Coordinate）。
   *
   * 对于浮动基机器人，输入应包括 [线性位置_W, 基座方向四元数, 关节角度]。
   * 对于固定基机器人，输入仅需 [关节角度]。
   * @param jointState 包含机器人当前关节状态的向量。
   */
  void setGeneralizedCoordinate(const Eigen::VectorXd &jointState);

    /**
   * @brief 设置广义速度（Generalized Velocity）。
   *
   * 对于浮动基机器人，输入应包括 [线性速度_W, 角速度_W, 关节速度]。
   * 对于固定基机器人，输入仅需关节速度。
   * @param jointVel 包含机器人当前关节速度的向量。
   */
  void setGeneralizedVelocity(const Eigen::VectorXd &jointVel);

    /**
   * @brief 使用初始化列表设置广义坐标。
   *
   * 与 setGeneralizedCoordinate(const Eigen::VectorXd&) 功能类似，但使用 initializer_list 作为输入。
   * @param jointState 初始化列表形式的关节状态。
   */
  void setGeneralizedCoordinate(std::initializer_list<double> jointState);

    /**
     * @brief 使用初始化列表设置广义速度。
     *
     * 与 setGeneralizedVelocity(const Eigen::VectorXd&) 功能类似，但使用 initializer_list 作为输入。
     * @param jointVel 初始化列表形式的关节速度。
     */
  void setGeneralizedVelocity(std::initializer_list<double> jointVel);

    /**
   * @brief 使用初始化列表设置广义力（Generalized Force）。
   *
   * @param tau 初始化列表形式的关节力矩/广义力。
   */
  void setGeneralizedForce(std::initializer_list<double> tau);

    /**
   * @brief 使用 Eigen 向量设置广义力（Generalized Force）。
   *
   * @param tau 包含关节力矩/广义力的向量。
   */
  void setGeneralizedForce(const Eigen::VectorXd &tau);

    /**
   * @brief 获取机器人的当前状态，包括广义坐标和广义速度。
   *
   * @param genco 输出的广义坐标向量。
   * @param genvel 输出的广义速度向量。
   */
  void getState(Eigen::VectorXd &genco, Eigen::VectorXd &genvel);

    /**
   * @brief 设置机器人的当前状态，包括广义坐标和广义速度。
   *
   * @param genco 输入的广义坐标向量。
   * @param genvel 输入的广义速度向量。
   */
  void setState(const Eigen::VectorXd &genco, const Eigen::VectorXd &genvel);

    /**
    * @brief 获取广义力（Generalized Force）。
    *
    * @return 广义力的 Eigen 向量。
    */
  const EigenVec getGeneralizedForce();

    /**
   * @brief 获取机器人的自由度（DOF，Degree of Freedom）。
   *
   * 对于浮动基机器人，DOF 包含基座的 6 个自由度加上关节自由度。
   * @return 机器人的自由度。
   */
  int getDOF();

   /** 获取广义坐标维度 */
  int getGeneralizedCoordinateDim();

 private:
    /**
    * @brief 已弃用的积分函数，执行一步模拟并根据时间步长更新系统状态。
    *
    * @deprecated 该函数已弃用，建议使用无参数版本。
    * @param dt 时间步长（秒）。
    */
  void integrate(double dt) override ;

    /**
   * @brief 已弃用的第一个积分步骤，根据时间步长执行初步模拟更新。
   *
   * @deprecated 该函数已弃用，建议使用无参数版本。
   * @param dt 时间步长（秒）。
   */
  void integrate1(double dt) override;

    /**
   * @brief 已弃用的第二个积分步骤，根据时间步长执行进一步的模拟更新。
   *
   * @deprecated 该函数已弃用，建议使用无参数版本。
   * @param dt 时间步长（秒）。
   */
  void integrate2(double dt) override;


  /** mjModel */
  mjModel *worldModel_;

  /** mjData */
  mjData *worldData_;

  /** mjOption */
  mjOption *simOption_;

  /** list */
  std::vector<object::MjcSingleBodyObject*> objectList_;

  /** 接触问题列表 */
  std::vector<Single3DContactProblem> contactProblemList_;

  /** generalized coordinate：广义坐标、广义速度、广义力矩 */
  benchmark::VecDyn generalizedCoordinate_;
  benchmark::VecDyn generalizedVelocity_;
  benchmark::VecDyn generalizedForce_;

  /** linear momentum */
  benchmark::Vec<3> linearMomentum_;

  /** dim */
  int dof_ = 0;
  int dimGenCoord_ = 0;
  int numActuators_ = 0;
};

} // mujoco_sim

#endif //BENCHMARK_WORLD_HPP
