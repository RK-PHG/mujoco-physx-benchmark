/** 此文件定义MjcSim, 它继承于WorldRG */

#ifndef MUJOCOSIM_WORLD_RG_HPP
#define MUJOCOSIM_WORLD_RG_HPP

#include <raiGraphics/RAI_graphics.hpp>

#include "common/WorldRG.hpp"
#include "MjcWorld.hpp"
#include "UserHandle.hpp"


namespace mujoco_sim {

class MjcSim: public benchmark::WorldRG {

 public:

    /**
  * @brief 带可视化的 MjcSim 构造函数。
  *
  * @param windowWidth 窗口的宽度，单位为像素。
  * @param windowHeight 窗口的高度，单位为像素。
  * @param cms 物理单位转换为像素的比例因子（厘米到屏幕的像素）。
  * @param modelPath 模型文件的路径（XML 格式），描述了仿真中的物理模型。
  * @param keyPath 认证文件的路径，在高版本中已经弃用
  * @param flags 控制仿真行为的标志（通常是位掩码）。
  * @param solver 选择的求解器类型（默认为 PGS）。
  * @param integrator 选择的积分器类型（默认为 EULER）。
  */
  MjcSim(int windowWidth,
         int windowHeight,
         float cms,
         const char *modelPath,
         const char *keyPath,
         int flags,
         SolverOption solver = SOLVER_PGS,
         IntegratorOption integrator = INTEGRATOR_EULER);

    /**
   * @brief 不带可视化的 MjcSim 构造函数。
   *
   * @param modelPath 模型文件的路径（XML 格式），描述了仿真中的物理模型。
   * @param keyPath 认证文件的路径，在高版本中已经弃用
   * @param solver 选择的求解器类型（默认为 PGS）。
   * @param integrator 选择的积分器类型（默认为 EULER）。
   */
  MjcSim(const char *modelPath,
         const char *keyPath,
         SolverOption solver = SOLVER_PGS,
         IntegratorOption integrator = INTEGRATOR_EULER);
  virtual ~MjcSim();

    /**
     * @brief 更新可视化帧。
     *
     * 该方法负责刷新仿真窗口中的可视化内容，以反映最新的仿真状态。
     * 通过调用此方法，可以在每一帧中更新物体的位置、速度和其他属性。
     */
  virtual void updateFrame() override ;

  //////////////////////////
  /// simulation methods ///
  //////////////////////////

    /**
   * @brief 设置无滑移参数。
   *
   * @param maxiter 最大迭代次数，用于解决无滑移约束的算法。
   *
   * 该方法配置无滑移条件下的最大迭代次数，以提高仿真中的接触精度和稳定性。
   * 用户可以根据需要调整该参数以优化仿真性能。
   */
  void setNoSlipParameter(int maxiter);

    /**
   * @brief 设置重力向量。
   *
   * @param gravity 重力向量，表示仿真环境中的重力加速度（单位为米/秒²）。
   *
   * 该方法允许用户设置仿真环境中的重力，以便更真实地模拟物体在重力场中的运动。
   * 重力向量通常以三维向量的形式给出，其中包含 x、y 和 z 方向的重力分量。
   */
  void setGravity(Eigen::Vector3d gravity) override ;

    /**
     * @brief 主循环，用于执行仿真步骤。
     *
     * @param realTimeFactor 实际时间因子，控制仿真速度与真实时间的比例。
     *
     * 该方法负责驱动仿真进程，通过多次调用运动积分和前向运动学方法来更新仿真状态。
     * 用户可以通过调整实际时间因子来控制仿真速度，以便在不同的硬件条件下实现流畅的仿真体验。
     */
  void loop(double realTimeFactor = 1.0);

    /**
   * @brief 进行一次完整的运动积分。
   *
   * 该方法执行一次运动积分，更新所有物体的位置和速度。
   * 它会根据当前的物理状态计算出下一个时刻的状态，并在仿真中应用这些变化。
   */
  void integrate();

    /**
   * @brief 进行一次类型为 integrate1 的运动积分。
   *
   * 该方法是另一种运动积分实现，可能在特定条件下优化计算过程或处理不同的物理特性。
   * 用户可以根据需要选择使用该方法来进行运动积分。
   */
  void integrate1();

    /**
   * @brief 进行一次类型为 integrate2 的运动积分。
   *
   * 该方法是第三种运动积分实现，专注于特定的物理计算或性能优化。
   * 与其他积分方法不同，可能使用不同的算法或步骤来更新仿真状态。
   */
  void integrate2();

    /**
   * @brief 进行前向运动学计算。
   *
   * 该方法用于更新物体的姿态和位置，确保所有物体的状态与当前的运动模型保持一致。
   * 前向运动学计算通常在运动积分之后调用，以更新仿真中的视觉效果和物理状态。
   */
  void forwardKinematics();

    /**
     * @brief 重置 MuJoCo 数据。
     *
     * 该方法用于重置 MuJoCo 仿真数据，通常在开始新一轮仿真时调用。
     * 重置后，所有物体的位置、速度和其他状态信息将被初始化为默认值。
     */
  void resetSimulation();

    /**
   * @brief 设置固定的仿真时间步长。
   *
   * @param timeStep 时间步长大小（单位：秒）。
   *
   * 该方法允许用户设置仿真过程中的时间步长，以控制每次仿真更新的时间间隔。
   * 选择合适的时间步长对于确保仿真的稳定性和准确性至关重要。
   */
  void setTimeStep(double timeStep);

    /**
   * @brief 设置接触标志。
   *
   * @param flagYN 如果设置为 false，则禁用世界中的接触。
   *
   * 该方法用于启用或禁用物体之间的接触检测。禁用接触检测可以用于测试或仿真中需要避免物体交互的情况。
   */
  void setWorldContactFlag(bool flagYN);

    /**
    * @brief 设置热启动标志。
    *
    * @param flagYN 如果设置为 false，则禁用热启动。
    *
    * 该方法控制是否在仿真开始时使用热启动技术，这有助于在每次仿真迭代中快速收敛，尤其是在接触问题频繁的情况下。
    */
  void setWarmStartFlag(bool flagYN);

    /**
     * @brief 设置求解器参数。
     *
     * @param solverMaxIteration 求解器最大迭代次数，默认值为 100。
     * @param solverTolerance    求解器容忍度，默认值为 1e-8。
     *
     * 该方法允许用户自定义求解器的行为，通过设置最大迭代次数和容忍度来调整求解过程的精度和效率。
     */
  void setSolverParameter(int solverMaxIteration, double solverTolerance);



  benchmark::SingleBodyHandle getSingleBodyHandle(int index);
  int getWorldNumContacts() override ;
  int getNumObject() override ;

  const Eigen::Map<Eigen::Matrix<double, 3, 1>> getLinearMomentumInCartesianSpace();
  double getTotalMass();

  /**
   * Get energy of the world.
   * Note. Gravity should be set to world with same value of param &gravity
   *
   * @param gravity this parameter is deprecated!
   * @return returm world energy
   */
  double getEnergy(const benchmark::Vec<3> &gravity);

  /**
  * Get kinetic energy of the robot
  *
  * @return        Kinetic energy of the robot
  */
  double getKineticEnergy();

  /**
   * Get potential energy of the robot
   *
   * @param gravity Gravitational acceleration
   * @return        Potential energy of the robot
   */
  double getPotentialEnergy(const benchmark::Vec<3> &gravity);

  /// the functions below are articulated system related.
  /// ===================================

    /**
   * @brief 获取广义坐标（关节位置）。
   *
   * @return 返回一个包含当前关节坐标的向量。
   *
   * 对于浮动基机器人，返回的向量包含线性位置、基座的四元数表示和关节角度。
   * 对于固定基机器人，返回的向量仅包含关节角度。
   */
  const EigenVec getGeneralizedCoordinate();

    /**
   * @brief 获取广义速度（关节速度）。
   *
   * @return 返回一个包含当前关节速度的向量。
   *
   * 对于浮动基机器人，返回的向量包含线性速度、角速度和关节速度。
   * 对于固定基机器人，返回的向量仅包含关节速度。
   */
  const EigenVec getGeneralizedVelocity();

    /**
    * @brief 设置广义坐标（关节位置）。
    *
    * @param jointState 包含新的关节状态的向量。
    *
    * 对于浮动基机器人，输入的向量应包含线性位置、基座的四元数表示和关节角度。
    * 对于固定基机器人，输入的向量应仅包含关节角度。
    */
  void setGeneralizedCoordinate(const Eigen::VectorXd &jointState);

    /**
   * @brief 设置广义速度（关节速度）。
   *
   * @param jointVel 包含新的关节速度的向量。
   *
   * 对于浮动基机器人，输入的向量应包含线性速度、角速度和关节速度。
   * 对于固定基机器人，输入的向量应仅包含关节速度。
   */
  void setGeneralizedVelocity(const Eigen::VectorXd &jointVel);

    /**
   * @brief 设置广义坐标（关节位置）使用初始化列表。
   *
   * @param jointState 初始化列表，包含新的关节状态。
   */
  void setGeneralizedCoordinate(std::initializer_list<double> jointState);

    /**
   * @brief 设置广义速度（关节速度）使用初始化列表。
   *
   * @param jointVel 初始化列表，包含新的关节速度。
   */
  void setGeneralizedVelocity(std::initializer_list<double> jointVel);

    /**
   * @brief 设置广义力。
   *
   * @param tau 初始化列表，包含新的广义力。
   *
   * 此方法用于设置施加在关节上的力或扭矩。
   */
  void setGeneralizedForce(std::initializer_list<double> tau);

    /**
   * @brief 设置广义力。
   *
   * @param tau 包含新的广义力的向量。
   */
  void setGeneralizedForce(const Eigen::VectorXd &tau);

    /**
   * @brief 获取当前的状态（广义坐标和广义速度）。
   *
   * @param genco 用于存储当前广义坐标的向量。
   * @param genvel 用于存储当前广义速度的向量。
   *
   * 此方法将当前的广义坐标和速度复制到传入的参数中。
   */
  void getState(Eigen::VectorXd &genco, Eigen::VectorXd &genvel);

    /**
   * @brief 设置当前的状态（广义坐标和广义速度）。
   *
   * @param genco 包含新的广义坐标的向量。
   * @param genvel 包含新的广义速度的向量。
   *
   * 此方法将新的广义坐标和速度应用于系统。
   */
  void setState(const Eigen::VectorXd &genco, const Eigen::VectorXd &genvel);

    /**
   * @brief 获取当前施加的广义力。
   *
   * @return 返回一个包含当前施加的广义力的向量。
   */
  const EigenVec getGeneralizedForce();

    /**
   * @brief 获取系统的自由度（DOF）。
   *
   * @return 返回系统的自由度数。
   */
  int getDOF();

    /**
   * @brief 获取状态的维度。
   *
   * @return 返回状态向量的维度。
   *
   * 状态维度通常是广义坐标和广义速度的总和。
   */
  int getStateDimension();
  /// ===================================

 private:
  void initFromModel();

  /// deprecated functions
  void setERP(double erp, double erp2, double frictionErp) override ;
  void integrate(double dt) override ;
  void integrate1(double dt) override;
  void integrate2(double dt) override;
  void loop(double dt, double realTimeFactor) override ;

  //////////////////////////////////
  /// adding or removing objects ///
  //////////World////////////////////////

  /// note: use last two parameters as bodyId and geomId rather than collisionGroup and collisionMask
  benchmark::SingleBodyHandle addSphere(double radius,
                                        double mass,
                                        int bodyId,
                                        int geomId) override ;

  benchmark::SingleBodyHandle addBox(double xLength,
                                     double yLength,
                                     double zLength,
                                     double mass,
                                     int bodyId,
                                     int geomId) override ;

  benchmark::SingleBodyHandle addCylinder(double radius,
                                          double height,
                                          double mass,
                                          int bodyId,
                                          int geomId) override ;

  benchmark::SingleBodyHandle addCheckerboard(double gridSize,
                                              double xLength,
                                              double yLength,
                                              double reflectanceI,
                                              bo::CheckerboardShape shape,
                                              int bodyId,
                                              int geomId,
                                              int flags = 0) override ;

  benchmark::SingleBodyHandle addCapsule(double radius,
                                         double height,
                                         double mass,
                                         int bodyId,
                                         int geomid) override ;

  /** MjcWorld */
  mujoco_sim::MjcWorld world_;

  double timeStep_ = 0.01;
};

} // mujoco_sim

#endif //MUJOCOSIM_WORLD_RG_HPP
