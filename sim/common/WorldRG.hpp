/** 用于控制场景的显示与渲染 */

#ifndef BENCHMARK_WORLD_RG_HPP
#define BENCHMARK_WORLD_RG_HPP

#include <raiGraphics/RAI_graphics.hpp>
#include "math.hpp"
#include "Configure.hpp"
#include "UserHandle.hpp"

#include "interface/CheckerboardInterface.hpp"

namespace bo = benchmark::object;

namespace benchmark {

/**
 * Options for GUI
 */
enum VisualizerOption {
  NO_BACKGROUND = 1<<(1),           // 不显示背景
  DISABLE_INTERACTION = 1<<(2)      // 禁用用户交互
};

/**
 * WorldRG class is GUI wrapper of dynamics world.
 * Each Sim (BtSim, MjcSim ...) inherit WorldRG
 */
class WorldRG {

 public:

/**
 * 创建一个可视化的 WorldRG 实例
 * @param windowWidth 窗口的宽度
 * @param windowHeight 窗口的高度
 * @param cms 相机移动缩放比例
 * @param flags 可视化选项标志，默认为0
 */
  WorldRG(int windowWidth,
          int windowHeight,
          float cms,
          int flags = 0);

/**
 * 创建一个不带可视化的 WorldRG 实例
 */
  WorldRG() = default;

  virtual ~WorldRG();

/**
 * 执行一次仿真循环
 * @param dt 时间步长
 * @param realTimeFactor 实际时间因子，默认为1.0
 */
  virtual void loop(double dt, double realTimeFactor = 1.0);

/** 开始可视化 */
  virtual void visStart();

/** 结束可视化 */
    virtual void visEnd();

/**
 * 使相机跟随指定的物体
 * @param followingObject 要跟随的单体物体句柄
 * @param relativePosition 相对位置
 */
    virtual void cameraFollowObject(SingleBodyHandle followingObject, Eigen::Vector3d relativePosition);

/**
 * 使相机跟随指定的物体
 * @param followingObject 要跟随的单体物体指针
 * @param relativePosition 相对位置
 */
  virtual void cameraFollowObject(rai_graphics::object::SingleBodyObject *followingObject, Eigen::Vector3d relativePosition);

/**
 * 设置光源的位置
 * @param x 光源在 x 轴上的位置
 * @param y 光源在 y 轴上的位置
 * @param z 光源在 z 轴上的位置
 */
  virtual void setLightPosition(float x, float y, float z);

/**
* 执行可视化循环
* @param dt 时间步长
* @param realTimeFactor 实际时间因子，默认为1.0
* @return 如果可视化循环成功返回 true，否则返回 false
*/
  virtual bool visualizerLoop(double dt, double realTimeFactor = 1.0);

/**
 * 开始录制视频
 * @param dir 视频保存的目录
 * @param fileName 视频文件名
 */
  virtual void startRecordingVideo(std::string dir, std::string fileName);

/**
* 停止录制视频
*/
  virtual void stopRecordingVideo();

    /**
   * 添加一个棋盘对象
   * @param gridSize 棋盘的网格大小
   * @param xLength 棋盘在 x 方向上的长度
   * @param yLength 棋盘在 y 方向上的长度
   * @param reflectanceI 棋盘的反射率
   * @param shape 棋盘的形状，默认为平面形状
   * @param collisionGroup 碰撞组类型，默认为1
   * @param collisionMask 碰撞掩码类型，默认为-1
   * @param flags 其他标志，默认为0
   * @return 返回指向单体物体句柄的指针
   */
  virtual SingleBodyHandle addCheckerboard(double gridSize,
                                           double xLength,
                                           double yLength,
                                           double reflectanceI,
                                           bo::CheckerboardShape shape = bo::PLANE_SHAPE,
                                           benchmark::CollisionGroupType collisionGroup = 1,
                                           benchmark::CollisionGroupType collisionMask = -1,
                                           int flags = 0) = 0;

    /**
   * 添加一个球体对象
   * @param radius 球体的半径
   * @param mass 球体的质量
   * @param collisionGroup 碰撞组类型，默认为1
   * @param collisionMask 碰撞掩码类型，默认为-1
   * @return 返回指向单体物体句柄的指针
   */
  virtual SingleBodyHandle addSphere(double radius,
                                     double mass,
                                     benchmark::CollisionGroupType collisionGroup = 1,
                                     benchmark::CollisionGroupType collisionMask=-1) = 0;

    /**
   * 添加一个立方体对象
   * @param xLength 立方体在 x 方向上的长度
   * @param yLength 立方体在 y 方向上的长度
   * @param zLength 立方体在 z 方向上的长度
   * @param mass 立方体的质量
   * @param collisionGroup 碰撞组类型，默认为1
   * @param collisionMask 碰撞掩码类型，默认为-1
   * @return 返回指向单体物体句柄的指针
   */
  virtual SingleBodyHandle addBox(double xLength,
                                  double yLength,
                                  double zLength,
                                  double mass,
                                  benchmark::CollisionGroupType collisionGroup = 1,
                                  benchmark::CollisionGroupType collisionMask = -1) = 0;

    /**
   * 添加一个圆柱体对象
   * @param radius 圆柱体的半径
   * @param height 圆柱体的高度
   * @param mass 圆柱体的质量
   * @param collisionGroup 碰撞组类型，默认为1
   * @param collisionMask 碰撞掩码类型，默认为-1
   * @return 返回指向单体物体句柄的指针
   */
  virtual SingleBodyHandle addCylinder(double radius,
                                       double height,
                                       double mass,
                                       benchmark::CollisionGroupType collisionGroup = 1,
                                       benchmark::CollisionGroupType collisionMask=-1) = 0;
    /**
     * 添加一个胶囊体对象
     * @param radius 胶囊体的半径
     * @param height 胶囊体的高度
     * @param mass 胶囊体的质量
     * @param collisionGroup 碰撞组类型，默认为1
     * @param collisionMask 碰撞掩码类型，默认为-1
     * @return 返回指向单体物体句柄的指针
     */
  virtual SingleBodyHandle addCapsule(double radius,
                                      double height,
                                      double mass,
                                      benchmark::CollisionGroupType collisionGroup = 1,
                                      benchmark::CollisionGroupType collisionMask=-1) = 0;

  /// pure virtual getter, setter

    /**
   * 获取当前世界中的对象数量
   * @return 返回当前世界中的对象数量。
   *         该方法用于获取已经添加到世界中的所有对象的计数。
   */
  virtual int getNumObject() = 0;

    /**
   * 获取当前世界中的接触数量
   * @return 返回当前世界中发生的接触数量。
   *         该方法用于检测物体之间的碰撞或接触事件，常用于物理仿真和碰撞检测。
   */
  virtual int getWorldNumContacts() = 0;

  /// pure virtual simulation methods

  /**
   * 更新当前帧状态。
   * 该方法通常在每个仿真步骤调用，以更新世界中所有对象的状态。
   */
  virtual void updateFrame() = 0;

    /**
   * 根据给定的时间步长进行仿真积分计算。
   * @param dt 时间步长，以秒为单位。
   */
  virtual void integrate(double dt) = 0;

    /**
   * 在控制输入应用之前，更新物体的运动学。
   * @param dt 时间步长，以秒为单位。
   */
  virtual void integrate1(double dt) = 0;

    /**
     * 在控制输入应用之后，进行第二阶段的积分计算。
     * @param dt 时间步长，以秒为单位。
     */
  virtual void integrate2(double dt) = 0;

    /**
   * 设置重力加速度。
   * @param gravity 重力向量，以三维向量表示（x, y, z）。
   */
  virtual void setGravity(Eigen::Vector3d gravity) = 0;

    /**
   * 设置弹簧的弹性常数（ERP）和摩擦弹性常数。
   * @param erp 弹性常数1。
   * @param erp2 弹性常数2。
   * @param frictionErp 摩擦弹性常数。
   */
  virtual void setERP(double erp, double erp2, double frictionErp) = 0;

 protected:
  /// sub methods

  /** 检查文件是否存在 */
  virtual void checkFileExistance(std::string nm);

  /** 处理单体 */
  virtual void processSingleBody(SingleBodyHandle handle);

    /**
   * 处理图形对象的逻辑。
   * @param go 指向图形对象的指针。
   * @param li 图形对象的索引或标识符。
   */
  virtual void processGraphicalObject(rai_graphics::object::SingleBodyObject* go, int li);

    /**
     * 调整指定对象的透明度。
     * @param ob 指向需要调整透明度的对象的指针。
     * @param hidable 指定该对象是否可隐藏。
     */
  virtual void adjustTransparency(rai_graphics::object::SingleBodyObject* ob, bool hidable);

  /// object list
  std::vector<SingleBodyHandle> sbHandles_;
  std::vector<object::SingleBodyObjectInterface *> framesAndCOMobj_;

  /// gui objects

  // GUI对象指针，用于管理图形界面操作。
  std::unique_ptr<rai_graphics::RAI_graphics> gui_;

  // 碰撞法线箭头指针，用于可视化碰撞法线方向。
  std::unique_ptr<rai_graphics::object::Arrow> contactNormalArrow_;

  // 碰撞点标记球体指针，用于标识碰撞点位置
  std::unique_ptr<rai_graphics::object::Sphere> contactPointMarker_;

  // 背景对象指针，用于设置图形界面的背景。
  std::unique_ptr<rai_graphics::object::Background> background_;

  // 质心标记球体指针，用于可视化物体的质心。
  std::unique_ptr<rai_graphics::object::Sphere> graphicalComMarker_;

  // 坐标轴箭头指针，表示三维空间的X、Y、Z轴方向。
  std::unique_ptr<rai_graphics::object::Arrow> frameX_, frameY_, frameZ_;

  /// gui properties
  rai_graphics::CameraProp cameraProperty_;
  rai_graphics::LightProp lightProperty_;

  /// gui watch timer
  StopWatch watch_, visualizerWatch_;

  /// gui window size
  const int windowWidth_ = 800;
  const int windowHeight_ = 600;

  /// gui option and flags
  int visualizerFlags_ = 0;
  bool isReady_=false;
  bool isEnded_=false;

};

} // benchmark

#endif //BENCHMARK_WORLD_RG_HPP
