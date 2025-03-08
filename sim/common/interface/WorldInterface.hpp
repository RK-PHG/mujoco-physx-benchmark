/** 用于控制世界变量 */

#ifndef BENCHMARK_WORLDINTERFACE_HPP
#define BENCHMARK_WORLDINTERFACE_HPP

#include "SingleBodyObjectInterface.hpp"
#include "common/interface/CheckerboardInterface.hpp"
#include "../Configure.hpp"

namespace bo = benchmark::object;

namespace benchmark {

class WorldInterface {

 public:

    /**
     * 添加一个棋盘对象
     * @param gridSize 棋盘的网格大小
     * @param xLength 棋盘在 x 方向上的长度
     * @param yLength 棋盘在 y 方向上的长度
     * @param reflectanceI 棋盘的反射率
     * @param shape 棋盘的形状
     * @param collisionGroup 碰撞组类型
     * @param collisionMask 碰撞掩码类型
     * @return 返回指向单体物体接口的指针
     */
  virtual object::SingleBodyObjectInterface *addCheckerboard(double gridSize,
                                                             double xLength,
                                                             double yLength,
                                                             double reflectanceI,
                                                             bo::CheckerboardShape shape,
                                                             benchmark::CollisionGroupType collisionGroup,
                                                             benchmark::CollisionGroupType collisionMask) = 0;
    /**
     * 添加一个球体对象
     * @param radius 球体的半径
     * @param mass 球体的质量
     * @param collisionGroup 碰撞组类型
     * @param collisionMask 碰撞掩码类型
     * @return 返回指向单体物体接口的指针
     */
  virtual object::SingleBodyObjectInterface *addSphere(double radius,
                                                       double mass,
                                                       benchmark::CollisionGroupType collisionGroup,
                                                       benchmark::CollisionGroupType collisionMask) = 0;

    /**
     * 添加一个立方体对象
     * @param xLength 立方体在 x 方向上的长度
     * @param yLength 立方体在 y 方向上的长度
     * @param zLength 立方体在 z 方向上的长度
     * @param mass 立方体的质量
     * @param collisionGroup 碰撞组类型
     * @param collisionMask 碰撞掩码类型
     * @return 返回指向单体物体接口的指针
     */
  virtual object::SingleBodyObjectInterface *addBox(double xLength,
                                                    double yLength,
                                                    double zLength,
                                                    double mass,
                                                    benchmark::CollisionGroupType collisionGroup,
                                                    benchmark::CollisionGroupType collisionMask) = 0;

    /**
   * 添加一个胶囊对象
   * @param radius 胶囊的半径
   * @param height 胶囊的高度
   * @param mass 胶囊的质量
   * @param collisionGroup 碰撞组类型
   * @param collisionMask 碰撞掩码类型
   * @return 返回指向单体物体接口的指针
   */
  virtual object::SingleBodyObjectInterface *addCapsule(double radius,
                                                        double height,
                                                        double mass,
                                                        benchmark::CollisionGroupType collisionGroup,
                                                        benchmark::CollisionGroupType collisionMask) = 0;

    /**
   * 添加一个圆柱体对象
   * @param radius 圆柱体的半径
   * @param height 圆柱体的高度
   * @param mass 圆柱体的质量
   * @param collisionGroup 碰撞组类型，默认值为 1
   * @param collisionMask 碰撞掩码类型，默认值为 -1
   * @return 返回指向单体物体接口的指针
   */
  virtual object::SingleBodyObjectInterface *addCylinder(double radius,
                                                         double height,
                                                         double mass,
                                                         CollisionGroupType collisionGroup=1,
                                                         CollisionGroupType collisionMask=-1) = 0;


    /**
     * 设置重力加速度
     * @param gravity 3D 重力加速度向量
     */
  virtual void setGravity(const benchmark::Vec<3> &gravity) = 0;

    /**
   * 获取当前场景中的物体数量
   * @return 当前物体的数量
   */
  virtual int getNumObject() = 0;

    /**
    * 进行一次模拟步骤，时间步长为 dt
    * @param dt 时间步长，单位为秒
    */
  virtual void integrate(double dt) = 0;


    /**
     * 在应用控制输入之前更新运动学，使用时间步长 dt
     * @param dt 时间步长，单位为秒
     */
  virtual void integrate1(double dt) = 0;

    /**
     * 在应用控制输入之后进行一次模拟步骤
     * @param dt 时间步长，单位为秒
     */
  virtual void integrate2(double dt) = 0;
};

}


#endif //BENCHMARK_WORLDINTERFACE_HPP
