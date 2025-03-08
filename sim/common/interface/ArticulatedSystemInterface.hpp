/** 本文件定义了关节系统的接口  */

#ifndef BENCHMARK_ARTICULATEDSYSTEM_HPP
#define BENCHMARK_ARTICULATEDSYSTEM_HPP

#include "../math.hpp"

namespace benchmark {
namespace object {

/** shape */
enum class Shape {
  Box = 0,
  Cylinder,
  Sphere,
  Mesh,
  Capsule,
  Cone
};

/** 关节系统的接口，用于定义机器人 */
class ArticulatedSystemInterface {

 public:
  /** 列向量类型，大小在运行时决定 */
  typedef Eigen::Map<Eigen::Matrix<double, -1, 1> > EigenVec;
  /** 矩阵类型，大小在运行时决定 */
  typedef Eigen::Map<Eigen::Matrix<double, -1, -1> > EigenMat;

  /** 获取可视化对象信息 */
  std::vector<std::tuple<benchmark::Mat<3, 3>, benchmark::Vec<3>, int, Shape, benchmark::Vec<4>>>& getVisOb() {
    return visObj;
  };

  /** 获取碰撞对象信息 */
  std::vector<std::tuple<benchmark::Mat<3, 3>, benchmark::Vec<3>, int, Shape>>& getVisColOb() {
    return visColObj;
  };

  /** 获取广义坐标 */
  virtual const EigenVec getGeneralizedCoordinate() = 0;

  /** 获取广义速度 */
  virtual const EigenVec getGeneralizedVelocity() = 0;

  /** 设置广义坐标
   *  对于浮动机器人： 线性位置，基座旋转，关节位置
   * */
  /* For floating-base robots, [linearPosition_W, baseRationInQuaternion, joint Angles]
   * For fixed-base robot, [joint angles]
   * The dimension is the DOF+1 for floating-based, and DOF for fixed based. (obtained by getDOF())*/
  virtual void setGeneralizedCoordinate(const Eigen::VectorXd &jointState) = 0;

  /** 设置广义速度
   *  对于浮动机器人，有线速度，角速度，和关节速度
   * */
  /* For floating-base robots, [linearVelocity_W, angularVelocity_W, jointVelocity]
   * The dimension is the same as dof (obtained with getDOF)*/
  virtual void setGeneralizedVelocity(const Eigen::VectorXd &jointVel) = 0;

  /** 设置广义坐标 */
  virtual void setGeneralizedCoordinate(std::initializer_list<double> jointState) = 0;

  /** 设置广义速度 */
  virtual void setGeneralizedVelocity(std::initializer_list<double> jointVel) = 0;

  /** 设置广义力矩 */
  virtual void setGeneralizedForce(std::initializer_list<double> tau) = 0;

  /** 获取机器人状态： 坐标和速度*/
  virtual void getState(Eigen::VectorXd &genco, Eigen::VectorXd &genvel) = 0;

  /** 设置机器人状态 */
  virtual void setState(const Eigen::VectorXd &genco, const Eigen::VectorXd &genvel) = 0;

  /** 设置广义力矩 */
  virtual void setGeneralizedForce(const Eigen::VectorXd &tau) = 0;

  /** 获取广义力矩 */
  virtual const EigenVec getGeneralizedForce() = 0;

  /** 获取自由度 */
  virtual int getDOF() = 0;

  /** 获取机器人的状态维度 */
  virtual int getStateDimension() = 0;

  /// r, g, b, alpha 设置机器人颜色，rgb通道再加上透明度
  virtual void setColor(Eigen::Vector4d color) = 0;

  /** 获取笛卡尔空间的线性动量，机器人在xyz三个方向上的动量分量 */
  virtual const Eigen::Map<Eigen::Matrix<double, 3, 1>> getLinearMomentumInCartesianSpace() = 0;

  /** 获取机器人的总质量 */
  virtual double getTotalMass() = 0;

  /** 获取在给定重力条件下机器人的动能 */
  virtual double getEnergy(const benchmark::Vec<3> &gravity) = 0;

 public:
  // orientation, position, link_id, shape, color
  /** 这是一个存储可视化碰撞对象的向量，每个对象由一个 3x3 的矩阵（可能表示物体的变换）、一个 3 维向量（可能表示物体的位置或方向）、一个整数（可能表示对象的 ID 或状态），以及一个形状（Shape 类型）组成。 */
  std::vector<std::tuple<benchmark::Mat<3, 3>, benchmark::Vec<3>, int, Shape>> visColObj;

  /** 这是一个存储可视化对象的向量，每个对象由一个 3x3 的矩阵、一个 3 维向量、一个整数、一个形状和一个 4 维向量（可能表示颜色或其他属性）组成*/
  std::vector<std::tuple<benchmark::Mat<3, 3>, benchmark::Vec<3>, int, Shape, benchmark::Vec<4>>> visObj;

  /** 这是一个存储可视化属性的向量，每个属性由一个字符串（可能表示属性的名称）和一个 4 维向量（可能表示与该属性相关的数值）组成。*/
  std::vector<std::pair<std::string, benchmark::Vec<4>>> visProps_;

  /** 这是一个存储可视化碰撞属性的向量，每个属性由一个字符串和一个 4 维向量组成 */
  std::vector<std::pair<std::string, benchmark::Vec<4>>> visColProps_;

 protected:
  //// generalized states, velocity, force 坐标、速度、力矩
  benchmark::VecDyn genCoordinate_;
  benchmark::VecDyn genVelocity_;
  benchmark::VecDyn genForce_;

  //// 颜色
  benchmark::Vec<4> color_ = {1.0, 1.0, 1.0, 1.0};

  int dof_ = 0;
  int stateDimension_ = 0;
  bool isFixed_ = true;
};

} // object
} // benchmark

#endif //BENCHMARK_ARTICULATEDSYSTEM_HPP
