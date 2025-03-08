/** 定义了单体物体的接口 */

#ifndef BENCHMARK_SINGLEBODYOBJECT_HPP
#define BENCHMARK_SINGLEBODYOBJECT_HPP

#include <Eigen/Geometry>
#include "../math.hpp"

namespace benchmark {

/** 定义一个类型别名 eQuaternion，表示一个 4x1 的列向量，通常用于表示四元数。**/
typedef Eigen::Map<Eigen::Matrix<double, 4, 1>> eQuaternion;

/** 定义一个类型别名 eRotationMat，表示一个 3x3 的矩阵，通常用于表示三维空间中的旋转矩阵 */
typedef Eigen::Map<Eigen::Matrix<double, 3, 3>> eRotationMat;

/** 定义一个类型别名 eVector3，表示一个 3x1 的列向量，通常用于表示三维空间中的向量 */
typedef Eigen::Map<Eigen::Matrix<double, 3, 1>> eVector3;

namespace object {

class SingleBodyObjectInterface {

 public:

/**
 * 获取相对于世界坐标系的物体的旋转四元数 (w, x, y, z)
 * @return 以 Eigen 矩阵格式返回的旋转四元数
 */
  virtual const eQuaternion getQuaternion() = 0;

/**
* 获取相对于世界坐标系的物体的旋转四元数 (w, x, y, z)
* @param quat 输出的旋转四元数，格式为 Vec<4>
*/
  virtual void getQuaternion(Vec<4>& quat) = 0;

/**
 * 获取相对于世界坐标系的物体的旋转矩阵
 * @return 以 Eigen 矩阵格式返回的旋转矩阵
 */
  virtual const eRotationMat getRotationMatrix() = 0;

/**
 * 获取相对于世界坐标系的物体的旋转矩阵
 * @param rotation 输出的旋转矩阵，格式为 Mat<3,3>
 */
  virtual void getRotationMatrix(Mat<3,3>& rotation) = 0;

/**
 * 获取相对于世界坐标系的物体的体框原点位置。
 * 注意：体框原点可能不在质心 (COM) 上
 *
 * @return 物体相对于世界坐标系的体框原点位置
 */
  virtual const eVector3 getPosition() = 0;

/**
 * 获取物体相对于世界坐标系的体框原点位置。
 * 注意：体框原点可能不在质心 (COM) 上
 *
 * @param pos_w   体框原点相对于世界坐标系的位置（输出）
 */
  virtual void getPosition_W(Vec<3>& pos_w) = 0;

/**
 * 获取物体相对于世界坐标系的质心原点位置。
 * @return  质心相对于世界坐标系的位置
 */
  virtual const eVector3 getComPosition() = 0;

/**
 * 获取物体相对于世界坐标系的线性速度。
 * @return  物体相对于世界坐标系的线性速度。
 */
  virtual const eVector3 getLinearVelocity() = 0;

/**
 * 获取物体相对于世界坐标系的角速度。
 * @return  物体相对于世界坐标系的角速度。
 */
  virtual const eVector3 getAngularVelocity() = 0;

/**
 * 设置体框原点相对于世界坐标系的位置
 * @param originPosition  以 Vector3d 格式表示的体框原点相对于世界坐标系的位置
 */
  virtual void setPosition(Eigen::Vector3d originPosition) = 0;

/**
 * 设置体框原点相对于世界坐标系的位置
 * @param x  x 坐标
 * @param y  y 坐标
 * @param z  z 坐标
 */
  virtual void setPosition(double x, double y, double z) = 0;

/**
 * 设置相对于世界坐标系的旋转四元数
 * @param quaternion  以 Quaterniond 格式表示的旋转四元数 (w, x, y, z)
 */
  virtual void setOrientation(Eigen::Quaterniond quaternion) = 0;

/**
 * 设置相对于世界坐标系的旋转四元数
 * @param w  四元数的实部
 * @param x  四元数的虚部 x
 * @param y  四元数的虚部 y
 * @param z  四元数的虚部 z
 */
  virtual void setOrientation(double w, double x, double y, double z) = 0;

/**
 * 设置相对于世界坐标系的旋转矩阵
 * @param rotationMatrix  以 Matrix3d 格式表示的 3x3 旋转矩阵
 */
  virtual void setOrientation(Eigen::Matrix3d rotationMatrix) = 0;

/**
 * 随机设置物体的朝向。
 */
  virtual void setOrientationRandom() = 0;

/**
 * 设置物体的体框原点和四元数相对于世界坐标系
 * @param originPosition  物体的体框原点
 * @param quaternion      物体的旋转四元数
 */
  virtual void setPose(Eigen::Vector3d originPosition, Eigen::Quaterniond quaternion) = 0;

/**
 * 设置物体的体框原点和旋转矩阵相对于世界坐标系
 * @param originPosition  物体的体框原点
 * @param rotationMatrix  物体的旋转矩阵
 */
  virtual void setPose(Eigen::Vector3d originPosition, Eigen::Matrix3d rotationMatrix) = 0;

/**
 * 设置物体的线性速度和角速度相对于世界坐标系
 * @param linearVelocity      质心的线性速度
 * @param angularVelocity     物体的角速度
 */
  virtual void setVelocity(Eigen::Vector3d linearVelocity, Eigen::Vector3d angularVelocity) = 0;

/**
 * 设置物体的线性速度和角速度相对于世界坐标系
 * @param dx  线性速度 x 轴分量
 * @param dy  线性速度 y 轴分量
 * @param dz  线性速度 z 轴分量
 * @param wx  角速度 x 轴分量
 * @param wy  角速度 y 轴分量
 * @param wz  角速度 z 轴分量
 */
  virtual void setVelocity(double dx, double dy, double dz, double wx, double wy, double wz) = 0;

/**
 * 设置外部施加的力
 * @param force  施加的外力，格式为 Eigen::Vector3d
 */
  virtual void setExternalForce(Eigen::Vector3d force) = 0;

/**
 * 设置外部施加的力矩
 * @param torque  施加的外力矩，格式为 Eigen::Vector3d
 */
  virtual void setExternalTorque(Eigen::Vector3d torque) = 0;

/**
   * 设置恢复系数
   * @param restitution  恢复系数，表示碰撞后的弹性程度
   */
  virtual void setRestitutionCoefficient(double restitution) = 0;

/**
 * 设置摩擦系数
 * @param friction  摩擦系数，表示物体间摩擦力的大小
 */
  virtual void setFrictionCoefficient(double friction) = 0;

/**
 * 获取动能
 * @return 动能的值
 */
  virtual double getKineticEnergy() = 0;

/**
 * 获取势能
 * @param gravity  重力向量，格式为 benchmark::Vec<3>
 * @return 势能的值
 */
  virtual double getPotentialEnergy(const benchmark::Vec<3> &gravity) = 0;

/**
 * 获取总能量
 * @param gravity  重力向量，格式为 benchmark::Vec<3>
 * @return 总能量的值
 */
  virtual double getEnergy(const benchmark::Vec<3> &gravity) = 0;

/**
 * 获取线动量
 * @return 线动量的值，格式为 Eigen::Map<Eigen::Matrix<double, 3, 1>>
 */
  virtual const Eigen::Map<Eigen::Matrix<double, 3, 1>> getLinearMomentum() = 0;

/**
 * 判断是否可视化坐标系和质心
 * @return 返回是否可视化的布尔值
 */
  virtual bool isVisualizeFramesAndCom() const = 0;

 protected:

  // from object
  bool visualizeFramesAndCom_ = true;

};

} // object
} // benchmark

#endif //BENCHMARK_SINGLEBODYOBJECT_HPP
