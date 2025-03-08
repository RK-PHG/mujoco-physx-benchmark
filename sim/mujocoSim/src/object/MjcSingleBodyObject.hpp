/** 用于实现适用于mujoco的SingleBodyObjectInterface */

#ifndef MUJOCOSIM_SINGLEBODYOBJECT_HPP
#define MUJOCOSIM_SINGLEBODYOBJECT_HPP

#include <Eigen/Geometry>
#include <mujoco/mujoco.h>
#include <common/UserHandle.hpp>

#include "common/math.hpp"
#include "common/interface/SingleBodyObjectInterface.hpp"

namespace mujoco_sim {
namespace object {

/**
 * @brief MjcSingleBodyObject 类实现单体物体的接口。
 */
class MjcSingleBodyObject: public benchmark::object::SingleBodyObjectInterface {

 public:
    /**
   * @brief 构造一个 MjcSingleBodyObject 实例。
   *
   * @param data 指向 mujoco 数据结构的指针。
   * @param model 指向 mujoco 模型结构的指针。
   * @param bodyId 物体的 ID。
   * @param geomId 物体几何体的 ID。
   */
  MjcSingleBodyObject(mjData *data,
                     mjModel *model,
                     int bodyId,
                     int geomId);

  /// see base class for details
    /**
      * @brief 获取物体的四元数表示。
      *
      * @return 物体的四元数。
      */
  const benchmark::eQuaternion getQuaternion() override ;

    /**
   * @brief 获取物体的四元数表示，存储到给定的向量中。
   *
   * @param quat 存储四元数的向量。
   */
  void getQuaternion(benchmark::Vec<4>& quat) override ;

    /**
    * @brief 获取物体的旋转矩阵表示。
    *
    * @return 物体的旋转矩阵。
    */
  const benchmark::eRotationMat getRotationMatrix() override ;

    /**
   * @brief 获取物体的旋转矩阵，存储到给定的矩阵中。
   *
   * @param rotation 存储旋转矩阵的矩阵。
   */
  void getRotationMatrix(benchmark::Mat<3,3>& rotation) override ;

    /**
     * @brief 获取物体的位置。
     *
     * @return 物体的位置向量。
     */
  const benchmark::eVector3 getPosition() override ;

    /**
     * @brief 获取物体的质心位置。
     *
     * @return 物体的质心位置向量。
     */
  const benchmark::eVector3 getComPosition() override ;

    /**
     * @brief 获取物体的线速度。
     *
     * @return 物体的线速度向量。
     */
  const benchmark::eVector3 getLinearVelocity() override ;

    /**
   * @brief 获取物体的角速度。
   *
   * @return 物体的角速度向量。
   */
  const benchmark::eVector3 getAngularVelocity() override ;

    /**
     * @brief 获取物体在世界坐标系下的位置。
     *
     * @param pos_w 存储世界坐标系位置的向量。
     */
  void getPosition_W(benchmark::Vec<3>& pos_w) override ;

    /**
    * @brief 获取物体的动能。
    *
    * @return 物体的动能值。
    */
  double getKineticEnergy() override ;

    /**
   * @brief 获取物体在给定重力下的势能。
   *
   * @param gravity 重力向量。
   * @return 物体的势能值。
   */
  double getPotentialEnergy(const benchmark::Vec<3> &gravity) override ;

    /**
   * @brief 获取物体在给定重力下的总能量。
   *
   * @param gravity 重力向量。
   * @return 物体的总能量值。
   */
  double getEnergy(const benchmark::Vec<3> &gravity) override ;

    /**
   * @brief 获取物体的线动量。
   *
   * @return 物体的线动量向量。
   */
  const benchmark::eVector3 getLinearMomentum() override;

    /**
   * @brief 设置外部力。
   *
   * @param force 外部力向量。
   */
  void setExternalForce(Eigen::Vector3d force) override ;

    /**
    * @brief 设置外部力矩。
    *
    * @param torque 外部力矩向量。
    */
  void setExternalTorque(Eigen::Vector3d torque) override ;

    /**
   * @brief 设置摩擦系数。
   *
   * @param friction 摩擦系数值。
   */
  void setFrictionCoefficient(double friction) override ;

    /**
   * @brief 设置碰撞组和碰撞掩码。
   *
   * @param collisionGroup 碰撞组 ID。
   * @param collisionMask 碰撞掩码 ID。
   */
  void setCollisionGroupAndMask(int collisionGroup, int collisionMask);

    /**
    * @brief 获取物体的质量。
    *
    * @return 物体的质量值。
    */
  double getMass();

    /**
    * @brief 判断物体是否可移动。
    *
    * @return 如果可移动返回 true，否则返回 false。
    */
  bool isMovable() const;

 private:
  /// deprecated overrided functions
  /// ===================================
  void setPosition(Eigen::Vector3d originPosition) override ;
  void setPosition(double x, double y, double z) override ;
  void setOrientation(Eigen::Quaterniond quaternion) override ;
  void setOrientation(double w, double x, double y, double z) override ;
  void setOrientation(Eigen::Matrix3d rotationMatrix) override ;
  void setOrientationRandom() override ;
  void setPose(Eigen::Vector3d originPosition, Eigen::Quaterniond quaternion) override ;
  void setPose(Eigen::Vector3d originPosition, Eigen::Matrix3d rotationMatrix) override ;
  void setVelocity(Eigen::Vector3d linearVelocity, Eigen::Vector3d angularVelocity) override ;
  void setVelocity(double dx, double dy, double dz, double wx, double wy, double wz) override ;

  void setRestitutionCoefficient(double restitution) override ;

  bool isVisualizeFramesAndCom() const override ;
  /// ===================================

  mjtNum *getGeomPosition();
  mjtNum *getGeomRotMat();
  mjtNum *getBodyComPosition();

  mjtNum *getBodyLinearVelocity();
  mjtNum *getBodyAngularVelocity();

  mjtNum getBodyMass();
  mjtNum *getBodyInertia();

  void setGeomFriction(benchmark::Vec<3> friction);

 protected:
  int bodyID_ = 0;    // body id in world
  int geomID_ = 0;    // geometry id in body

  mjData* worldData_;
  mjModel* worldModel_;

  // pose and velocity
  benchmark::Vec<4> quatTemp_ = {1.0, 0.0, 0.0, 0.0};
  benchmark::Mat<3, 3> rotMatTemp_;
  benchmark::Vec<3> posTemp_ = {0.0, 0.0, 0.0};
  benchmark::Vec<3> linVelTemp_ = {0.0, 0.0, 0.0};
  benchmark::Vec<3> angVelTemp_ = {0.0, 0.0, 0.0};

  benchmark::Vec<3> linearMomentum_ = {0, 0, 0};

  mjtNum temp6D_[6] = {0, 0, 0, 0, 0, 0};
  mjtNum temp3D_[3] = {0, 0, 0};
  mjtNum *tempPtr;

  bool isMovable_ = true;

};

} // object
} // mujoco_sim

#endif //MUJOCOSIM_SINGLEBODYOBJECT_HPP
