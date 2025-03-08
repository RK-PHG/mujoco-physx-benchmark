#include "MjcSingleBodyObject.hpp"

namespace mujoco_sim {
namespace object {

    /**
 * @brief MjcSingleBodyObject 构造函数。
 *
 * @param data 指向 mujoco 数据结构的指针。
 * @param model 指向 mujoco 模型结构的指针。
 * @param bodyId 物体的 ID。
 * @param geomId 物体几何体的 ID。
 */
MjcSingleBodyObject::MjcSingleBodyObject(mjData *data,
                                   mjModel *model,
                                   int bodyId,
                                   int geomId)
    : worldData_(data), worldModel_(model), bodyID_(bodyId), geomID_(geomId) {

  // init physical properties
  setGeomFriction({0.8, 0, 0});
}


/**
 * @brief 获取物体的四元数表示。
 *
 * 该函数从几何体的旋转矩阵中计算并返回物体的四元数。
 *
 * @return 返回物体的四元数。
 */
const benchmark::eQuaternion MjcSingleBodyObject::getQuaternion() {
  mjtNum *rotMat = getGeomRotMat(); // 获取几何体的旋转矩阵
  rotMatTemp_.e() << rotMat[0], rotMat[1], rotMat[2],
      rotMat[3], rotMat[4], rotMat[5],
      rotMat[6], rotMat[7], rotMat[8]; // 将旋转矩阵存储到临时变量中
  benchmark::rotMatToQuat(rotMatTemp_, quatTemp_);  // 将旋转矩阵转换为四元数
  return quatTemp_.e();  // 返回计算得到的四元数
}

/**
 * @brief 获取物体的四元数表示并存储到给定的向量中。
 *
 * 该函数从几何体的旋转矩阵中计算物体的四元数，并将结果存储到传入的向量参数中。
 *
 * @param quat 输出参数，用于存储计算得到的四元数（四个元素的向量）。
 */
    void MjcSingleBodyObject::getQuaternion(benchmark::Vec<4> &quat) {
      mjtNum *rotMat = getGeomRotMat(); // 获取几何体的旋转矩阵
      rotMatTemp_.e() << rotMat[0], rotMat[1], rotMat[2],
              rotMat[3], rotMat[4], rotMat[5],
              rotMat[6], rotMat[7], rotMat[8]; // 将旋转矩阵存储到临时变量中

      benchmark::rotMatToQuat(rotMatTemp_, quat); // 将旋转矩阵转换为四元数并存储到输出参数
    }

/**
 * @brief 获取物体的旋转矩阵表示。
 *
 * 该函数从几何体的旋转矩阵中提取物体的旋转信息，并返回对应的旋转矩阵。
 *
 * @return 返回一个包含物体旋转矩阵的三维矩阵对象。
 */
const benchmark::eRotationMat MjcSingleBodyObject::getRotationMatrix() {
  mjtNum *rotMat = getGeomRotMat();
  rotMatTemp_.e() << rotMat[0], rotMat[1], rotMat[2],
      rotMat[3], rotMat[4], rotMat[5],
      rotMat[6], rotMat[7], rotMat[8];
  return rotMatTemp_.e();
}

/**
 * @brief 获取物体的旋转矩阵并存储到指定的矩阵对象中。
 *
 * 该函数从几何体的旋转矩阵中提取物体的旋转信息，并将其存储到提供的三维矩阵参数中。
 *
 * @param rotation 输出参数，存储物体的旋转矩阵。
 */
    void MjcSingleBodyObject::getRotationMatrix(benchmark::Mat<3, 3> &rotation) {
      mjtNum *rotMat = getGeomRotMat(); // 获取几何体的旋转矩阵
      rotation.e() << rotMat[0], rotMat[1], rotMat[2],
              rotMat[3], rotMat[4], rotMat[5],
              rotMat[6], rotMat[7], rotMat[8]; // 将旋转矩阵存储到输出参数中
    }

/**
 * @brief 获取物体的位置。
 *
 * 该函数从几何体获取当前位置，并返回一个三维向量表示物体的位置。
 *
 * @return 物体的位置，表示为三维向量。
 */
const benchmark::eVector3 MjcSingleBodyObject::getPosition() {
  mjtNum *pos = getGeomPosition();
  posTemp_ = {pos[0], pos[1], pos[2]};
  return posTemp_.e();
}


/**
 * @brief 获取物体的质心位置。
 *
 * 该函数从物体的质心位置获取并返回一个三维向量表示物体的质心位置。
 *
 * @return 物体的质心位置，表示为三维向量。
 */
    const benchmark::eVector3 MjcSingleBodyObject::getComPosition() {
      mjtNum *pos = getBodyComPosition(); // 获取物体的质心位置
      posTemp_ = {pos[0], pos[1], pos[2]}; // 存储位置到临时变量
      return posTemp_.e(); // 返回质心位置向量
    }

/**
 * @brief 获取物体的线速度。
 *
 * 该函数从物体获取当前的线速度并返回一个三维向量表示物体的线速度。
 *
 * @return 物体的线速度，表示为三维向量。
 */
    const benchmark::eVector3 MjcSingleBodyObject::getLinearVelocity() {
      mjtNum *linVel = getBodyLinearVelocity(); // 获取物体的线速度
      linVelTemp_ = {linVel[0], linVel[1], linVel[2]}; // 存储线速度到临时变量
      return linVelTemp_.e(); // 返回线速度向量
    }

/**
 * @brief 获取物体的角速度。
 *
 * 该函数从物体获取当前的角速度并返回一个三维向量表示物体的角速度。
 *
 * @return 物体的角速度，表示为三维向量。
 */
    const benchmark::eVector3 MjcSingleBodyObject::getAngularVelocity() {
      mjtNum *angVel = getBodyAngularVelocity(); // 获取物体的角速度
      angVelTemp_ = {angVel[0], angVel[1], angVel[2]}; // 存储角速度到临时变量
      return angVelTemp_.e(); // 返回角速度向量
    }

/**
 * @brief 获取物体在世界坐标系中的位置。
 *
 * 该函数将物体的当前位置填充到给定的三维向量中。
 *
 * @param pos_w 输出参数，表示物体在世界坐标系中的位置。
 */
    void MjcSingleBodyObject::getPosition_W(benchmark::Vec<3> &pos_w) {
      mjtNum *pos = getGeomPosition(); // 获取几何体的位置
      pos_w = {pos[0], pos[1], pos[2]}; // 填充世界坐标系中的位置
    }

/**
 * @brief 检查是否可视化框架和质心。
 *
 * 该函数返回一个布尔值，指示是否可视化物体的框架和质心。
 *
 * @return 如果可视化框架和质心，则返回true；否则返回false。
 */
    bool MjcSingleBodyObject::isVisualizeFramesAndCom() const {
      return visualizeFramesAndCom_; // 返回可视化标志
    }

/**
 * @brief 设置物体的外力。
 *
 * 该函数将外力应用于物体。
 *
 * @param force 要施加的外力，表示为三维向量。
 */
    void MjcSingleBodyObject::setExternalForce(Eigen::Vector3d force) {
      mjtNum *extforce = (worldData_->xfrc_applied + 6 * bodyID_); // 获取外力指针
      extforce[0] = force[0]; // 设置外力的x分量
      extforce[1] = force[1]; // 设置外力的y分量
      extforce[2] = force[2]; // 设置外力的z分量
    }

/**
 * @brief 设置物体的外转矩。
 *
 * 该函数将外转矩应用于物体。
 *
 * @param torque 要施加的外转矩，表示为三维向量。
 */
    void MjcSingleBodyObject::setExternalTorque(Eigen::Vector3d torque) {
      mjtNum *extforce = (worldData_->xfrc_applied + 6 * bodyID_ + 3); // 获取外转矩指针
      extforce[0] = torque[0]; // 设置外转矩的x分量
      extforce[1] = torque[1]; // 设置外转矩的y分量
      extforce[2] = torque[2]; // 设置外转矩的z分量
    }

/**
 * @brief 获取几何体的位置。
 *
 * 该函数根据物体的ID和几何体ID返回几何体在世界坐标系中的位置。
 *
 * @return 指向几何体位置的指针。
 */
    mjtNum *MjcSingleBodyObject::getGeomPosition() {
      int geomIndex = worldModel_->body_geomadr[bodyID_] + geomID_; // 计算几何体索引
      return worldData_->geom_xpos + 3 * geomIndex; // 返回几何体位置指针
    }

/**
 * @brief 获取几何体的旋转矩阵。
 *
 * 该函数根据物体的ID和几何体ID返回几何体的旋转矩阵。
 *
 * @return 指向几何体旋转矩阵的指针。
 */
    mjtNum *MjcSingleBodyObject::getGeomRotMat() {
      int geomIndex = worldModel_->body_geomadr[bodyID_] + geomID_; // 计算几何体索引
      return worldData_->geom_xmat + 9 * geomIndex; // 返回几何体旋转矩阵指针
    }

/**
 * @brief 获取物体的质心位置。
 *
 * 该函数返回物体的质心位置。
 *
 * @return 指向物体质心位置的指针。
 */
    mjtNum *MjcSingleBodyObject::getBodyComPosition() {
      return worldData_->xipos + 3 * bodyID_; // 返回物体质心位置指针
    }

/**
 * @brief 获取物体的线速度。
 *
 * 该函数返回物体的线速度。
 *
 * @return 指向物体线速度的指针。
 */
    mjtNum *MjcSingleBodyObject::getBodyLinearVelocity() {
      mj_objectVelocity(worldModel_, worldData_, mjOBJ_BODY, bodyID_, temp6D_, 0); // 获取物体的线速度
      return temp6D_ + 3; // 返回线速度指针
    }

/**
 * @brief 获取物体的角速度。
 *
 * 该函数返回物体的角速度。
 *
 * @return 指向物体角速度的指针。
 */
    mjtNum *MjcSingleBodyObject::getBodyAngularVelocity() {
      mj_objectVelocity(worldModel_, worldData_, mjOBJ_BODY, bodyID_, temp6D_, 0); // 获取物体的角速度
      return temp6D_; // 返回角速度指针
    }

/**
 * @brief 设置几何体的摩擦系数。
 *
 * 该函数根据给定的摩擦值设置几何体的摩擦系数，支持滑动、旋转和滚动摩擦。
 *
 * @param friction 摩擦系数，表示为一个包含滑动、旋转和滚动摩擦的三维向量。
 */
    void MjcSingleBodyObject::setGeomFriction(benchmark::Vec<3> friction) {
      int geomIndex = worldModel_->body_geomadr[bodyID_] + geomID_; // 计算几何体索引
      worldModel_->geom_friction[3 * geomIndex] = friction[0]; // 设置滑动摩擦
      worldModel_->geom_friction[3 * geomIndex + 1] = friction[1]; // 设置旋转摩擦
      worldModel_->geom_friction[3 * geomIndex + 2] = friction[2]; // 设置滚动摩擦
    }

/**
 * @brief 设置摩擦系数。
 *
 * 该函数将给定的摩擦系数应用于几何体的滑动摩擦，其他摩擦设为零。
 *
 * @param friction 要设置的摩擦系数。
 */
    void MjcSingleBodyObject::setFrictionCoefficient(double friction) {
      setGeomFriction({friction, 0, 0}); // 设置滑动摩擦，其余为0
    }

/**
 * @brief 获取物体的质量。
 *
 * 该函数返回物体的质量。
 *
 * @return 物体的质量。
 */
    mjtNum MjcSingleBodyObject::getBodyMass() {
      return worldModel_->body_mass[bodyID_]; // 返回物体的质量
    }

/**
 * @brief 获取物体的惯性矩阵。
 *
 * 该函数返回物体的惯性矩阵。
 *
 * @return 指向物体惯性矩阵的指针。
 */
    mjtNum *MjcSingleBodyObject::getBodyInertia() {
      return worldModel_->body_inertia + 3 * bodyID_; // 返回物体惯性矩阵指针
    }

/**
 * @brief 获取物体的动能。
 *
 * 该函数根据物体的质量和速度计算动能。
 *
 * @return 物体的动能。
 */
    double MjcSingleBodyObject::getKineticEnergy() {
      double mass = getBodyMass(); // 获取物体的质量
      double *inertia = getBodyInertia(); // 获取物体的惯性矩阵

      getLinearVelocity(); // 获取线速度
      getAngularVelocity(); // 获取角速度
      benchmark::Mat<3,3> I; // 创建惯性矩阵
      I.e() << inertia[0], 0, 0,
              0, inertia[1], 0,
              0, 0, inertia[2];

      // 计算角动能
      double angEnergy = 0;
      benchmark::Mat<3,3> I_w; // 创建世界坐标系下的惯性矩阵
      getRotationMatrix(); // 获取旋转矩阵
      benchmark::similarityTransform(rotMatTemp_, I, I_w); // 计算世界坐标系下的惯性矩阵
      benchmark::vecTransposeMatVecMul(angVelTemp_, I_w, angEnergy); // 计算角动能

      // 计算线动能
      double linEnergy = 0;
      benchmark::vecDot(linVelTemp_, linVelTemp_, linEnergy); // 计算线动能

      return 0.5 * angEnergy + 0.5 * mass * linEnergy; // 返回总动能
    }

/**
 * @brief 获取物体的势能。
 *
 * 该函数根据物体的高度和重力计算势能。
 *
 * @param gravity 表示重力的三维向量。
 * @return 物体的势能。
 */
    double MjcSingleBodyObject::getPotentialEnergy(const benchmark::Vec<3> &gravity) {
      double potential = 0; // 初始化势能
      double mass = getBodyMass(); // 获取物体的质量

      getPosition(); // 获取物体的位置
      benchmark::vecDot(posTemp_, gravity, potential); // 计算势能
      return -potential * mass; // 返回势能
    }

/**
 * @brief 获取物体的总能量。
 *
 * 该函数返回物体的动能和势能之和。
 *
 * @param gravity 表示重力的三维向量。
 * @return 物体的总能量。
 */
    double MjcSingleBodyObject::getEnergy(const benchmark::Vec<3> &gravity) {
      return getKineticEnergy() + getPotentialEnergy(gravity); // 返回总能量
    }


/// deprecated
/// ===================================================================
void MjcSingleBodyObject::setPosition(Eigen::Vector3d originPosition) {

}
void MjcSingleBodyObject::setPosition(double x, double y, double z) {

}
void MjcSingleBodyObject::setOrientation(Eigen::Quaterniond quaternion) {

}
void MjcSingleBodyObject::setOrientation(double w, double x, double y, double z) {

}
void MjcSingleBodyObject::setOrientation(Eigen::Matrix3d rotationMatrix) {

}
void MjcSingleBodyObject::setOrientationRandom() {

}
void MjcSingleBodyObject::setPose(Eigen::Vector3d originPosition, Eigen::Quaterniond quaternion) {

}
void MjcSingleBodyObject::setPose(Eigen::Vector3d originPosition, Eigen::Matrix3d rotationMatrix) {

}

void MjcSingleBodyObject::setVelocity(Eigen::Vector3d linearVelocity, Eigen::Vector3d angularVelocity) {

}
void MjcSingleBodyObject::setVelocity(double dx, double dy, double dz, double wx, double wy, double wz) {

}
void MjcSingleBodyObject::setRestitutionCoefficient(double restitution) {

}
double MjcSingleBodyObject::getMass() {
  return getBodyMass();
}
bool MjcSingleBodyObject::isMovable() const {
  return isMovable_;
}
const Eigen::Map<Eigen::Matrix<double, 3, 1>> MjcSingleBodyObject::getLinearMomentum() {
  getLinearVelocity();
  double mass = getMass();
  benchmark::vecScalarMul(mass, linVelTemp_, linearMomentum_);
  return linearMomentum_.e();
}

void MjcSingleBodyObject::setCollisionGroupAndMask(int collisionGroup, int collisionMask) {
  int geomIndex = worldModel_->body_geomadr[bodyID_] + geomID_;
  worldModel_->geom_contype[geomIndex] = collisionGroup;
  worldModel_->geom_conaffinity[geomIndex] = collisionMask;
}

} // object
} // mujoco_sim