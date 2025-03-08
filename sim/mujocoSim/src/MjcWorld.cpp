/** 用于实现 MjcWorld 类函数 */

#include "MjcWorld.hpp"

namespace mujoco_sim {

/**
 * @brief MjcWorld 构造函数，用于初始化 MuJoCo 模拟世界。
 *
 * 通过指定模型路径、密钥路径、求解器类型以及积分器类型来构建模拟环境。
 *
 * @param modelPath 模型文件的路径（XML 格式）。
 * @param keyPath MuJoCo 密钥文件的路径。(较高版本的 mujoco 无需秘钥，此变量废弃)
 * @param solver 指定求解器类型（如 PGS、NEWTON、CG）。
 * @param integrator 指定积分器类型（如 EULER、RK4）。
 */
mujoco_sim::MjcWorld::MjcWorld(const char *modelPath,
                               const char *keyPath,
                               SolverOption solver,
                               IntegratorOption integrator) {

  // 用于存储错误消息
  char error[1000];

  // 加载模型文件
  worldModel_ = mj_loadXML(modelPath, NULL, error, 1000);

  // 判断是否加载成功
  if( !worldModel_ )
  {
    RAIFATAL(error);
  }

  // 设置求解器类型
  switch (solver) {
    case SOLVER_PGS:
      worldModel_->opt.solver = mjSOL_PGS;  // PGS 求解器
      break;
    case SOLVER_NEWTON:
      worldModel_->opt.solver = mjSOL_NEWTON; // Newton 求解器
      break;
    case SOLVER_CG:
      worldModel_->opt.solver = mjSOL_CG; // 共轭梯度法（CG）求解器
      break;
    default:
      worldModel_->opt.solver = mjSOL_PGS; // 默认使用 PGS 求解器
  }

  // 设置积分器类型
  switch (integrator) {
    case INTEGRATOR_EULER:
      worldModel_->opt.integrator = mjINT_EULER; // 欧拉积分器
      break;
    case INTEGRATOR_RK4:
      worldModel_->opt.integrator = mjINT_RK4; // 四阶龙格库塔（RK4）积分器
      break;
    default:
      worldModel_->opt.integrator = mjINT_EULER; // 默认使用欧拉积分器
  }

  // 根据模型创建数据
  worldData_ = mj_makeData(worldModel_);
  simOption_ = &worldModel_->opt;

  // 初始化仿真选项，设置重力
  simOption_->gravity[0] = 0;
  simOption_->gravity[1] = 0;
  simOption_->gravity[2] = -9.81;

  // 初始化变量
  dof_ = worldModel_->nv;
  numActuators_ = worldModel_->na;
  dimGenCoord_ = worldModel_->nq;

  // 初始化广义坐标、速度和力的向量，并全部置为零
  generalizedCoordinate_.resize(dimGenCoord_);
  generalizedCoordinate_.setZero();
  generalizedVelocity_.resize(dof_);
  generalizedVelocity_.setZero();
  generalizedForce_.resize(dof_);
  generalizedForce_.setZero();

  // 启用能量计算
  simOption_->enableflags |= mjENBL_ENERGY;
}

mujoco_sim::MjcWorld::~MjcWorld() {

  // 删除物体释放空间
  for (auto *ob: objectList_)
    delete ob;

  // 删除数据释放空间
  mj_deleteData(worldData_);
  mj_deleteModel(worldModel_);

}

/** 获取模型 */
mjModel *mujoco_sim::MjcWorld::getWorldModel() const {
  return worldModel_;
}

/** 获取数据 */
mjData *mujoco_sim::MjcWorld::getWorldData() const {
  return worldData_;
}


/**
 * @brief 向模拟世界中添加一个球体对象。
 *
 * 该函数创建一个球体对象并将其添加到当前的物体列表中。
 *
 * @param radius 球体的半径。
 * @param mass 球体的质量。
 * @param bodyId 球体所属的刚体 ID。
 * @param geomId 球体几何体的 ID。
 * @return object::MjcSphere* 返回指向新创建的 MjcSphere 球体对象的指针。
 */
object::MjcSphere *MjcWorld::addSphere(double radius,
                                 double mass,
                                 int bodyId,
                                 int geomId) {
  object::MjcSphere *sphere = new object::MjcSphere(radius, mass, worldData_, worldModel_, bodyId, geomId);
  objectList_.push_back(sphere);
  return sphere;
}

object::MjcBox *MjcWorld::addBox(double xLength,
                           double yLength,
                           double zLength,
                           double mass,
                           int bodyId,
                           int geomId) {
  object::MjcBox *box = new object::MjcBox(xLength, yLength, zLength, worldData_, worldModel_, bodyId, geomId);
  objectList_.push_back(box);
  return box;
}

object::MjcCheckerBoard *MjcWorld::addCheckerboard(double gridSize,
                                             double xLength,
                                             double yLength,
                                             double reflectanceI,
                                             bo::CheckerboardShape shape,
                                             int bodyId,
                                             int geomId) {
  RAIFATAL_IF(shape == bo::BOX_SHAPE, "box shape ground is not supported")

  object::MjcCheckerBoard *checkerBoard =
      new object::MjcCheckerBoard(xLength, yLength, worldData_, worldModel_, bodyId, geomId);
  objectList_.push_back(checkerBoard);
  return checkerBoard;
}

object::MjcCapsule *MjcWorld::addCapsule(double radius,
                                   double height,
                                   double mass,
                                   int bodyId,
                                   int geomId) {
  object::MjcCapsule *capsule = new object::MjcCapsule(radius, height, worldData_, worldModel_, bodyId, geomId);
  objectList_.push_back(capsule);
  return capsule;
}

object::MjcCylinder *MjcWorld::addCylinder(double radius,
                                     double height,
                                     double mass,
                                     int bodyId,
                                     int geomId) {
  object::MjcCylinder *cylinder = new object::MjcCylinder(radius, height, worldData_, worldModel_, bodyId, geomId);
  objectList_.push_back(cylinder);
  return cylinder;
}

/**
 * @brief 获取系统的自由度 (DOF)。
 *
 * @return int 返回系统的自由度。
 */

int MjcWorld::getDOF() {
  return dof_;
}

/**
 * @brief 获取广义坐标的维度。
 *
 * @return int 返回广义坐标的维度。
 */
int MjcWorld::getGeneralizedCoordinateDim() {
  return dimGenCoord_;
}

/**
 * @brief 获取当前的广义坐标。
 *
 * @return const EigenVec 返回当前的广义坐标向量。返回的一个列向量
 */
const EigenVec MjcWorld::getGeneralizedCoordinate() {
  for(int i = 0; i < dimGenCoord_; i++)
    generalizedCoordinate_[i] = worldData_->qpos[i];  // 从模拟数据中获取当前的广义坐标
  return generalizedCoordinate_.e();
}

/**
 * @brief 获取当前的广义速度。
 *
 * @return const EigenVec 返回当前的广义速度向量。
 */
const EigenVec MjcWorld::getGeneralizedVelocity() {
  // 从模拟数据获取当前广义速度
  for(int i = 0; i < dof_; i++)
    generalizedVelocity_[i] = worldData_->qvel[i];
  return generalizedVelocity_.e();
}

/**
 * @brief 获取当前的广义力。
 *
 * @return const EigenVec 返回当前的广义力向量。
 */
const EigenVec MjcWorld::getGeneralizedForce() {
  // 从模拟数据中获取当前的广义力
  for(int i = 0; i < dof_; i++) {
    generalizedForce_[i] = worldData_->qfrc_applied[i];
  }
  return generalizedForce_.e();
}

/**
 * @brief 设置广义坐标。
 *
 * @param jointState 新的广义坐标向量。
 * @throws 如果输入的广义坐标维度无效则触发异常。
 */
void MjcWorld::setGeneralizedCoordinate(const Eigen::VectorXd &jointState) {
  /** 检查输入的广义坐标维度是否有效 */
  RAIFATAL_IF(jointState.size() != dimGenCoord_, "invalid generalized coordinate input")
  for(int i = 0; i < dimGenCoord_; i++) {
    generalizedCoordinate_[i] = jointState[i];
    worldData_->qpos[i] = jointState[i];
  }
}

/**
 * 设置广义坐标，接受一个初始化列表作为输入。
 * @param jointState 一个包含广义坐标值的初始化列表。
 */
void MjcWorld::setGeneralizedCoordinate(std::initializer_list<double> jointState) {
    // 检查输入的广义坐标大小是否与模型的广义坐标维度匹配
    RAIFATAL_IF(jointState.size() != dimGenCoord_, "invalid generalized coordinate input")
    // 遍历广义坐标并将其设置到模型中
    for(int i = 0; i < dimGenCoord_; i++) {
        generalizedCoordinate_[i] = jointState.begin()[i];
        worldData_->qpos[i] = jointState.begin()[i];    // mjModel 需要同时更新
    }
}

/**
 * 设置广义速度，接受一个Eigen::VectorXd作为输入。
 * @param jointVel 一个包含广义速度值的Eigen向量。
 */
void MjcWorld::setGeneralizedVelocity(const Eigen::VectorXd &jointVel) {
  RAIFATAL_IF(jointVel.size() != dof_, "invalid generalized velocity input")
  for(int i = 0; i < dof_; i++) {
    generalizedVelocity_[i] = jointVel[i];
    worldData_->qvel[i] = jointVel[i];
  }
}

/**
 * 设置广义速度，接受一个初始化列表作为输入。
 * @param jointVel 一个包含广义速度值的初始化列表。
 */
void MjcWorld::setGeneralizedVelocity(std::initializer_list<double> jointVel) {
  RAIFATAL_IF(jointVel.size() != dof_, "invalid generalized velocity input")
  for(int i = 0; i < dof_; i++) {
    generalizedVelocity_[i] = jointVel.begin()[i];
    worldData_->qvel[i] = jointVel.begin()[i];
  }
}

/**
 * 设置广义力，接受一个初始化列表作为输入。
 * @param tau 一个包含广义力值的初始化列表。
 */
void MjcWorld::setGeneralizedForce(std::initializer_list<double> tau) {
  RAIFATAL_IF(tau.size() != dof_, "invalid generalized force input")
  for(int i = 0; i < dof_; i++) {
    generalizedForce_[i] = tau.begin()[i];
    worldData_->qfrc_applied[i] = tau.begin()[i];
  }
}

/**
 * 设置广义力，接受一个Eigen::VectorXd作为输入。
 * @param tau 一个包含广义力值的Eigen向量。
 */
void MjcWorld::setGeneralizedForce(const Eigen::VectorXd &tau) {
  RAIFATAL_IF(tau.size() != dof_, "invalid generalized force input")
  for(int i = 0; i < dof_; i++) {
    generalizedForce_[i] = tau[i];
    worldData_->qfrc_applied[i] = tau[i];
  }
}

/**
 * 获取当前状态，包括广义坐标和广义速度。
 * @param genco 用于存储广义坐标的Eigen向量。
 * @param genvel 用于存储广义速度的Eigen向量。
 */
void MjcWorld::getState(Eigen::VectorXd &genco, Eigen::VectorXd &genvel) {
  RAIFATAL_IF(genco.size() != dimGenCoord_, "invalid generalized coordinate input")
  RAIFATAL_IF(genvel.size() != dof_, "invalid generalized velocity input")

   /** 获取当前的广义速度 */
  for(int i = 0; i < dof_; i++) {
    generalizedVelocity_[i] = worldData_->qvel[i];
    genvel[i] = generalizedVelocity_[i];
  }

   /** 获取当前的广义坐标 */
  for(int i = 0; i < dof_; i++) {
    generalizedCoordinate_[i] = worldData_->qpos[i];
    genco[i] = generalizedCoordinate_[i];
  }
}

/**
 * 设置当前状态，包括广义坐标和广义速度。
 * @param genco 一个包含广义坐标值的Eigen向量。
 * @param genvel 一个包含广义速度值的Eigen向量。
 */
void MjcWorld::setState(const Eigen::VectorXd &genco, const Eigen::VectorXd &genvel) {

  // 检查输入的广义坐标大小是否与模型的广义坐标维度匹配
  RAIFATAL_IF(genco.size() != dimGenCoord_, "invalid generalized coordinate input")
  // 检查输入的广义速度大小是否与模型的自由度匹配
  RAIFATAL_IF(genvel.size() != dof_, "invalid generalized velocity input")

  // 设置广义速度
  for(int i = 0; i < dof_; i++) {
    generalizedVelocity_[i] = genvel[i];
    worldData_->qvel[i] = generalizedVelocity_[i];
  }

  // 设置广义坐标
  for(int i = 0; i < dof_; i++) {
    generalizedCoordinate_[i] = genco[i];
    worldData_->qpos[i] = generalizedCoordinate_[i];
  }
}

/**
 * 设置当前的重力向量。
 * @param gravity 一个包含重力分量的Vec对象，通常表示为三个分量（x, y, z）。
 */
void MjcWorld::setGravity(const benchmark::Vec<3> &gravity) {
  simOption_->gravity[0] = gravity[0];
  simOption_->gravity[1] = gravity[1];
  simOption_->gravity[2] = gravity[2];
}

/**
 * 获取当前世界中的接触数量。
 * @return 当前世界中的接触数量。
 */
int MjcWorld::getWorldNumContacts() {
  return worldData_->ncon;
}

/**
 * 获取当前世界中的物体数量。
 * @return 当前物体的数量。
 */
int MjcWorld::getNumObject() {
  return objectList_.size();
}

/**
 * 设置无滑移参数的最大迭代次数。
 * @param maxIter 最大迭代次数，控制无滑移算法的精度。
 */
void MjcWorld::setNoSlipParameter(int maxIter) {
  simOption_->noslip_iterations = maxIter;
}

/**
 * 设置模拟的时间步长。
 * @param timeStep 模拟的时间步长，控制每次仿真更新的时间间隔。
 */
void MjcWorld::setTimeStep(double timeStep) {
  worldModel_->opt.timestep = timeStep;
}

/**
 * 执行一次模拟步骤，更新世界状态并处理接触问题。
 * 此函数清除现有的接触问题列表，然后根据当前的接触数据填充接触问题列表。
 * 最后，调用 MuJoCo 的步进函数 `mj_step` 来更新模拟状态。
 */
void MjcWorld::integrate() {

  /** 清空接触问题列表 */
  contactProblemList_.clear();

  /** 为接触问题列表保留足够的空间 */
  contactProblemList_.reserve(worldData_->ncon);

  /** 遍历当前接触数量 */
  for(int i = 0; i < worldData_->ncon; i++) {
    contactProblemList_.emplace_back(
        worldData_->contact[i].pos[0], worldData_->contact[i].pos[1], worldData_->contact[i].pos[2]);
  }

  /** 更新模拟状态 */
  mj_step(worldModel_, worldData_);
}

/**
 * 执行一次模拟步骤，更新世界状态并处理接触问题（使用 mj_step1）。
 * 此函数清除现有的接触问题列表，然后根据当前的接触数据填充接触问题列表。
 * 最后，调用 MuJoCo 的步进函数 `mj_step1` 来更新模拟状态。
 */
void MjcWorld::integrate1() {

  /** 清空接触问题列表 */
  contactProblemList_.clear();

  /** 为接触问题列表保留足够的空间 */
  contactProblemList_.reserve(worldData_->ncon);
  for(int i = 0; i < worldData_->ncon; i++) {
    contactProblemList_.emplace_back(
        worldData_->contact[i].pos[0], worldData_->contact[i].pos[1], worldData_->contact[i].pos[2]);
  }

  /** 使用 mj_step1 更新模拟状态 */
  mj_step1(worldModel_, worldData_);
}


void MjcWorld::integrate2() {
  mj_step2(worldModel_, worldData_);
}

/**
 * @brief 计算世界中所有可移动物体的线性动量，并返回其在笛卡尔坐标系中的表示。
 *
 * @return 返回一个包含线性动量的三维向量（Eigen::Vector3d），
 *         该向量表示世界中所有可移动物体的线性动量总和。
 */
const Eigen::Map<Eigen::Matrix<double, 3, 1>> MjcWorld::getLinearMomentumInCartesianSpace() {

  /** 初始化线性动量向量为零 */
  Eigen::Vector3d linearMomentum;
  linearMomentum.setZero();

  /** 遍历所有物体 */
  for(int i = 0; i < objectList_.size(); i++) {
    // 检查物体是否可以移动
    if(!static_cast<object::MjcSingleBodyObject *>(objectList_[i])->isMovable()) continue;
    // 计算该物体的线性动量并累加到总线性动量
    linearMomentum += objectList_[i]->getMass() * objectList_[i] -> getLinearVelocity();
  }

  /** 将计算得到的线性动量保存到成员变量 */
  linearMomentum_ = {linearMomentum.x(), linearMomentum.y(), linearMomentum.z()};
  return linearMomentum_.e();
}

/** 获取物体质量 */
double MjcWorld::getTotalMass() {
  double mass = 0;
  for(int i = 0; i < objectList_.size(); i++) {
    if(!static_cast<object::MjcSingleBodyObject *>(objectList_[i])->isMovable()) continue;
    mass += objectList_[i]->getMass();
  }
  return mass;
}


/** deprecated */
void mujoco_sim::MjcWorld::integrate(double dt) {
  RAIFATAL("use setTimeStep(double dt) + integrate() instead")
}
void MjcWorld::integrate1(double dt) {
  RAIFATAL("use setTimeStep(double dt) + integrate1() instead")
}
void MjcWorld::integrate2(double dt) {
  RAIFATAL("use setTimeStep(double dt) + integrate2() instead")
}


/**
 * @brief 执行前向运动学计算以更新世界中所有物体的状态（位置和朝向）。
 *
 * 该函数调用 MuJoCo 库的 `mj_forward` 函数来计算模型的前向运动学。
 * 在前向运动学中，系统的配置（即关节的位置和姿态）用于计算每个物体在空间中的位置和方向。
 */
void MjcWorld::forwardKinematics() {
  mj_forward(worldModel_, worldData_);
}

/**
 * @brief 重置仿真状态并执行前向运动学计算以更新模型状态。
 *
 * 该函数首先调用 `mj_resetData` 函数重置世界数据，然后再调用 `mj_forward`
 * 函数执行前向运动学计算，以确保所有物体的位置和状态被正确初始化。
 */
void MjcWorld::resetSimulation() {
  mj_resetData(worldModel_, worldData_);
  mj_forward(worldModel_, worldData_);
}

/**
 * @brief 获取当前的接触问题列表。
 *
 * @return 返回指向当前接触问题列表的常量指针，该列表包含在当前仿真步骤中
 *         检测到的所有接触问题（`Single3DContactProblem` 类型的对象）。
 */
const std::vector<Single3DContactProblem> *MjcWorld::getCollisionProblem() const {
  return &contactProblemList_;
}

} // namespace mujoco_sim
