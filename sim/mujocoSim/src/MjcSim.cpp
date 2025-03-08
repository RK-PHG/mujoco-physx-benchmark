/** MjcSim.hpp的函数实现 */

#include "MjcSim.hpp"

// the mass data is from XML
#define MJC_MASS_FROM_XML -1

/** 支持gui的构造函数 */
mujoco_sim::MjcSim::MjcSim(int windowWidth,
                                     int windowHeight,
                                     float cms,
                                     const char *modelPath,
                                     const char *keyPath,
                                     int flags,
                                     SolverOption solver,
                                     IntegratorOption integrator) :
    world_(modelPath, keyPath, solver, integrator),
    benchmark::WorldRG(windowWidth, windowHeight, cms, flags) {
  initFromModel();
}

/** 无可视化界面的构造函数 */
mujoco_sim::MjcSim::MjcSim(const char *modelPath,
                                     const char *keyPath,
                                     SolverOption solver,
                                     IntegratorOption integrator) :
    world_(modelPath, keyPath, solver, integrator),
    benchmark::WorldRG() {
  initFromModel();
}

/**
 * @brief 从 Mujoco 模型初始化场景中的对象。
 *
 * 此函数遍历 Mujoco 模型的所有身体和几何体，并根据几何体的类型创建相应的对象。
 */
void mujoco_sim::MjcSim::initFromModel() {

    /** 获取当前的 Mujoco 模型 */
  mjModel *model = world_.getWorldModel();

   /** 遍历模型中所有物体 */
  for(int i = 0; i < model->nbody; i++) {

    /** 获取当前身体的几何体数量 */
    const int geomNumInBody = model->body_geomnum[i];

    /** 获取当前身体的几何体地址 */
    const int geomAddrInBody = model->body_geomadr[i];

    /** 获取模型名称 */
    std::stringstream bodyname;
    for(int j = model->name_bodyadr[i]; i+1 <= model->nbody && j < model->name_bodyadr[i+1] - 1; j++) {
      bodyname << model->names[j];
    }

    if(bodyname.str() == "ground") {
      // checkerboard if the name of the body is ground
      addCheckerboard(2, 100, 100, 0.1, bo::PLANE_SHAPE, i, 0, bo::GRID);
      return;
    }

    for(int j = 0; j < geomNumInBody; j++) {

      /** 计算当前几何体的索引，geomAddrInBody 是当前身体的几何体起始地址 */
      const int geomIndex = geomAddrInBody + j;

      /** 从模型中获取当前几何体的尺寸，geomSize 数组包含 3 个元素
       *对于不同几何体类型，geomSize 的含义不同
       */
      mjtNum *geomSize = model->geom_size + geomIndex * 3;

      /** 获取当前几何体的类型 */
      int geomType = model->geom_type[geomIndex];

      switch (geomType) {
        case mjGEOM_PLANE: {
          // geomsize = (xlength, ylength, gridsize)
          addCheckerboard(geomSize[2], geomSize[0], geomSize[1], 0.1, bo::PLANE_SHAPE, i, j, bo::GRID);
          break;
        }
        case mjGEOM_SPHERE: {
          addSphere(geomSize[0], MJC_MASS_FROM_XML, i, j);
          break;
        }
        case mjGEOM_CAPSULE: {
          // geomsize = (radius_, height * 0.5)
          addCapsule(geomSize[0], geomSize[1] * 2, MJC_MASS_FROM_XML, i, j);
          break;
        }
        case mjGEOM_BOX: {
          // geomsize = (xlength, ylength, zlength)
          addBox(geomSize[0] * 2, geomSize[1] * 2, geomSize[2] * 2, MJC_MASS_FROM_XML, i, j);
          break;
        }
        case mjGEOM_CYLINDER: {
          // geomsize = (radius_, height * 0.5)
          addCylinder(geomSize[0], geomSize[1] * 2, MJC_MASS_FROM_XML, i, j);
          break;
        }
        case mjGEOM_ELLIPSOID: {
          RAIFATAL("ellpisoid geometry is not supported.")
          break;
        }
        case mjGEOM_NONE: {
          RAIFATAL("invalid geometry type")
          break;
        }
        default: {
          RAIFATAL("not supported geometry type");
        }
      }

      /** 设置摩擦系数 */
      model->geom_friction[3 * geomIndex] = 0.8;      // 设置当前几何体的静摩擦系数为 0.8
      model->geom_friction[3 * geomIndex + 1] = 0;    // 设置当前几何体的滑动摩擦系数为 0
      model->geom_friction[3 * geomIndex + 2] = 0;    // 设置当前几何体的弹性摩擦系数为 0
    }
  }
}

mujoco_sim::MjcSim::~MjcSim() {

}

/** 添加球形 */
benchmark::SingleBodyHandle mujoco_sim::MjcSim::addSphere(double radius,
                                                            double mass,
                                                            int bodyId,
                                                            int geomId) {
  benchmark::SingleBodyHandle handle(world_.addSphere(radius, mass, bodyId, geomId), {}, {});
  /** 如果支持gui，设置可视化信息 */
  if(gui_) {
      handle.visual().push_back(new rai_graphics::object::Sphere(radius, true));
  }
  processSingleBody(handle);
  return handle;
}

benchmark::SingleBodyHandle mujoco_sim::MjcSim::addBox(double xLength,
                                                         double yLength,
                                                         double zLength,
                                                         double mass,
                                                         int bodyId,
                                                         int geomId) {
  benchmark::SingleBodyHandle handle(world_.addBox(xLength, yLength, zLength, mass, bodyId, geomId), {}, {});
  if(gui_) handle.visual().push_back(new rai_graphics::object::Box(xLength, yLength, zLength, true));
  processSingleBody(handle);
  return handle;
}

benchmark::SingleBodyHandle mujoco_sim::MjcSim::addCheckerboard(double gridSize,
                                                                  double xLength,
                                                                  double yLength,
                                                                  double reflectanceI,
                                                                  bo::CheckerboardShape shape,
                                                                  int bodyId,
                                                                  int geomId,
                                                                  int flags) {
  benchmark::SingleBodyHandle handle(world_.addCheckerboard(gridSize, xLength, yLength, reflectanceI, shape, bodyId, geomId), {}, {});
  handle.hidable = false;
  if(gui_) {
    handle.visual().push_back(new rai_graphics::object::CheckerBoard(gridSize, xLength, yLength, reflectanceI));
    static_cast<rai_graphics::object::CheckerBoard *>(handle.visual()[0])->gridMode = flags & bo::GRID;
    gui_->addCheckerBoard(static_cast<rai_graphics::object::CheckerBoard *>(handle.visual()[0]));
  }
  sbHandles_.push_back(handle);
  return handle;
}

benchmark::SingleBodyHandle mujoco_sim::MjcSim::addCapsule(double radius,
                                                             double height,
                                                             double mass,
                                                             int bodyId,
                                                             int geomid) {
  benchmark::SingleBodyHandle handle(world_.addCapsule(radius, height, mass, bodyId, geomid), {}, {});
  if(gui_) handle.visual().push_back(new rai_graphics::object::Capsule(radius, height, true));
  processSingleBody(handle);
  return handle;
}

benchmark::SingleBodyHandle mujoco_sim::MjcSim::addCylinder(double radius,
                                                              double height,
                                                              double mass,
                                                              int bodyId,
                                                              int geomId) {
  benchmark::SingleBodyHandle handle(world_.addCylinder(radius, height, mass, bodyId, geomId), {}, {});
  if(gui_) handle.visual().push_back(new rai_graphics::object::Cylinder(radius, height, true));
  processSingleBody(handle);
  return handle;
}

void mujoco_sim::MjcSim::setGravity(Eigen::Vector3d gravity) {
  world_.setGravity({gravity[0], gravity[1], gravity[2]});
}

void mujoco_sim::MjcSim::setERP(double erp, double erp2, double frictionErp) {
  RAIFATAL("no erp for mujoco")
}

benchmark::SingleBodyHandle mujoco_sim::MjcSim::getSingleBodyHandle(int index) {
  if(index > sbHandles_.size())
  RAIFATAL("get singlebody handle failed. invalid index");
  return sbHandles_[index];
}
const mujoco_sim::EigenVec mujoco_sim::MjcSim::getGeneralizedCoordinate() {
  return world_.getGeneralizedCoordinate();
}
const mujoco_sim::EigenVec mujoco_sim::MjcSim::getGeneralizedVelocity() {
  return world_.getGeneralizedVelocity();
}
void mujoco_sim::MjcSim::setGeneralizedCoordinate(const Eigen::VectorXd &jointState) {
  world_.setGeneralizedCoordinate(jointState);
}
void mujoco_sim::MjcSim::setGeneralizedVelocity(const Eigen::VectorXd &jointVel) {
  world_.setGeneralizedVelocity(jointVel);
}
void mujoco_sim::MjcSim::setGeneralizedCoordinate(std::initializer_list<double> jointState) {
  world_.setGeneralizedCoordinate(jointState);
}
void mujoco_sim::MjcSim::setGeneralizedVelocity(std::initializer_list<double> jointVel) {
  world_.setGeneralizedVelocity(jointVel);
}
void mujoco_sim::MjcSim::setGeneralizedForce(std::initializer_list<double> tau) {
  world_.setGeneralizedForce(tau);
}
void mujoco_sim::MjcSim::setGeneralizedForce(const Eigen::VectorXd &tau) {
  world_.setGeneralizedForce(tau);
}
void mujoco_sim::MjcSim::getState(Eigen::VectorXd &genco, Eigen::VectorXd &genvel) {
  world_.getState(genco, genvel);
}
void mujoco_sim::MjcSim::setState(const Eigen::VectorXd &genco, const Eigen::VectorXd &genvel) {
  world_.setState(genco, genvel);
}
const mujoco_sim::EigenVec mujoco_sim::MjcSim::getGeneralizedForce() {
  return world_.getGeneralizedForce();
}
int mujoco_sim::MjcSim::getDOF() {
  return world_.getDOF();
}
int mujoco_sim::MjcSim::getStateDimension() {
  return world_.getGeneralizedCoordinateDim();
}

int mujoco_sim::MjcSim::getWorldNumContacts() {
  return world_.getWorldNumContacts();
}
int mujoco_sim::MjcSim::getNumObject() {
  return world_.getNumObject();
}

void mujoco_sim::MjcSim::setNoSlipParameter(int maxiter) {
  world_.setNoSlipParameter(maxiter);
}

void mujoco_sim::MjcSim::setTimeStep(double timeStep) {
  timeStep_ = timeStep;
  world_.setTimeStep(timeStep);
}

void mujoco_sim::MjcSim::loop(double realTimeFactor) {
  while (visualizerLoop(timeStep_, realTimeFactor))
    integrate();
}

void mujoco_sim::MjcSim::integrate() {
  world_.integrate();
}

void mujoco_sim::MjcSim::integrate1() {
  world_.integrate1();
}

void mujoco_sim::MjcSim::integrate2() {
  world_.integrate2();
}

/**
 * 获取当前仿真世界中线动量的函数。
 *
 * @return 当前仿真世界中线动量的三维向量，表示在笛卡尔坐标系中的线动量。
 */
const Eigen::Map<Eigen::Matrix<double, 3, 1>> mujoco_sim::MjcSim::getLinearMomentumInCartesianSpace() {
  return world_.getLinearMomentumInCartesianSpace();
}

/**
 * 获取当前仿真世界中所有物体的总质量的函数。
 *
 * @return 当前仿真世界中所有物体的总质量，单位为千克 (kg)。
 */
double mujoco_sim::MjcSim::getTotalMass() {
  return world_.getTotalMass();
}

/**
 * 获取当前仿真世界的总能量（动能 + 势能）的函数。
 *
 * @param gravity 引力向量，表示重力方向和大小，类型为 benchmark::Vec<3>。
 * @return 当前仿真世界的总能量，单位为焦耳 (J)。
 */
double mujoco_sim::MjcSim::getEnergy(const benchmark::Vec<3> &gravity) {
  return getKineticEnergy() + getPotentialEnergy(gravity);
}

/**
 * 获取当前仿真世界中的动能的函数。
 *
 * @return 当前仿真世界的动能，单位为焦耳 (J)。
 */
double mujoco_sim::MjcSim::getKineticEnergy() {
  return world_.worldData_->energy[1];
}


double mujoco_sim::MjcSim::getPotentialEnergy(const benchmark::Vec<3> &gravity) {
  return world_.worldData_->energy[0];
}

void mujoco_sim::MjcSim::forwardKinematics() {
  world_.forwardKinematics();
}

void mujoco_sim::MjcSim::loop(double dt, double realTimeFactor) {
  RAIFATAL("use setTimeStep(double dt) + loop() instead")
}
void mujoco_sim::MjcSim::integrate(double dt) {
  RAIFATAL("use setTimeStep(double dt) + integrate() instead")
}
void mujoco_sim::MjcSim::integrate1(double dt) {
  RAIFATAL("use setTimeStep(double dt) + integrate1() instead")
}


void mujoco_sim::MjcSim::integrate2(double dt) {
  RAIFATAL("use setTimeStep(double dt) + integrate2() instead")
}

/**
 * 更新当前帧的可视化状态。
 * 该函数负责根据当前仿真状态更新可视化元素的位置、方向和可见性。
 */
void mujoco_sim::MjcSim::updateFrame() {

  /** 检查是否启用了 GUI 可视化 */
  RAIFATAL_IF(!gui_, "use different constructor for visualization")

  /** 获取自定义切换状态，决定是否显示备用图形 */
  const bool showAlternateGraphicsIfexists = gui_->getCustomToggleState(3);

  benchmark::Vec<3> bodyPosition;  // 存储身体的位置
  benchmark::Vec<4> quat;          // 存储身体的四元数（旋转

  /** 遍历所有单体手柄 */
  for (auto sb : sbHandles_) {
    sb->getPosition_W(bodyPosition); // 获取当前身体在世界坐标系中的位置
    sb->getQuaternion(quat);         // 获取当前身体的四元数

    /** 判断是否显示备用图形 */
    if (!showAlternateGraphicsIfexists || sb.alternateVisual().size() == 0) {
      // 如果没有备用图形，则隐藏备用视觉
      for (auto *go: sb.alternateVisual()) go->setVisibility(false);
      // 更新可视化元素的可见性、位置和方向
      for (auto *go: sb.visual()) {
        go->setVisibility(true);
        go->setPos({bodyPosition.v[0], bodyPosition.v[1], bodyPosition.v[2]});
        go->setOri(quat.v[0], quat.v[1], quat.v[2], quat.v[3]);
        adjustTransparency(go, sb.hidable);
      }
    } else {
      // 如果有备用图形，隐藏主视觉
      for (auto *go: sb.visual()) go->setVisibility(false);
      for (auto *go: sb.visual()) {
        go->setVisibility(true);
        go->setPos({bodyPosition.v[0], bodyPosition.v[1], bodyPosition.v[2]});
        go->setOri(quat.v[0], quat.v[1], quat.v[2], quat.v[3]);
        adjustTransparency(go, sb.hidable);
      }
    }
  }

//  rai_sim::Mat<3, 3> rot;
//  for (auto sb : comHandles_) {
//    sb->getPosition_W(bodyPosition);
//    sb->getRotationMatrix(rot);
//    auto &trans = sb->getTransformation();
//
//    if (!showAlternateGraphicsIfexists || sb.alternateVisual().size() == 0) {
//      for (auto *go: sb.alternateVisual()) go->setVisibility(false);
//      for (int i = 0; i < sb.visual().size(); i++) {
//        Vec<3> pos;
//        Vec<4> quat;
//        matvecmul(rot, trans[i].pos, pos);
//        sb.visual()[i]->setPos(bodyPosition.v[0] + pos.v[0],
//                               bodyPosition.v[1] + pos.v[1],
//                               bodyPosition.v[2] + pos.v[2]);
//        matmul(rot, trans[i].rot, rot);
//        rotMatToQuat(rot, quat);
//        sb.visual()[i]->setOri(quat.v[0], quat.v[1], quat.v[2], quat.v[3]);
//        adjustTransparency(sb.visual()[i], sb.hidable);
//      }
//    }
//  }

  /// contact points
  // 如果自定义切换状态开启，显示接触点
  if (gui_->getCustomToggleState(1)) {
    contactPointMarker_->mutexLock();
    contactPointMarker_->clearGhost();
    for (auto &pro: *world_.getCollisionProblem()) {
      Eigen::Vector3d pos = pro.point_;
      contactPointMarker_->addGhost(pos);
    }
    contactPointMarker_->mutexUnLock();
  } else
    contactPointMarker_->clearGhost();

  /// contact forces
//  if (gui_->getCustomToggleState(2)) {
//    double maxForce = 0;
//    for (auto &pro: *world_.getCollisionProblem())
//      maxForce = (maxForce < pro.imp_i.norm()) ? pro.imp_i.norm() : maxForce;
//    contactNormalArrow_->mutexLock();
//    contactNormalArrow_->clearGhost();
//    for (auto &pro: world_.getObjList()) {
//      for (auto &con: pro->getPerObjectContact().getContacts()) {
//        const double norm = con.getImpulse()->norm();
//        Eigen::Vector3d pos(con.getPosition().v[0], con.getPosition().v[1], con.getPosition().v[2]);
//        Vec<3> scaledImp;
//        matTransposevecmul(con.getContactFrame(), *con.getImpulse(), scaledImp);
//        vecScalarMul(1.0 / norm, scaledImp);
//        Eigen::Vector3d dir = scaledImp.e();
//        Eigen::Vector3f color(norm / maxForce, 0.2, 1 - norm / maxForce);
//        Eigen::Vector3f scale(norm / maxForce, 1, 1);
//        contactNormalArrow_->addGhostWithVector(pos, dir, color, scale);
//      }
//    }
//    contactNormalArrow_->mutexUnLock();
//  } else
//    contactNormalArrow_->clearGhost();

  /// frames and COM
  // 如果自定义切换状态开启，显示坐标系和质心
  if (gui_->getCustomToggleState(4)) {

    frameX_->mutexLock();
    frameY_->mutexLock();
    frameZ_->mutexLock();
    graphicalComMarker_->mutexLock();

    frameX_->clearGhost();
    frameY_->clearGhost();
    frameZ_->clearGhost();
    graphicalComMarker_->clearGhost();
    Eigen::Vector3f colorR(1, 0, 0), colorG(0, 1, 0), colorB(0, 0, 1);
    Eigen::Vector3d xdir, ydir, zdir;

    for (auto *cf: framesAndCOMobj_) {
      if (!cf->isVisualizeFramesAndCom()) continue;
      Eigen::Vector3d pos = cf->getPosition();
      Eigen::Matrix3d dir = cf->getRotationMatrix();
      Eigen::Vector3f scale(1, 1, 1);

      xdir = dir.col(0);
      ydir = dir.col(1);
      zdir = dir.col(2);

      frameX_->addGhostWithVector(pos, xdir, colorR, scale);
      frameY_->addGhostWithVector(pos, ydir, colorG, scale);
      frameZ_->addGhostWithVector(pos, zdir, colorB, scale);
      graphicalComMarker_->addGhost(pos);
    }
    frameX_->mutexUnLock();
    frameY_->mutexUnLock();
    frameZ_->mutexUnLock();
    graphicalComMarker_->mutexUnLock();
  } else {
    frameX_->clearGhost();
    frameY_->clearGhost();
    frameZ_->clearGhost();
    graphicalComMarker_->clearGhost();
  }

  if(visualizerFlags_ & benchmark::DISABLE_INTERACTION)
    return;

  /// interaction
//  if (gui_->isInteracting()) {
//    auto indices = interactionIdx_.find(gui_->getInteractingObjectID());
//    interactionForce_ = gui_->getInteractionMagnitude();
//    interactionForce_ *= interactionForce_.norm() * 20.0 * world_.getObjList()[indices->second.first]->getMass(indices->second.second);
//    objToInteract_ = world_.getObjList()[indices->second.first];
//    objToInteractLocalIdx_ = indices->second.second;
//    std::stringstream inStr;
//    inStr << std::setprecision(3) << interactionForce_.norm() << "N";
//    gui_->changeMenuText(1, false, inStr.str());
//    gui_->setMenuPositionNextToCursor(1);
//  } else {
//    gui_->changeMenuText(1, false, "");
//    objToInteract_ = nullptr;
//  }

  /// deletion
//  if (gui_->getKeyboardEvent(rai_graphics::KeyboardEvent::DEL)) {
//    auto indices = interactionIdx_.find(gui_->getInteractingObjectID());
//    if (indices != interactionIdx_.end()) {
//      object::Object *obj = world_.getObjList()[indices->second.first];
//      long int id =
//          std::find_if(sbHandles_.begin(), sbHandles_.end(), [obj](const SingleBodyHandle &a) { return a.s_ == obj; })
//              - sbHandles_.begin();
//      if (id == sbHandles_.size()) {
//        long int id = std::find_if(asHandles_.begin(),
//                                   asHandles_.end(),
//                                   [obj](const ArticulatedSystemHandle &a) { return a.s_ == obj; })
//            - asHandles_.begin();
//        if (!id == asHandles_.size())
//          removeObject(asHandles_[id]);
//      } else {
//        removeObject(sbHandles_[id]);
//      }
//      interactionIdx_.erase(gui_->getInteractingObjectID());
//    }
//  }
}
void mujoco_sim::MjcSim::resetSimulation() {
  world_.resetSimulation();
}

/** 设置世界的接触标志 */
void mujoco_sim::MjcSim::setWorldContactFlag(bool flagYN) {
  if (flagYN == true)
    world_.simOption_->disableflags &= ~mjDSBL_CONTACT;
  else
    world_.simOption_->disableflags |= mjDSBL_CONTACT;
}

/** 设置热启动标志 */
void mujoco_sim::MjcSim::setWarmStartFlag(bool flagYN) {
  if (flagYN == true)
    world_.simOption_->disableflags &= ~mjDSBL_WARMSTART;
  else
    world_.simOption_->disableflags |= mjDSBL_WARMSTART;
}

/** 设置求解器参数 */
void mujoco_sim::MjcSim::setSolverParameter(int solverMaxIteration, double solverTolerance) {
  world_.simOption_->iterations = solverMaxIteration;
  world_.simOption_->tolerance = solverTolerance;
}
