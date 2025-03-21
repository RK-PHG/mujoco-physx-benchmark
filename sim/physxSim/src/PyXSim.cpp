/** PyXSim  */

#include "PyXSim.hpp"

using namespace physx_sim;

PyXSim::PyXSim(int windowWidth,
               int windowHeight,
               float cms,
               int flags) :
        benchmark::WorldRG(windowWidth, windowHeight, cms, flags) {
    world_ = new PyXWorld();
}

PyXSim::PyXSim() {
    world_ = new PyXWorld();
}

PyXSim::PyXSim(const std::string &modelPath) {
    world_ = new PyXWorld();
    world_->loadModel(modelPath);
}

PyXSim::~PyXSim() {
    delete world_;
}

/**
 * 添加球
 */
benchmark::SingleBodyHandle PyXSim::addSphere(double radius,
                                              double mass,
                                              int bodyId,
                                              int geomId){
    object::PyXSphere* sphere = world_->addSphere(radius,mass,{0.0f,0.0f,0.0f});
    benchmark::SingleBodyHandle handle(sphere, {}, {});

    if (gui_) {
        handle.visual().push_back(new rai_graphics::object::Sphere(radius, true));
    }

    sbHandles_.push_back(handle);

    for (auto *go: handle.visual())
        processGraphicalObject(go, 0);

    for (auto *av: handle.alternateVisual())
        processGraphicalObject(av, 0);

    if(gui_) framesAndCOMobj_.push_back(handle.s_);
    return handle;
}

/**
 * 添加盒体
 */
benchmark::SingleBodyHandle PyXSim::addBox(double xLength,
                                           double yLength,
                                           double zLength,
                                           double mass,
                                           int bodyId,
                                           int geomId) {
    object::PyXBox *box = world_->addBox(xLength, yLength, zLength, mass, {0.0f, 0.0f, 0.0f});
    benchmark::SingleBodyHandle handle(box, {}, {});

    if (gui_) {
        handle.visual().push_back(new rai_graphics::object::Box(xLength, yLength, zLength, true));
    }

    sbHandles_.push_back(handle);

    for (auto *go: handle.visual()) {
        processGraphicalObject(go, 0);
    }

    for (auto *av: handle.alternateVisual())
        processGraphicalObject(av, 0);

    if (gui_) framesAndCOMobj_.push_back(handle.s_);
    return handle;
}

/**
 * 添加地面
 */
benchmark::SingleBodyHandle PyXSim::addCheckerboard(double gridSize,
                                                    double xLength,
                                                    double yLength,
                                                    double reflectanceI,
                                                    bo::CheckerboardShape shape,
                                                    int bodyId,
                                                    int geomId,
                                                    int flags){
    object::PyXCheckerBoard* checkerBoard = world_->addCheckerboard(benchmark::Vec<3>({0.0f,0.0f,1.0f}));
    benchmark::SingleBodyHandle handle(checkerBoard,{},{});
    handle.hidable = false;
    if(gui_) {
        handle.visual().push_back(new rai_graphics::object::CheckerBoard(gridSize, xLength, yLength, reflectanceI));
        static_cast<rai_graphics::object::CheckerBoard *>(handle.visual()[0])->gridMode = flags & bo::GRID;
        gui_->addCheckerBoard(static_cast<rai_graphics::object::CheckerBoard *>(handle.visual()[0]));
    }
    sbHandles_.push_back(handle);
    return handle;
}

/**
 * 添加胶囊体
 */
benchmark::SingleBodyHandle PyXSim::addCapsule(double radius,
                                               double height,
                                               double mass,
                                               int bodyId,
                                               int geomid) {
    object::PyXCapsule *capsule = world_->addCapsule(radius, height, mass, {0.0f, 0.0f, 0.0f});
    benchmark::SingleBodyHandle handle(capsule, {}, {});

    if (gui_) {
        handle.visual().push_back(new rai_graphics::object::Capsule(radius, height, true));
    }

    sbHandles_.push_back(handle);

    for (auto *go: handle.visual()) {
        processGraphicalObject(go, 0);
    }

    for (auto *av: handle.alternateVisual())
        processGraphicalObject(av, 0);

    if (gui_) framesAndCOMobj_.push_back(handle.s_);
    return handle;

}

/**
 * 添加关节系统
 */
ArticulatedSystemHandle  PyXSim::addArticulatedSystem(physx_sim::object::PyXArticulatedSystem* articulatedSystem){

    ArticulatedSystemHandle handle(
            articulatedSystem, {}, {});
    if(!gui_) {
        asHandles_.push_back(handle);
        return handle;
    }

    for (int i = 0; i < handle->visObj.size(); i++) {
        switch (std::get<3>(handle->visObj[i])) {
            case benchmark::object::Shape::Box:
                handle.visual().push_back(new rai_graphics::object::Box(handle->visProps_[i].second.v[0],
                                                                        handle->visProps_[i].second.v[1],
                                                                        handle->visProps_[i].second.v[2], true));
                break;
            case benchmark::object::Shape::Cylinder:
                handle.visual().push_back(new rai_graphics::object::Cylinder(handle->visProps_[i].second.v[0],
                                                                             handle->visProps_[i].second.v[1], true));
                break;
            case benchmark::object::Shape::Sphere:
                handle.visual().push_back(new rai_graphics::object::Sphere(handle->visProps_[i].second.v[0], true));
                break;
            case benchmark::object::Shape::Mesh:
                checkFileExistance(handle->visProps_[i].first);
                handle.visual().push_back(new rai_graphics::object::Mesh(handle->visProps_[i].first,
                                                                         handle->visProps_[i].second.v[0]));
                break;
        }
        handle.visual().back()->setColor({float(std::get<4>(handle->visObj[i]).v[0]),
                                          float(std::get<4>(handle->visObj[i]).v[1]),
                                          float(std::get<4>(handle->visObj[i]).v[2])});
        processGraphicalObject(handle.visual().back(), std::get<2>(handle->visObj[i]));
    }
    asHandles_.push_back(handle);
    return handle;
}

/**
 * 更新显示帧，包括更新单体物体和机器人
 */
void PyXSim::updateFrame() {

    RAIFATAL_IF(!gui_, "use different constructor for visualization")
    const bool showAlternateGraphicsIfexists = gui_->getCustomToggleState(3);

    for (auto &as: asHandles_) {

        benchmark::Vec<4> quat;
        benchmark::Vec<3> pos;
        benchmark::Vec<4> color;

        as->updateVisuals();
        std::cout << as.visual().size() << std::endl;

        if (showAlternateGraphicsIfexists) {

            for (int i = 0; i < as->getVisColOb().size(); i++) {
                as.alternateVisual()[i]->setVisibility(true);
                pos = std::get<1>(as->getVisColOb()[i]);
                as.alternateVisual()[i]->setPos(
                        pos[0],
                        pos[1],
                        pos[2]);
                rotMatToQuat(std::get<0>(as->getVisColOb()[i]), quat);
                as.alternateVisual()[i]->setOri(quat.v[0], quat.v[1], quat.v[2], quat.v[3]);
                adjustTransparency(as.alternateVisual()[i], as.hidable);
            }

            for (int i = 0; i < as->getVisOb().size(); i++)
                as.visual()[i]->setVisibility(false);

        } else {
            for (int i = 0; i < as->getVisOb().size(); i++) {
                as.visual()[i]->setVisibility(true);
                if (!as.visual()[i]->isVisible()) continue;
                pos = std::get<1>(as->getVisOb()[i]);
                color = std::get<4>(as->getVisOb()[i]);
                as.visual()[i]->setPos(
                        pos[0],
                        pos[1],
                        pos[2]
                );
                rotMatToQuat(std::get<0>(as->getVisOb()[i]), quat);
                as.visual()[i]->setOri(quat.v[0], quat.v[1], quat.v[2], quat.v[3]);
                as.visual()[i]->setColor({float(color[0]),
                                          float(color[1]),
                                          float(color[2])});
                as.visual()[i]->setTransparency(float(color[3]));
                adjustTransparency(as.visual()[i], as.hidable);
            }
            for (int i = 0; i < as->getVisColOb().size(); i++)
                as.alternateVisual()[i]->setVisibility(false);
        }
    }

    benchmark::Vec<3> bodyPosition;
    benchmark::Vec<4> quat;

    for (auto sb: sbHandles_) {

        sb->getPosition_W(bodyPosition);
        sb->getQuaternion(quat);

        if (!showAlternateGraphicsIfexists || sb.alternateVisual().size() == 0) {
            for (auto *go: sb.alternateVisual()) go->setVisibility(false);
            for (auto *go: sb.visual()) {
                go->setVisibility(true);
                go->setPos({bodyPosition.v[0], bodyPosition.v[1], bodyPosition.v[2]});
                go->setOri(quat.v[0], quat.v[1], quat.v[2], quat.v[3]);
                adjustTransparency(go, sb.hidable);
            }
        } else {
            for (auto *go: sb.visual()) go->setVisibility(false);
            for (auto *go: sb.visual()) {
                go->setVisibility(true);
                go->setPos({bodyPosition.v[0], bodyPosition.v[1], bodyPosition.v[2]});
                go->setOri(quat.v[0], quat.v[1], quat.v[2], quat.v[3]);
                adjustTransparency(go, sb.hidable);
            }
        }
    }

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

    if (visualizerFlags_ & benchmark::DISABLE_INTERACTION)
        return;
}

// 静摩擦
void PyXSim::setNoSlipParameter(double friction) {
    world_->setNoSlipParameter(friction);
}

// 设置重力
void PyXSim::setGravity(Eigen::Vector3d gravity) {
    world_->setGravity(benchmark::Vec<3>({gravity.x(), gravity.y(), gravity.z()}));
}

// 仿真步长
void PyXSim::setTimeStep(double timeStep) {
    world_->setTimeStep(timeStep);
}

// 获取单体的Handle
benchmark::SingleBodyHandle PyXSim::getSingleBodyHandle(int index) {
    return benchmark::SingleBodyHandle(world_->objectList_.at(index), {}, {});
}

// 获取接触点数量
int PyXSim::getWorldNumContacts() {
    return world_->getWorldNumContacts();
}

// 获取单体数量
int PyXSim::getNumObject() {
    return world_->getNumObject();
}

// 获取线性动量
const Eigen::Map<Eigen::Matrix<double, 3, 1>> PyXSim::getLinearMomentumInCartesianSpace() {
    return world_->getLinearMomentumInCartesianSpace();
}

// 获取总质量
double PyXSim::getTotalMass() {
    return world_->getTotalMass();
}

// 获取系统总能量
double PyXSim::getEnergy(const benchmark::Vec<3> &gravity) {
    return world_->getEnergy(gravity);
}

// 获取系统动能
double PyXSim::getKineticEnergy() {
    return world_->getKineticEnergy();
}

// 获取系统重力势能
double PyXSim::getPotentialEnergy(const benchmark::Vec<3> &gravity) {
    return world_->getPotentialEnergy(gravity);
}

void PyXSim::loop(double dt, double realTimeFactor) {}

void PyXSim::integrate() {
    world_->integrate();
}
