/** 关节系统 */
#include "PyXArticulatedSystem.hpp"
#include "PyXWorld.hpp"

using namespace physx_sim;
using namespace physx;

object::PyXArticulatedSystem::PyXArticulatedSystem(physx::PxArticulationReducedCoordinate *articulation)
        : dof_(articulation->getDofs()), isFixed_(false) {

    this->articulation_ = articulation;
    if(isFixed_){
        stateDimension_ = dof_;
    }
    else{
        dof_ = dof_ + 6;    // 底座的自由度+6
        stateDimension_ = dof_ + 1;  // 底座的旋转用4元数表示，+1
    }

    genCoordinate_.resize(stateDimension_);
    genCoordinate_.setZero();
    genVelocity_.resize(dof_);
    genVelocity_.resize(dof_);
    genForce_.resize(dof_);
    genForce_.resize(dof_);

}

object::PyXArticulatedSystem::~PyXArticulatedSystem() {}

int object::PyXArticulatedSystem::getDOF() {
    return dof_;
}

int object::PyXArticulatedSystem::getStateDimension() {
    return stateDimension_;
}


const EigenVec object::PyXArticulatedSystem::getGeneralizedCoordinate() {

    PxArticulationCache *cache = articulation_->createCache();
    articulation_->copyInternalStateToCache(*cache, PxArticulationCacheFlag::ePOSITION);

    if(isFixed_){
        for(int i = 0; i < stateDimension_; i++){
            genCoordinate_[i] = cache->jointPosition[i];
        }
    }else{
        PxTransform transform = articulation_->getRootGlobalPose();
        PxQuat quat = transform.q;
        PxVec3 pos = transform.p;
        //rotation
        genCoordinate_[3] = quat.w;
        genCoordinate_[4] = quat.x;
        genCoordinate_[5] = quat.y;
        genCoordinate_[6] = quat.z;
        // position
        genCoordinate_[0] = pos[0];
        genCoordinate_[1] = pos[1];
        genCoordinate_[2] = pos[2];
        for(int i = 7; i < stateDimension_; i++) {
            genCoordinate_[i] = cache->jointPosition[i-7];
        }
    }

    return genCoordinate_.e();

}

const EigenVec object::PyXArticulatedSystem::getGeneralizedVelocity(){

    PxArticulationCache* cache = articulation_->createCache();
    articulation_->copyInternalStateToCache(*cache,PxArticulationCacheFlag::eVELOCITY);

    if(isFixed_){
        for(int i = 0; i < dof_; ++i){
             genVelocity_[i] = cache->jointVelocity[i];
        }
    }else{
        PxVec3 linerVel = articulation_->getRootLinearVelocity();
        PxVec3 angVel = articulation_->getRootAngularVelocity();
        genVelocity_[0] = linerVel[0];
        genVelocity_[1] = linerVel[1];
        genVelocity_[2] = linerVel[2];
        genVelocity_[3] = angVel[0];
        genVelocity_[4] = angVel[1];
        genVelocity_[5] = angVel[2];
        for(int i = 6; i < dof_; i++){
            genVelocity_[i] = cache->jointVelocity[i-6];
        }
    }

    return genVelocity_.e();
}

const EigenVec object::PyXArticulatedSystem::getGeneralizedForce(){

    PxArticulationCache* cache = articulation_->createCache();
    articulation_->copyInternalStateToCache(*cache, PxArticulationCacheFlag::eFORCE);

    if(isFixed_){
        for(int i = 0; i < dof_; i++){
            genForce_[i] = cache->jointForce[i];
        }
    }else{
        int sz = articulation_->getNbLinks();
        std::vector<PxArticulationLink*> links(sz, nullptr);
        articulation_->getLinks(links.data(),sz,0);
        PxArticulationLink* root = links[0];

        PxVec3 force = root->getLinearAcceleration() * root->getMass();
        PxVec3 torque = root->getAngularAcceleration() * root->getMass();

        genForce_[0] = force[0];
        genForce_[1] = force[1];
        genForce_[2] = force[2];
        genForce_[3] = torque[0];
        genForce_[4] = torque[1];
        genForce_[5] = torque[2];

        for(int i = 6; i < dof_; i++) {
            genForce_[i] = cache->jointForce[i-6];
        }
    }

    return genForce_.e();
}

void object::PyXArticulatedSystem::setGeneralizedCoordinate(const Eigen::VectorXd &jointState){

    assert(jointState.size()==stateDimension_);

    PxArticulationCache* cache  = articulation_->createCache();
    articulation_->copyInternalStateToCache(*cache,PxArticulationCacheFlag::ePOSITION);

    if(isFixed_){
        for(int i=0; i<stateDimension_; i++){
            genCoordinate_[i] = jointState[i];
            cache->jointPosition[i] = jointState[i];
        }
    }else{
        // root position and rotation
        PxQuat q(jointState[4],jointState[5],jointState[6],jointState[3]);
        PxVec3 p(jointState[0],jointState[1],jointState[2]);
        PxTransform trans(p,q);
        articulation_->setRootGlobalPose(trans);
        for(int i = 0; i < stateDimension_; i++){
            genCoordinate_[i] = jointState[i];
            if(i > 6) {
                cache->jointPosition[i-7] = jointState[i];
            }
        }
    }

    articulation_->applyCache(*cache, PxArticulationCacheFlag::ePOSITION);
}

void object::PyXArticulatedSystem::setGeneralizedCoordinate(std::initializer_list<double> jointState){

    assert(jointState.size()==stateDimension_);

    PxArticulationCache* cache  = articulation_->createCache();
    articulation_->copyInternalStateToCache(*cache,PxArticulationCacheFlag::ePOSITION);

    if(isFixed_){
        for(int i=0; i<stateDimension_; i++){
            genCoordinate_[i] = jointState.begin()[i];
            cache->jointPosition[i] = jointState.begin()[i];
        }
    }else{
        // root position and rotation
        PxQuat q(jointState.begin()[4],jointState.begin()[5],jointState.begin()[6],jointState.begin()[3]);
        PxVec3 p(jointState.begin()[0],jointState.begin()[1],jointState.begin()[2]);
        PxTransform trans(p,q);
        articulation_->setRootGlobalPose(trans);

        // other joints
        for(int i = 0; i < stateDimension_; i++){
            genCoordinate_[i] = jointState.begin()[i];
            if(i > 6) {
                cache->jointPosition[i-7] = jointState.begin()[i];
            }
        }
    }

    articulation_->applyCache(*cache, PxArticulationCacheFlag::ePOSITION);
}


void object::PyXArticulatedSystem::setGeneralizedVelocity(const Eigen::VectorXd &jointVel) {

    assert(jointVel.size()==dof_);
    PxArticulationCache* cache  = articulation_->createCache();
    articulation_->copyInternalStateToCache(*cache,PxArticulationCacheFlag::eVELOCITY);
    if(isFixed_){
        for(int i=0; i<dof_; i++){
            genVelocity_[i] = jointVel[i];
            cache->jointVelocity[i] = jointVel[i];
        }
    }else{
        PxVec3 linerVel(jointVel[0], jointVel[1], jointVel[2]);
        PxVec3 angVel(jointVel[3], jointVel[4], jointVel[5]);
        articulation_->setRootLinearVelocity(linerVel);
        articulation_->setRootAngularVelocity(angVel);
        // other joints
        for(int i = 6; i < dof_; i++){
            cache->jointVelocity[i-6] = jointVel[i];
        }
    }

    articulation_->applyCache(*cache, PxArticulationCacheFlag::eVELOCITY);

}

void object::PyXArticulatedSystem::setGeneralizedVelocity(std::initializer_list<double> jointVel){

    assert(jointVel.size()==dof_);
    PxArticulationCache* cache  = articulation_->createCache();
    articulation_->copyInternalStateToCache(*cache,PxArticulationCacheFlag::eVELOCITY);
    if(isFixed_){
        for(int i=0; i<dof_; i++){
            genVelocity_[i] = jointVel.begin()[i];
            cache->jointVelocity[i] = jointVel.begin()[i];
        }
    }else{
        PxVec3 linerVel(jointVel.begin()[0], jointVel.begin()[1], jointVel.begin()[2]);
        PxVec3 angVel(jointVel.begin()[3], jointVel.begin()[4], jointVel.begin()[5]);
        articulation_->setRootLinearVelocity(linerVel);
        articulation_->setRootAngularVelocity(angVel);
        // other joints
        for(int i = 6; i < dof_; i++){
            cache->jointVelocity[i-6] = jointVel.begin()[i];
        }
    }

    articulation_->applyCache(*cache, PxArticulationCacheFlag::eVELOCITY);

}

void object::PyXArticulatedSystem::setGeneralizedForce(std::initializer_list<double> tau){

    PxArticulationCache* cache = articulation_->createCache();
    articulation_->copyInternalStateToCache(*cache, PxArticulationCacheFlag::eFORCE);
    if(isFixed_){
        for(int i = 0; i < dof_; i++){
            cache->jointForce[i] = tau.begin()[i];
        }
    }else{
        int sz = articulation_->getNbLinks();
        std::vector<PxArticulationLink*> links(sz, nullptr);
        articulation_->getLinks(links.data(),sz,0);
        PxArticulationLink* root = links[0];

        // 设置根节点力与扭矩
        PxVec3 force(tau.begin()[3], tau.begin()[4], tau.begin()[5]);
        PxVec3 torque(tau.begin()[0], tau.begin()[1], tau.begin()[2]);
        root->addForce(force);
        root->addTorque(torque);

        // 设置关节作用力
        for(int i = 6; i < dof_; i++){
            cache->jointForce[i-6] = tau.begin()[i];
        }
    }

    for(int i = 0; i < dof_; i++) {
        genForce_[i] = tau.begin()[i];
    }

    articulation_->applyCache(*cache, PxArticulationCacheFlag::eFORCE);
}

void object::PyXArticulatedSystem::setGeneralizedForce(const Eigen::VectorXd &tau){

    PxArticulationCache* cache = articulation_->createCache();
    articulation_->copyInternalStateToCache(*cache, PxArticulationCacheFlag::eFORCE);
    if(isFixed_){
        for(int i = 0; i < dof_; i++){
            cache->jointForce[i] = tau[i];
        }
    }else{
        // 获取根节点
        int sz = articulation_->getNbLinks();
        std::vector<PxArticulationLink*> links(sz, nullptr);
        articulation_->getLinks(links.data(),sz,0);
        PxArticulationLink* root = links[0];

        // 设置根节点力与扭矩
        PxVec3 force(tau[3], tau[4], tau[5]);
        PxVec3 torque(tau[0], tau[1], tau[2]);
        root->addForce(force);
        root->addTorque(torque);

        // 设置关节作用力
        for(int i = 6; i < dof_; i++){
            cache->jointForce[i-6] = tau[i];
        }
    }

    for(int i = 0; i < dof_; i++) {
        genForce_[i] = tau[i];
    }

    articulation_->applyCache(*cache, PxArticulationCacheFlag::eFORCE);
}


void object::PyXArticulatedSystem::getState(Eigen::VectorXd &genco, Eigen::VectorXd &genvel){
    genco = getGeneralizedCoordinate();
    genvel = getGeneralizedVelocity();
}

void object::PyXArticulatedSystem::setState(const Eigen::VectorXd &genco, const Eigen::VectorXd &genvel){
    setGeneralizedCoordinate(genco);
    setGeneralizedVelocity(genvel);
}

void object::PyXArticulatedSystem::setColor(Eigen::Vector4d color) {
    std::cout << "Setting color in PhysX is not supported currently." << std::endl;
    return;
}

const Eigen::Map<Eigen::Matrix<double, 3, 1>> object::PyXArticulatedSystem::getLinearMomentumInCartesianSpace() {
    double m = getTotalMass();
    PxVec3 vec = articulation_->getRootLinearVelocity();
    linearMomentum_ = {m * vec.x,
                       m * vec.y,
                       m * vec.z};
    return linearMomentum_.e();
}

/** 获取总质量 */
double object::PyXArticulatedSystem::getTotalMass() {
    int n = articulation_->getNbLinks();
    int mass = 0;
    std::vector<PxArticulationLink *> links(n, nullptr);
    articulation_->getLinks(links.data(), n, 0);
    for (int i = 0; i < n; ++i) {
        mass += links[i]->getMass();
    }
    return mass;
}

double object::PyXArticulatedSystem::getEnergy(const benchmark::Vec<3> &gravity) {
    return getKineticEnergy() + getPotentialEnergy(gravity);
}

double object::PyXArticulatedSystem::getKineticEnergy() {

    double kineticEnergy = 0;

    int n = articulation_->getNbLinks();
    std::vector<PxArticulationLink *> links(n, nullptr);
    articulation_->getLinks(links.data(), n, 0);

    for (int i = 0; i < n; i++) {
        double mass = links[i]->getMass();
        physx::PxTransform trans = links[i]->getGlobalPose();
        physx::PxVec3 vel = links[i]->getLinearVelocity();
        physx::PxVec3 inertia = links[i]->getMassSpaceInertiaTensor();
        physx::PxVec3 ang = links[i]->getAngularVelocity();
        benchmark::Mat<3, 3> I; // 创建惯性矩阵
        I.e() << inertia[0], 0, 0,
                0, inertia[1], 0,
            0, 0, inertia[2];

        double angEnergy = 0;

        benchmark::Mat<3,3> I_w; // 创建世界坐标系下的惯性矩阵
        benchmark::Mat<3,3> mat;
        benchmark::Vec<3> a = {ang.x, ang.y, ang.z};

        benchmark::quatToRotMat({trans.q.x,trans.q.y,trans.q.z,trans.q.w},mat);
        benchmark::similarityTransform(mat,I,I_w);
        benchmark::vecTransposeMatVecMul(a,I_w,angEnergy);

        double linEnergy = 0;
        benchmark::Vec<3> v = {vel.x, vel.y, vel.z};
        benchmark::vecDot(v,v,linEnergy);
        kineticEnergy += (0.5 * linEnergy * mass);

    }

    return kineticEnergy;
}

double object::PyXArticulatedSystem::getPotentialEnergy(const benchmark::Vec<3> &gravity) {

    double totalPotential = 0;
    double potential = 0;

    int n = articulation_->getNbLinks();
    std::vector<PxArticulationLink *> links(n, nullptr);
    articulation_->getLinks(links.data(), n, 0);

    for (int i = 0; i < n; i++) {
        physx::PxTransform trans = links[i]->getGlobalPose();
        benchmark::vecDot({trans.p.x, trans.p.y, trans.p.z}, gravity, potential);
        totalPotential += (-potential * links[i]->getMass());
    }

    return totalPotential;
}

void physx_sim::object::PyXArticulatedSystem::updateVisuals() {
    visObj.clear();
    visColObj.clear();
    visProps_.clear();
    visColProps_.clear();
    initVisual();
}

void physx_sim::object::PyXArticulatedSystem::initVisual() {

    physx::PxArticulationReducedCoordinate* articulation = articulation_;

    // 获取所有关节
    int n = articulation->getNbLinks();
    std::vector<physx::PxArticulationLink*> links(n, nullptr);
    articulation->getLinks(links.data(), n, 0);

    for(int i = 0; i < n; i++) {
        physx::PxArticulationLink* link = links[i];

        // 获取该关节所有形状
        physx::PxU32 nbShapes = link->getNbShapes();
        std::vector<physx::PxShape*> shapes(nbShapes);
        link->getShapes(shapes.data(), nbShapes);

        for(physx::PxU32 j = 0; j < nbShapes; j++) {
            physx::PxShape* shape = shapes[j];
            physx::PxGeometryHolder geom = shape->getGeometry();

            // 计算全局位姿
            physx::PxTransform linkPose = link->getGlobalPose();
            physx::PxTransform shapeLocalPose = shape->getLocalPose();
            physx::PxTransform globalPose = linkPose * shapeLocalPose;

            // 转换旋转矩阵
            Eigen::Quaterniond quat(
                    globalPose.q.w,
                    globalPose.q.x,
                    globalPose.q.y,
                    globalPose.q.z
            );
            benchmark::Mat<3,3> mat;
            mat.e() = quat.toRotationMatrix();

            // 转换位置
            benchmark::Vec<3> position = {
                    static_cast<double>(globalPose.p.x),
                    static_cast<double>(globalPose.p.y),
                    static_cast<double>(globalPose.p.z)
            };

            // 处理不同几何类型
            switch(geom.getType()) {
                case physx::PxGeometryType::eBOX: {
                    physx::PxBoxGeometry box = geom.box();
                    benchmark::Vec<4> boxSize = {
                            box.halfExtents.x * 2,
                            box.halfExtents.y * 2,
                            box.halfExtents.z * 2,
                            0
                    };
                    visObj.emplace_back(std::make_tuple(mat, position, j, benchmark::object::Shape::Box, color_));
                    visProps_.emplace_back(std::make_pair("", boxSize));
                    break;
                }
                case physx::PxGeometryType::eSPHERE: {
                    physx::PxSphereGeometry sphere = geom.sphere();
                    benchmark::Vec<4> sphereSize = {sphere.radius, 0, 0, 0};
                    visObj.emplace_back(std::make_tuple(mat, position, j, benchmark::object::Shape::Sphere, color_));
                    visProps_.emplace_back(std::make_pair("", sphereSize));
                    break;
                }
                case physx::PxGeometryType::eCAPSULE: {
                    physx::PxCapsuleGeometry capsule = geom.capsule();
                    benchmark::Vec<4> cylSize = {
                            capsule.radius,
                            capsule.halfHeight * 2,
                            0,
                            0
                    };
                    visObj.emplace_back(std::make_tuple(mat, position, j, benchmark::object::Shape::Cylinder, color_));
                    visProps_.emplace_back(std::make_pair("", cylSize));
                    break;
                }
                case physx::PxGeometryType::eTRIANGLEMESH:
                case physx::PxGeometryType::eCONVEXMESH:
                    // 默认不处理mesh
                    break;
                default:
                RAIFATAL("not supported shape");
            }
        }
    }
}