/** 实现Physx的世界接口 */

#include "PyXWorld.hpp"
#include <queue>
#include <unordered_map>
#include <unordered_set>

using namespace physx_sim;
using namespace std;

// 辅助函数，欧拉角转四元数
PxQuat eulerToQuaternion(float roll, float pitch, float yaw) {

    // 将欧拉角转换为四元数
    float cy = cos(yaw * 0.5f);
    float sy = sin(yaw * 0.5f);
    float cp = cos(pitch * 0.5f);
    float sp = sin(pitch * 0.5f);
    float cr = cos(roll * 0.5f);
    float sr = sin(roll * 0.5f);

    // 四元数的计算公式
    return PxQuat(
            cy * cp * cr + sy * sp * sr,  // w
            cy * cp * sr - sy * sp * cr,  // x
            sy * cp * cr + cy * sp * sr,  // y
            sy * cp * sr - cy * sp * cr   // z
    );

}

// 辅助函数，从rootElement中获取某个name的link节点
tinyxml2::XMLElement *getLinkByName(tinyxml2::XMLElement *root, const char *name) {
    // 遍历所有link元素
    for (tinyxml2::XMLElement *linkElement = root->FirstChildElement(
            "link"); linkElement; linkElement = linkElement->NextSiblingElement("link")) {
        // 获取link元素的name属性
        const char *linkName = linkElement->Attribute("name");
        if (linkName && strcmp(linkName, name) == 0) {
            // 如果name属性匹配，返回该link元素
            return linkElement;
        }
    }

    // 如果没有找到匹配的link元素，返回nullptr
    return nullptr;
}

// 辅助函数，从root中找到所有以name为父节点的关节
vector<tinyxml2::XMLElement *> getRelatedJoints(tinyxml2::XMLElement *root, const char *parentName) {
    vector<tinyxml2::XMLElement *> relatedJoints;

    // 遍历所有的 <joint> 元素
    for (tinyxml2::XMLElement *jointElement = root->FirstChildElement(
            "joint"); jointElement; jointElement = jointElement->NextSiblingElement("joint")) {
        // 获取 <parent> 子元素
        tinyxml2::XMLElement *parentElement = jointElement->FirstChildElement("parent");
        if (parentElement) {
            // 获取 parent 元素中的 link 属性
            const char *parentLink = parentElement->Attribute("link");
            if (parentLink && strcmp(parentLink, parentName) == 0) {
                // 如果 parent 的 link 属性值匹配给定的 parentName，则将该 joint 元素加入到结果中
                relatedJoints.push_back(jointElement);
            }
        }
    }

    return relatedJoints;
}

// 辅助函数，解析link节点，提供父节点，创建一个新的link
// 参数：关节坐标接口、父节点指针、link对应的XML Element，用于检索材料的materials
PxArticulationLink *PyXWorld::addLink(PxArticulationReducedCoordinate *articulation,
                                      PxArticulationLink *parent,
                                      tinyxml2::XMLElement *linkElement,
                                      unordered_map<string, vector<float>> &materials) {

    PxArticulationLink *newLink = nullptr;

    // 获取节点名称
    const char *linkName = linkElement->Attribute("name");
    if (!linkName) {
        return nullptr;
    }

    // 获取节点初始姿态和材料信息
    // 解析并设置视觉属性
    tinyxml2::XMLElement *visualElement = linkElement->FirstChildElement("visual");
    if (!visualElement)
        return nullptr;

    // 获取初始姿态信息
    tinyxml2::XMLElement *originElement = visualElement->FirstChildElement("origin");
    if (!originElement) {
        return nullptr;
    }
    float rpy[3], xyz[3];
    originElement->QueryFloatAttribute("rpy", &rpy[0]);
    originElement->QueryFloatAttribute("xyz", &xyz[0]);

    // 设置初始旋转与位置
    PxQuat rotation = eulerToQuaternion(rpy[0], rpy[1], rpy[2]);
    PxVec3 position = PxVec3(xyz[0], xyz[1], xyz[2]);

    // 创建链接
    newLink = articulation->createLink(parent, PxTransform(position, rotation));

    // 创建材料
    PxMaterial *material = physics_->createMaterial(0.5, 0.5, 0.5);

    // 获取几何体信息
    tinyxml2::XMLElement *geometryElement = visualElement->FirstChildElement("geometry");
    if (!geometryElement)
        return nullptr;

    // 获取几何体类型
    const char *type = geometryElement->FirstChildElement()->Name();  // 获取几何体类型

    // 盒体
    if (strcmp(type, "box") == 0) {
        float size[3];
        // 获取 'size' 属性的字符串值
        const char* sizeStr = geometryElement->FirstChildElement("box")->Attribute("size");

        // 直接解析 'size' 属性值为三个 float 值
        sscanf(sizeStr, "%f %f %f", &size[0], &size[1], &size[2]);

        // 创建 PxBoxGeometry 并设置形状
        PxBoxGeometry boxGeometry(PxVec3(size[0], size[1], size[2]));
        PxRigidActorExt::createExclusiveShape(*newLink, boxGeometry, *material);
    }
    // 球体
    else if (strcmp(type, "sphere") == 0) {
        float radius;
        geometryElement->FirstChildElement("sphere")->QueryFloatAttribute("radius", &radius);
        PxSphereGeometry sphereGeometry(radius);
        PxRigidActorExt::createExclusiveShape(*newLink, sphereGeometry, *material);
    }
    // 圆柱体（使用胶囊体近似）
    else if (strcmp(type, "cylinder") == 0) {
        float length, radius;
        geometryElement->FirstChildElement("cylinder")->QueryFloatAttribute("length", &length);
        geometryElement->FirstChildElement("cylinder")->QueryFloatAttribute("radius", &radius);

        // 使用胶囊体近似圆柱体
        PxCapsuleGeometry capsuleGeometry(radius, length); // 半长为 length / 2
        PxRigidActorExt::createExclusiveShape(*newLink, capsuleGeometry, *material);
    }
    // 胶囊体
    else if (strcmp(type, "capsule") == 0) {
        float radius, halfHeight;
        geometryElement->FirstChildElement("capsule")->QueryFloatAttribute("radius", &radius);
        geometryElement->FirstChildElement("capsule")->QueryFloatAttribute("length", &halfHeight);

        // 将长度除以2，得到半高
        PxCapsuleGeometry capsuleGeometry(radius, halfHeight);
        PxRigidActorExt::createExclusiveShape(*newLink, capsuleGeometry, *material);
    }

    // 解析并设置惯性属性
    tinyxml2::XMLElement *inertialElement = linkElement->FirstChildElement("inertial");
    if (inertialElement) {
        // 获取质量
        float mass = 0.0f;
        inertialElement->FirstChildElement("mass")->QueryFloatAttribute("value", &mass);
        PxRigidBodyExt::updateMassAndInertia(*newLink, 1.0f);
    }

    return newLink;
}

// 加载模型
// 参数：模型文件路径
object::PyXArticulatedSystem* PyXWorld::loadModel(std::string modelPath) {

    // 初始化URDF解析器
    tinyxml2::XMLDocument urdfDoc;
    if (urdfDoc.LoadFile(modelPath.c_str()) != tinyxml2::XML_SUCCESS) {
        throw std::runtime_error("Failed to load URDF file: " + modelPath);
    }

    // 获取URDF根元素
    tinyxml2::XMLElement *robotElement = urdfDoc.FirstChildElement("robot");
    if (!robotElement) {
        throw std::runtime_error("Invalid URDF file: missing <robot> tag.");
    }

    // 创建关节系统
    physx::PxArticulationReducedCoordinate *articulation = physics_->createArticulationReducedCoordinate();
    articulation->setArticulationFlag(PxArticulationFlag::eFIX_BASE, true);
    articulation->setSolverIterationCounts(1000, 50);
    articulation->setMaxCOMLinearVelocity(5.0f);  // 设置最大质心速度为 5 m/s

    // 用于存储Links
    std::queue<pair<string, PxArticulationLink *>> links;
    unordered_map<string, vector<float>> materials;
    unordered_set<string> visited;

    // 开始进行URDF文件解析
    // 思路：首先获取“base”，然后按照广度优先进行添加，即添加所有以“base”为父节点的关节、同时
    // 添加子节点，按照层序进行添加，比较贴合physx的添加逻辑，默认关节系统为树形结构，如果存在环，则忽略产生环的关节，不进行处理
    // 首先找到base，为其创建一个初始链接
    tinyxml2::XMLElement *base = robotElement->FirstChildElement("link");
    if (!base)
        return nullptr;


    PxArticulationLink *link = addLink(articulation, nullptr, base, materials);
    links.push(pair<string, PxArticulationLink *>("base", link));
    unordered_map<PxArticulationLink* ,string> m;
    m.insert(pair<PxArticulationLink *,string>(link, "base"));

    visited.insert(string("base"));


    int joint_num = 0;

    // 层序遍历，逐一添加关节
    while (!links.empty()) {
        int n = links.size();
        for (int i = 0; i < n; i++) {
            pair<string, PxArticulationLink *> &l = links.front();
            links.pop();
            // 查找所有以该节点为parent的关节，添加对应的子节点
            vector<tinyxml2::XMLElement *> jointElements = getRelatedJoints(robotElement, l.first.c_str());
            // 遍历以当前节点为父节点的所有关节
            for (auto &j: jointElements) {
                tinyxml2::XMLElement *childElement = j->FirstChildElement("child");
                const char *childName = childElement->Attribute("link");
                //这里有环的情况直接跳过
                if (visited.count(string(childName))) {
                    continue;
                }
                tinyxml2::XMLElement *childLinkElement = getLinkByName(robotElement, childName);
                PxArticulationLink *child = addLink(articulation, l.second, childLinkElement, materials);
                PxArticulationJointReducedCoordinate* joint = child->getInboundJoint();

                // 设置关节类型，从节点属性中获取
                const char* typeValue = j->Attribute("type");
                // 设置对应的关节类型
                if (strcmp(typeValue, "revolute") == 0) {
                    joint->setJointType(PxArticulationJointType::eREVOLUTE);
                } else if (strcmp(typeValue, "prismatic") == 0) {
                    joint->setJointType(PxArticulationJointType::ePRISMATIC);
                } else if (strcmp(typeValue, "continuous") == 0) {
                    joint->setJointType(PxArticulationJointType::eREVOLUTE);  // PhysX treats continuous like revolute
                    joint->setMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eLIMITED);
                    PxArticulationLimit limits;
                    limits.low = -PxPiDivFour;  // in rad for a rotational motion
                    limits.high = PxPiDivFour;
                    joint->setLimitParams(PxArticulationAxis::eSWING2, limits);
                } else if (strcmp(typeValue, "fixed") == 0) {
                    joint->setJointType(PxArticulationJointType::eFIX);
                } else if (strcmp(typeValue, "floating") == 0) {
                    joint->setJointType(PxArticulationJointType::eSPHERICAL);
                }

                visited.insert(string(childName));
                links.push(pair<string, PxArticulationLink *>(string(childName), child));
                m.insert(pair<PxArticulationLink *,string>(child,string(childName)));
                joint_num ++;
            }
        }
    }

    // 向场景中添加关节系统
    scene_->addArticulation(*articulation);
    object::PyXArticulatedSystem* articulatedSystem = new object::PyXArticulatedSystem(articulation);
    articulatedSystemList_.push_back(articulatedSystem);

    articulatedSystem->initVisual();

    return articulatedSystem;

}

/**
 *  @brief 构建世界场景
 * */
PyXWorld::PyXWorld() {

    // foundation 对象初始化
    foundation_ = PxCreateFoundation(PX_PHYSICS_VERSION, allocator_, errorCallback_);

    // pvd
    physx::PxPvdTransport *mTransport = physx::PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10);

    // mPvd Flags
    physx::PxPvdInstrumentationFlags mPvdFlags = physx::PxPvdInstrumentationFlag::eALL;

    pvd_ = physx::PxCreatePvd(*foundation_);

    bool success = pvd_->connect(*mTransport, mPvdFlags);

    // physics 对象初始化
    physics_ = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation_, PxTolerancesScale(), true, pvd_);

    // 场景属性
    PxSceneDesc sceneDesc(physics_->getTolerancesScale());

    // 初始化重力加速度
    sceneDesc.gravity = PxVec3(0.0f, -9.80f, 0.0f);

    // dispatcher
    dispatcher_ = PxDefaultCpuDispatcherCreate(2);
    sceneDesc.cpuDispatcher = dispatcher_;
    sceneDesc.filterShader = PxDefaultSimulationFilterShader;

    // 创建场景
    scene_ = physics_->createScene(sceneDesc);

    PxPvdSceneClient *pvdClient = scene_->getScenePvdClient();

    if (!scene_) {
        std::cout << "Scene creation failed." << std::endl;
    }
    if (!pvd_) {
        std::cout << "PVD creation failed." << std::endl;
    }

    if (pvdClient) {
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
    }

    CollisionCallback *callback = new CollisionCallback(this);

    scene_->setSimulationEventCallback(callback);
}

/**
 * @brief 析构函数
 * */
PyXWorld::~PyXWorld() {

    PX_RELEASE(scene_);
    PX_RELEASE(dispatcher_);
    PX_RELEASE(physics_);
    PX_RELEASE(foundation_);
    if (pvd_) {
        PX_RELEASE(pvd_)
    }
}

object::PyXSphere *PyXWorld::addSphere(double radius, double mass, benchmark::Vec<3> pos) {

    PxRigidDynamic *rigidDynamic = physics_->createRigidDynamic(PxTransform(PxVec3(pos[0], pos[1], pos[2])));
    PxMaterial *material = physics_->createMaterial(0.0f, 0.0f, 0.0f);;
    PxShape *shape = PxRigidActorExt::createExclusiveShape(*rigidDynamic, PxSphereGeometry(radius), *material);

    rigidDynamic->attachShape(*shape);
    PxRigidBodyExt::updateMassAndInertia(*rigidDynamic, mass);
    object::PyXSphere *sphere = new object::PyXSphere(radius, mass, rigidDynamic, material);

    scene_->addActor(*rigidDynamic);
    objectList_.push_back(sphere);

    return sphere;
}

object::PyXBox *PyXWorld::addBox(double xLength,
                                 double yLength,
                                 double zLength,
                                 double mass,
                                 benchmark::Vec<3> pos) {

    PxRigidDynamic *rigidDynamic = physics_->createRigidDynamic(PxTransform(PxVec3(pos[0], pos[1], pos[2])));
    PxMaterial *material = physics_->createMaterial(0.0f, 0.0f, 0.0f);;
    auto shape = PxRigidActorExt::createExclusiveShape(*rigidDynamic, PxBoxGeometry(xLength, yLength, zLength),
                                                       *material);

    rigidDynamic->attachShape(*shape);
    PxRigidBodyExt::updateMassAndInertia(*rigidDynamic, mass);
    object::PyXBox *pyXBox = new object::PyXBox(xLength, yLength, zLength, mass, rigidDynamic, material);

    scene_->addActor(*rigidDynamic);
    objectList_.push_back(pyXBox);

    return pyXBox;

}

/**
 * @brief 参考地面的放置
 * */
object::PyXCheckerBoard *PyXWorld::addCheckerboard(benchmark::Vec<3> pos) {

    // 材质
    PxMaterial *material = physics_->createMaterial(0.0f, 0.0f, 0.0f);

    // 创建地面
    PxRigidStatic *groundPlane = PxCreatePlane(*physics_, PxPlane(pos[0], pos[1], pos[2], 0), *material);

    object::PyXCheckerBoard *pyXCheckerBoard = new object::PyXCheckerBoard(groundPlane, material);

    scene_->addActor(*groundPlane);

    objectList_.push_back(pyXCheckerBoard);

    return pyXCheckerBoard;

}

/** 胶囊体的放置 */
object::PyXCapsule *PyXWorld::addCapsule(double radius,
                                         double height,
                                         double mass,
                                         benchmark::Vec<3> pos) {

    // 几何体
    PxCapsuleGeometry capsuleGeometry(radius, height);

    // 创建刚体
    PxRigidDynamic *rigidDynamic = physics_->createRigidDynamic(PxTransform(PxVec3(pos[0], pos[1], pos[2])));

    // 材料
    PxMaterial *material = physics_->createMaterial(0.0f, 0.0f, 0.0f);

    // 形状
    auto shape = PxRigidActorExt::createExclusiveShape(*rigidDynamic, capsuleGeometry, *material);

    // 质量
    PxRigidBodyExt::updateMassAndInertia(*rigidDynamic, mass);

    // attach shape
    rigidDynamic->attachShape(*shape);

    // 初始化形状
    object::PyXCapsule *capsule = new object::PyXCapsule(radius, height, mass, rigidDynamic, material);

    // 添加球形到场景中
    scene_->addActor(*rigidDynamic);

    // 添加刚体到物体列表中
    objectList_.push_back(capsule);

    return nullptr;

}

/** -- 世界通用属性的设置 -- */

/** 设置重力 */
void PyXWorld::setGravity(const benchmark::Vec<3> &gravity) {

    if (scene_ != nullptr) {
        physx::PxVec3 pxGravity(gravity[0], gravity[1], gravity[2]);
        scene_->setGravity(pxGravity);
    } else {
        std::cerr << "Scene is not initialized. " << std::endl;
    }

}

/** 设置无滑动参数 */
void PyXWorld::setNoSlipParameter(double friction) {

    for (auto &obj: objectList_) {
        obj->setNoSlipCoefficient(friction);
    }

}

/** 设置步长 */
void PyXWorld::setTimeStep(double timeStep) {
    this->timeStep_ = timeStep;
}

/** -- 获取世界属性 -- */

/** 获取物体数量 */
int PyXWorld::getNumObject() {
    return objectList_.size();
}

/** 获取线性动量 */
const Eigen::Map<Eigen::Matrix<double, 3, 1>> PyXWorld::getLinearMomentumInCartesianSpace() {

    Eigen::Vector3d linearMomentum;
    linearMomentum.setZero();

    for (auto &obj: objectList_) {
        linearMomentum += obj->getMass() * obj->getLinearVelocity();
    }

    linearMomentum_ = {linearMomentum.x(), linearMomentum.y(), linearMomentum.z()};
    return linearMomentum_.e();

}

/** 获取总质量 */
double PyXWorld::getTotalMass() {

    double mass = 0;

    for(auto a: articulatedSystemList_){
        mass += a->getTotalMass();
    }

    for (auto &obj: objectList_) {
        if (obj->isMovable()) {
            mass += obj->getMass();
        }
    }

    return mass;
}

/** -- 仿真步骤 -- */

/** 执行一步 */
void PyXWorld::integrate() {

    // 模拟一步
    scene_->simulate(this->timeStep_);

    // 获取结果并更新场景状态
    scene_->fetchResults(true);

}

/** 获取动能 */
double PyXWorld::getKineticEnergy() {

    double kEnergy = 0;

    for (auto &obj: objectList_) {
        if (obj->isMovable())
            kEnergy += obj->getKineticEnergy();
    }

    for (auto a: articulatedSystemList_){
        kEnergy += a->getKineticEnergy();
    }

    return kEnergy;

}

/** 获取重力势能 */
double PyXWorld::getPotentialEnergy(const benchmark::Vec<3> &gravity) {

    double pEnergy = 0;

    for (auto &obj: objectList_) {
        if (obj->isMovable()) {
            pEnergy += obj->getPotentialEnergy(gravity);
        }
    }

    for (auto a: articulatedSystemList_){
        pEnergy += a->getPotentialEnergy(gravity);
    }

    return pEnergy;

}

/** 获取总能量 */
double PyXWorld::getEnergy(const benchmark::Vec<3> &gravity) {

    double energy = 0;

    for (auto &obj: objectList_) {
        if (obj->isMovable()) {
            energy += obj->getEnergy(gravity);
        }
    }

    for (auto a: articulatedSystemList_){
        energy += a->getEnergy(gravity);
    }

    return energy;

}

int PyXWorld::getWorldNumContacts() {
    return contactProblemList_.size();
}