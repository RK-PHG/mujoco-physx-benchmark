#include "PyXSingleBodyObject.hpp"

namespace physx_sim{
    namespace object{

        /** 无参构造函数 */
        PyXSingleBodyObject::PyXSingleBodyObject() {}

        /** *
         * @brief 构造函数
         *
         * @param actor 提供刚体对象指针
         * @param bodyId 提供物体编号
         * @param geomId 提供几何体编号
         */
        PyXSingleBodyObject::PyXSingleBodyObject(physx::PxRigidDynamic *actor, physx::PxMaterial* material)
                     :actor_(actor),material_(material){
        }

        /**
         * @brief 获得旋转四元数
         * */
        const benchmark::eQuaternion PyXSingleBodyObject::getQuaternion() {
            physx::PxTransform transform = actor_->getGlobalPose();
            physx::PxQuat p = transform.q;
            quatTemp_ = {p.x,p.y,p.z,p.w};
            return quatTemp_.e();
        }

        /**
         * @brief 获得旋转四元数，并到给定向量中
         * */
        void PyXSingleBodyObject::getQuaternion(benchmark::Vec<4> &quat) {
            if(actor_) {
                physx::PxTransform transform = actor_->getGlobalPose();
                physx::PxQuat p = transform.q;
                quat = {p.x, p.y, p.z, p.w};
            }
        }

        /**
         * @brief 获得旋转矩阵
         * */
        const benchmark::eRotationMat PyXSingleBodyObject::getRotationMatrix() {
            physx::PxTransform transform = actor_->getGlobalPose();
            physx::PxQuat quat = transform.q;
            benchmark::Vec<4> q = {quat.x,quat.y,quat.z,quat.w};
            benchmark::quatToRotMat(q,rotMatTemp_);
            return rotMatTemp_.e();
        }

        /**
         * @brief 获取旋转矩阵
         * @param rotation
         */
        void PyXSingleBodyObject::getRotationMatrix(benchmark::Mat<3,3> &rotation) {
            physx::PxTransform transform = actor_->getGlobalPose();
            physx::PxQuat quat = transform.q;
            benchmark::Vec<4> q = {quat.x,quat.y,quat.z,quat.w};
            benchmark::quatToRotMat(q,rotation);
        }

        /**
         * @brief 获取物体位置
         * @return 物体的位置，表示为三维向量
         */
        const benchmark::eVector3  PyXSingleBodyObject::getPosition() {
            physx::PxTransform t = actor_->getGlobalPose();
            posTemp_ = {t.p.x, t.p.y, t.p.z}; // 存储位置到临时变量
            return posTemp_.e();
        }

        /**
         * @brief 获取物体的线速度。
         *
         * 该函数从物体获取当前的线速度并返回一个三维向量表示物体的线速度。
         *
         * @return 物体的线速度，表示为三维向量。
         */
        const   benchmark::eVector3 PyXSingleBodyObject::getLinearVelocity() {
            physx::PxVec3 lv = actor_->getLinearVelocity();
            linVelTemp_ = {lv.x,lv.y,lv.z};
            return linVelTemp_.e();
        }

        /**
         * @brief 获取物体的角速度。
         *
         * 该函数从物体获取当前的角速度并返回一个三维向量表示物体的角速度。
         *
         * @return 物体的角速度，表示为三维向量。
         */
        const benchmark::eVector3 PyXSingleBodyObject::getAngularVelocity() {
            physx::PxVec3 la = actor_->getAngularVelocity();
            angVelTemp_ = {la.x,la.y,la.z};
            return angVelTemp_.e();
        }

        /**
         * @brief 获取物体在世界坐标系中的位置。
         *
         * 该函数将物体的当前位置填充到给定的三维向量中。
         *
         * @param pos_w 输出参数，表示物体在世界坐标系中的位置。
         */
        void PyXSingleBodyObject::getPosition_W(benchmark::Vec<3> &pos_w) {
            if(actor_) {
                physx::PxTransform t = actor_->getGlobalPose();
                pos_w = {t.p.x, t.p.y, t.p.z};
            }
        }

        /**
         * @brief 设置物体的外力。
         *
         * 该函数将外力应用于物体。
         *
         * @param force 要施加的外力，表示为三维向量。
         */
        void PyXSingleBodyObject::setExternalForce(Eigen::Vector3d force) {
            physx::PxRigidDynamic* dynamicActor = static_cast<physx::PxRigidDynamic*>(actor_);
            physx::PxVec3 f(force[0],force[1],force[2]);
            dynamicActor->addForce(f,physx::PxForceMode::eFORCE);
        }

        /**
         * @brief 设置物体的外转矩。
         *
         * 该函数将外转矩应用于物体。
         *
         * @param torque 要施加的外转矩，表示为三维向量。
         */
         void PyXSingleBodyObject::setExternalTorque(Eigen::Vector3d torque) {

            // 确保 actor_ 是 PxRigidDynamic 类型
            physx::PxRigidDynamic* dynamicActor = static_cast<physx::PxRigidDynamic*>(actor_);

            // 将 Eigen 向量转换为 PxVec3
            physx::PxVec3 pxTorque(torque.x(), torque.y(), torque.z());

            // 设置外转矩
            dynamicActor->addTorque(pxTorque, physx::PxForceMode::eFORCE); // 或使用其他转矩模式

         }

        /**
         * @brief 设置几何体的摩擦系数。
         *
         * 该函数根据给定的摩擦值设置几何体的摩擦系数，支持滑动、旋转和滚动摩擦。
         *
         * @param friction 摩擦系数，表示为一个包含滑动、旋转和滚动摩擦的三维向量。
         */
         void PyXSingleBodyObject::setGeomFriction(benchmark::Vec<3> friction) {
            setFrictionCoefficient(friction[0]);
            setNoSlipCoefficient(friction[2]);
         }

        /**
        * @brief 设置摩擦系数。
        * @param friction 摩擦系数值。
        */
        void PyXSingleBodyObject::setNoSlipCoefficient(double friction){
            material_->setStaticFriction(friction);
        }

        /**
        * @brief 设置摩擦系数。
        *
        * 该函数将给定的摩擦系数应用于几何体的滑动摩擦，其他摩擦设为零。
        *
        * @param friction 要设置的摩擦系数。
        */
         void PyXSingleBodyObject::setFrictionCoefficient(double friction) {
             material_->setStaticFriction(friction);
         }

        /**
         * @brief 获取物体的动能。
         *
         * 该函数根据物体的质量和速度计算动能。
         *
         * @return 物体的动能。
         */
         double PyXSingleBodyObject::getKineticEnergy() {

//            std::cout << "here1" << std::endl;

            double mass = actor_->getMass();
            physx::PxVec3 inertia = actor_->getMassSpaceInertiaTensor();

            getLinearVelocity();
            getAngularVelocity();

            benchmark::Mat<3,3> I; // 创建惯性矩阵
            I.e() << inertia[0], 0, 0,
                    0, inertia[1], 0,
                    0, 0, inertia[2];

            /** 计算角动能 */
            double angEnergy = 0;
            benchmark::Mat<3,3> I_w; // 创建世界坐标系下的惯性矩阵
            getRotationMatrix();
            benchmark::similarityTransform(rotMatTemp_,I,I_w);
            benchmark::vecTransposeMatVecMul(angVelTemp_,I_w,angEnergy);

            double linEnergy = 0;
            benchmark::vecDot(linVelTemp_,linVelTemp_,linEnergy);

//            std::cout << "here1 " << angEnergy << " " << linEnergy << " " << mass  << " " << std::endl;

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
         double PyXSingleBodyObject::getPotentialEnergy(const benchmark::Vec<3> &gravity) {



             double potential = 0;
             double mass = actor_->getMass();

             getPosition();
             benchmark::vecDot(posTemp_,gravity,potential);

//            std::cout << "here2 " << potential << " " << mass  << " " << std::endl;

             return -potential * mass;

         }

        /**
        * @brief 获取物体的总能量。
        *
        * 该函数返回物体的动能和势能之和。
        *
        * @param gravity 表示重力的三维向量。
        * @return 物体的总能量。
        */
         double PyXSingleBodyObject::getEnergy(const benchmark::Vec<3> &gravity) {
             return getKineticEnergy() + getPotentialEnergy(gravity);
//             std::cout << std::endl;
         }

         /**
          * @brief 物体的线动量向量
          * @return
          */
        const Eigen::Map<Eigen::Matrix<double, 3, 1>> PyXSingleBodyObject::getLinearMomentum() {
             getLinearVelocity();
             double mass = actor_->getMass();
             benchmark::vecScalarMul(mass, linVelTemp_, linearMomentum_);
             return linearMomentum_.e();
         }

        /**
         * @brief 获取物体的质心位置。
         *
         * 该函数从物体的质心位置获取并返回一个三维向量表示物体的质心位置。
         *
         * @return 物体的质心位置，表示为三维向量。
         */
        const benchmark::eVector3 PyXSingleBodyObject::getComPosition(){
            physx::PxTransform f = actor_->getGlobalPose();
            posTemp_ = {f.p.x, f.p.y, f.p.z}; // 存储位置到临时变量
            return posTemp_.e(); // 返回质心位置向量
        }


        /** 以下成员函数暂时不实现 */

        /**
       *  @brief 设置单体位置
       * */
        void  PyXSingleBodyObject::setPosition(Eigen::Vector3d originPosition){
            if(actor_){
                actor_->setGlobalPose(physx::PxTransform (originPosition[0],originPosition[1],originPosition[2]));
            }
        }

        /**
         *  @brief 设置单体位置
         * */
        void  PyXSingleBodyObject::setPosition(double x, double y, double z){
            if(actor_){
                actor_->setGlobalPose(physx::PxTransform (x,y,z));
            }
        }

        /**
         * 设置相对于世界坐标系的旋转四元数
         * @param quaternion  以 Quaterniond 格式表示的旋转四元数 (w, x, y, z)
         */
        void  PyXSingleBodyObject::setOrientation(Eigen::Quaterniond quaternion){
            physx::PxQuat quat(quaternion.x(),quaternion.y(),quaternion.z(),quaternion.w());
            if(actor_){
                actor_->setGlobalPose(physx::PxTransform(actor_->getGlobalPose().p,quat));
            }
        }

        /**
         * 设置相对于世界坐标系的旋转四元数
         * @param w  四元数的实部
         * @param x  四元数的虚部 x
         * @param y  四元数的虚部 y
         * @param z  四元数的虚部 z
         */
        void  PyXSingleBodyObject::setOrientation(double w, double x, double y, double z){
            physx::PxQuat quat(x,y,z,w);
            if(actor_){
                actor_->setGlobalPose(physx::PxTransform(actor_->getGlobalPose().p,quat));
            }
        }

        /**
         * 设置相对于世界坐标系的旋转矩阵
         * @param rotationMatrix  以 Matrix3d 格式表示的 3x3 旋转矩阵
         */
        void  PyXSingleBodyObject::setOrientation(Eigen::Matrix3d rotationMatrix){
            std::cerr << "设置旋转矩阵暂不实现" << std::endl;
        }

        /**
         * 随机设置物体的朝向。
         */
        void  PyXSingleBodyObject::setOrientationRandom(){
            std::cerr << "随机设置物体朝向暂不实现" << std::endl;
        }

        /**
         * 设置物体的体框原点和四元数相对于世界坐标系
         * @param originPosition  物体的体框原点
         * @param quaternion      物体的旋转四元数
         */
        void  PyXSingleBodyObject::setPose(Eigen::Vector3d originPosition, Eigen::Quaterniond quaternion){
            physx::PxQuat quat(quaternion.x(),quaternion.y(),quaternion.z(),quaternion.w());
            if(actor_){
                actor_->setGlobalPose(physx::PxTransform(physx::PxVec3(originPosition.x(),originPosition.y(),originPosition.z()),quat));
            }
        }

        /**
         * 设置物体的体框原点和旋转矩阵相对于世界坐标系
         * @param originPosition  物体的体框原点
         * @param rotationMatrix  物体的旋转矩阵
         */
        void  PyXSingleBodyObject::setPose(Eigen::Vector3d originPosition, Eigen::Matrix3d rotationMatrix){
            std::cerr << "设置位置和旋转矩阵暂不实现" << std::endl;
        }

        /**
         * 设置物体的线性速度和角速度相对于世界坐标系
         * @param linearVelocity      质心的线性速度
         * @param angularVelocity     物体的角速度
         */
        void  PyXSingleBodyObject::setVelocity(Eigen::Vector3d linearVelocity, Eigen::Vector3d angularVelocity){
            if(actor_){
                actor_->setLinearVelocity(physx::PxVec3(linearVelocity.x(),linearVelocity.y(),linearVelocity.z()));
                actor_->setAngularVelocity(physx::PxVec3(angularVelocity.x(),angularVelocity.y(),angularVelocity.z()));
            }
        }

        /**
         * 设置物体的线性速度和角速度相对于世界坐标系
         * @param dx  线性速度 x 轴分量
         * @param dy  线性速度 y 轴分量
         * @param dz  线性速度 z 轴分量
         * @param wx  角速度 x 轴分量
         * @param wy  角速度 y 轴分量
         * @param wz  角速度 z 轴分量
         */
        void  PyXSingleBodyObject::setVelocity(double dx, double dy, double dz, double wx, double wy, double wz) {
            if(actor_){
                actor_->setLinearVelocity(physx::PxVec3(dx,dy,dz));
                actor_->setAngularVelocity(physx::PxVec3(wx,wy,wz));
            }
        }

        /** 设置弹性碰撞的恢复系数 */
        void PyXSingleBodyObject::setRestitutionCoefficient(double restitution){
            if(actor_&&material_){
                material_->setRestitution(restitution);
            }
        };

    }
}
