/** 用于实现physx的SingleBodyObjectInterface */

#ifndef SIMBENCHMARK_PYXSINGLEBODYOBJECT_HPP
#define SIMBENCHMARK_PYXSINGLEBODYOBJECT_HPP

#include <Eigen/Geometry>
#include <common/UserHandle.hpp>
#include <PxPhysicsAPI.h>

#include "common/math.hpp"
#include "common/interface/SingleBodyObjectInterface.hpp"

namespace physx_sim {

    namespace object {

    /** physx对单体物体接口的实现类 */
    class PyXSingleBodyObject: public benchmark::object::SingleBodyObjectInterface{

    public:
        /**
         * constructor
         * */
        PyXSingleBodyObject(physx::PxRigidDynamic* actor, physx::PxMaterial* material);

        PyXSingleBodyObject();

        /**
          * @brief 获取物体的四元数表示。
          *
          * @return 物体的四元数。
          */
        const benchmark::eQuaternion getQuaternion() override ;

        /**
        * @brief 获取物体的四元数表示，存储到给定的向量中。
        *
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
        * @brief 获取物体质量
        */
        double getMass(){
           return mass_;
        }

        /**
         * @brief 判断物体是否能移动
         */
         bool isMovable(){
             return this->isMovable_;
         }

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
         * 该函数将给定的摩擦系数应用于几何体的滑动摩擦，其他摩擦设为零。
         *
         * @param friction 要设置的摩擦系数。
         */
        void setGeomFriction(benchmark::Vec<3> friction);

        /**
           * @brief 设置摩擦系数。
           * @param friction 摩擦系数值。
           */
        void setFrictionCoefficient(double friction) override ;

        /**
       * @brief 设置摩擦系数。
       * @param friction 摩擦系数值。
       */
        void setNoSlipCoefficient(double friction);

        /**
         *  @brief 设置单体位置
         * */
        void setPosition(Eigen::Vector3d originPosition) override ;

        /**
         *  @brief 设置单体位置
         * */
        void setPosition(double x, double y, double z) override ;

        /**
         * 设置相对于世界坐标系的旋转四元数
         * @param quaternion  以 Quaterniond 格式表示的旋转四元数 (w, x, y, z)
         */
        void setOrientation(Eigen::Quaterniond quaternion) override ;

        /**
         * 设置相对于世界坐标系的旋转四元数
         * @param w  四元数的实部
         * @param x  四元数的虚部 x
         * @param y  四元数的虚部 y
         * @param z  四元数的虚部 z
         */
        void setOrientation(double w, double x, double y, double z) override ;

        /**
         * 设置相对于世界坐标系的旋转矩阵
         * @param rotationMatrix  以 Matrix3d 格式表示的 3x3 旋转矩阵
         */
        void setOrientation(Eigen::Matrix3d rotationMatrix) override ;

        /**
         * 随机设置物体的朝向。
         */
        void setOrientationRandom() override ;

        /**
         * 设置物体的体框原点和四元数相对于世界坐标系
         * @param originPosition  物体的体框原点
         * @param quaternion      物体的旋转四元数
         */
        void setPose(Eigen::Vector3d originPosition, Eigen::Quaterniond quaternion) override ;

        /**
         * 设置物体的体框原点和旋转矩阵相对于世界坐标系
         * @param originPosition  物体的体框原点
         * @param rotationMatrix  物体的旋转矩阵
         */
        void setPose(Eigen::Vector3d originPosition, Eigen::Matrix3d rotationMatrix) override ;

        /**
         * 设置物体的线性速度和角速度相对于世界坐标系
         * @param linearVelocity      质心的线性速度
         * @param angularVelocity     物体的角速度
         */
        void setVelocity(Eigen::Vector3d linearVelocity, Eigen::Vector3d angularVelocity) override ;

        /**
         * 设置物体的线性速度和角速度相对于世界坐标系
         * @param dx  线性速度 x 轴分量
         * @param dy  线性速度 y 轴分量
         * @param dz  线性速度 z 轴分量
         * @param wx  角速度 x 轴分量
         * @param wy  角速度 y 轴分量
         * @param wz  角速度 z 轴分量
         */
        void setVelocity(double dx, double dy, double dz, double wx, double wy, double wz) override;

        /**
           * 设置恢复系数
           * @param restitution  恢复系数，表示碰撞后的弹性程度
        */
        void setRestitutionCoefficient(double restitution) override;


        bool isVisualizeFramesAndCom() const override {};


    protected:

        /** body id in the world */
        int bodyID_ = 0;

        /** geometry id in body */
        int geomID_ = 0;

        /** PhysX 引擎相关：一个刚体对象 */
        physx::PxRigidDynamic* actor_;

        /** 静态网格体，用于保存静态刚体，如地面 */
        physx::PxRigidStatic* staticActor_;

        /** 用于保存材质 */
        physx::PxMaterial* material_;

        /** pose and velocity */
        benchmark::Vec<4> quatTemp_ = {1.0, 0.0, 0.0, 0.0};
        benchmark::Mat<3, 3> rotMatTemp_;
        benchmark::Vec<3> posTemp_ = {0.0, 0.0, 0.0};
        benchmark::Vec<3> linVelTemp_ = {0.0, 0.0, 0.0};
        benchmark::Vec<3> angVelTemp_ = {0.0, 0.0, 0.0};

        /** momentum */
        benchmark::Vec<3> linearMomentum_ = {0, 0, 0};

        /** inertia */
        benchmark::Mat<3,3> localInertia;

        /** movable */
        bool isMovable_ = true;

        /** mass */
        double mass_ = 0;

    };

}   // object
}   // physx_sim


#endif //SIMBENCHMARK_PYXSINGLEBODYOBJECT_HPP
