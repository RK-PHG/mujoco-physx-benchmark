/** 此文件用于属性配置 */

#ifndef BENCHMARK_CONFIGURE_HPP
#define BENCHMARK_CONFIGURE_HPP

namespace benchmark {
/** 碰撞组类型 */
typedef int CollisionGroupType;

/** 物体类型 */
enum ObjectType { SPHERE, BOX, CYLINDER, CONE, CAPSULE, CONVEXMESH, HALFSPACE, COMPOUND, HEIGHTMAP, ARTICULATED_SYSTEM };

}

#endif //BENCHMARK_CONFIGURE_HPP
