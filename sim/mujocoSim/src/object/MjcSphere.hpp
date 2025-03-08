/** 适用于mujoco的球形 */

#ifndef MUJOCOSIM_SPHERE_HPP
#define MUJOCOSIM_SPHERE_HPP

#include "MjcSingleBodyObject.hpp"

namespace mujoco_sim {
namespace object {


class MjcSphere: public MjcSingleBodyObject {

 public:
  MjcSphere(double radius,
            double mass,
            mjData *data,
            mjModel *model,
            int bodyId,
            int geomId);

 private:
  double radius_;
  double mass_;

};

} // object
} // mujoco_sim

#endif //MUJOCOSIM_SPHERE_HPP
