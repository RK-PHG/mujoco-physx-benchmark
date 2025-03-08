#include "MjcSphere.hpp"

mujoco_sim::object::MjcSphere::MjcSphere(double radius,
                                   double mass,
                                   mjData *data,
                                   mjModel *model,
                                   int bodyId,
                                   int geomId)
    : MjcSingleBodyObject(data, model, bodyId, geomId), radius_(radius), mass_(mass) {}
