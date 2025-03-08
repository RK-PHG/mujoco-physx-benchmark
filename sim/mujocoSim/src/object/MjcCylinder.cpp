#include "MjcCylinder.hpp"

mujoco_sim::object::MjcCylinder::MjcCylinder(double radius,
                                       double height,
                                       mjData *data,
                                       mjModel *model,
                                       int bodyId,
                                       int geomId)
    : MjcSingleBodyObject(data, model, bodyId, geomId), radius_(radius), height_(height) {}
