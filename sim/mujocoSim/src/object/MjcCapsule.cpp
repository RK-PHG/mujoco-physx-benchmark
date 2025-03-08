#include "MjcCapsule.hpp"

mujoco_sim::object::MjcCapsule::MjcCapsule(double radius, double height, mjData *data, mjModel *model, int bodyId, int geomId)
    : MjcSingleBodyObject(data, model, bodyId, geomId), radius_(radius), height_(height) {}
