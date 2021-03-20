#ifndef POSE_SENSOR_H
#define POSE_SENSOR_H

#include "sensors/internalSensor.h"
#include <geometry_msgs/msg/pose_stamped.hpp>

using geometry_msgs::msg::PoseStamped;

class PoseSensor : public InternalSensor<PoseStamped>{
protected:
    virtual PoseStamped getMessage() override;
public:
    PoseSensor(SimObjectConfiguration conf);
};

#endif