#ifndef SENSOR_IMU_THIS_IS_A_TEST_H
#define SENSOR_IMU_THIS_IS_A_TEST_H

#include "sensors/internalSensor.h"
#include <sensor_msgs/msg/imu.hpp>

using sensor_msgs::msg::Imu;

class ImuSensor : public InternalSensor<Imu> {
protected:
    virtual Imu getMessage() override;
public:
    ImuSensor(SimObjectConfiguration conf);
};

#endif