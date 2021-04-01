#ifndef SENSOR_IMU_THIS_IS_A_TEST_H
#define SENSOR_IMU_THIS_IS_A_TEST_H

#include "sensors/internalSensor.h"
#include <sensor_msgs/msg/imu.hpp>
#include "eigenmvn.h"

using sensor_msgs::msg::Imu;

class ImuSensorConfiguration : public SimObjectConfiguration {
public:
    /// orientation (z only), angular velocity (z only), x/y acceleration
    Eigen::Matrix4f covariance;
};

class ImuSensor : public InternalSensor<Imu> {
protected:
    Imu messageBase;
    Eigen::MultivariateNormal<float> noise;
    virtual Imu getMessage() override;
public:
    ImuSensor(ImuSensorConfiguration conf);
};

#endif