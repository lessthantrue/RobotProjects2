#include "sensors/imuSensor.h"
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/LinearMath/Quaternion.h>

using geometry_msgs::msg::QuaternionStamped;
using geometry_msgs::msg::TransformStamped;

ImuSensor::ImuSensor(SimObjectConfiguration conf)
    : InternalSensor<Imu>(conf) {}

Imu ImuSensor::getMessage(){
    Imu toRet;
    toRet.header.frame_id = frameId;
    
    // create an orientation in this sensor's frame, transform it to the map frame
    tf2::Quaternion quat;
    quat.setRPY(0, 0, 0);
    QuaternionStamped qs_initial, qs_final;
    qs_initial.quaternion = tf2::toMsg(quat);
    TransformStamped tf = tfBuffer->lookupTransform(frameId, "map", rclcpp::Time(0));
    qs_initial.header.frame_id = frameId;
    tf2::doTransform(qs_initial, qs_final, tf);
    toRet.orientation = qs_final.quaternion;
    toRet.orientation_covariance[0] = 0;

    // TODO: linear acceleration, angular velocity
    toRet.linear_acceleration_covariance[0] = -1;
    toRet.angular_velocity_covariance[0] = -1;
    return toRet;
}
