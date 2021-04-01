#include "sensors/imuSensor.h"
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/LinearMath/Quaternion.h>

using geometry_msgs::msg::QuaternionStamped;
using geometry_msgs::msg::TransformStamped;

ImuSensor::ImuSensor(ImuSensorConfiguration conf)
    : InternalSensor<Imu>(conf), 
      noise(Eigen::Vector4f::Zero(), conf.covariance) {
    messageBase.header.frame_id = frameId;
    
    // zero existing covariances
    for(int i = 0; i < 9; i++){
        messageBase.orientation_covariance[i] = 0;
        messageBase.angular_velocity_covariance[i] = 0;
        messageBase.linear_acceleration_covariance[i] = 0;
    }

    // write covariances out
    messageBase.orientation_covariance[8] = conf.covariance(0, 0);
    messageBase.angular_velocity_covariance[0] = -1;
    messageBase.linear_acceleration_covariance[0] = -1;
}

Imu ImuSensor::getMessage(){    
    Eigen::Vector4f noiseValue;
    noiseValue <<  noise.samples(1);

    // create an orientation in this sensor's frame, transform it to the map frame
    tf2::Quaternion quat_tf, quat_noise;
    quat_tf.setRPY(0, 0, noiseValue(0));
    QuaternionStamped qs_initial, qs_final;
    qs_initial.quaternion = tf2::toMsg(quat_tf);
    TransformStamped tf = tfBuffer->lookupTransform("map", frameId, rclcpp::Time(0));
    qs_initial.header.frame_id = frameId;
    tf2::doTransform(qs_initial, qs_final, tf);

    // add noise
    // quat_tf = tf2::fromMsg(qs_final, quat_tf);
    // quat_noise.setRPY(0, 0, noiseValue(0));
    // quat_tf *= quat_noise;
    // qs_final.quaternion = tf2::toMsg(quat_tf);

    messageBase.orientation = qs_final.quaternion;
    return messageBase;
}
