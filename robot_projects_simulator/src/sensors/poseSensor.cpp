#include "sensors/poseSensor.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using geometry_msgs::msg::TransformStamped;

PoseSensor::PoseSensor(SimObjectConfiguration conf)
    : InternalSensor(conf) {}

PoseStamped PoseSensor::getMessage(){
    PoseStamped initial, toRet;
    initial.pose.orientation.w = 1; // make a valid quaternion
    TransformStamped tf = tfBuffer->lookupTransform("map", frameId, rclcpp::Time(0));
    tf2::doTransform(initial, toRet, tf);
    toRet.header.frame_id = "map";
    return toRet;
}
