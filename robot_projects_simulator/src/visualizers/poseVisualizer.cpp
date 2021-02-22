#include "visualizers/poseVisualizer.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>

using geometry_msgs::msg::PoseStamped;

PoseStamped PoseVisualizer::getPublishData(){
    return message;
}

void PoseVisualizer::setMessage(ControlAffineSystem * sys){
    message.pose.position.x = sys->getValueByName("x");
    message.pose.position.y = sys->getValueByName("y");
    message.pose.position.z = 0;
    message.header.frame_id = frameId;

    // orientation
    tf2::Quaternion quat;
    quat.setRPY(0, 0, sys->getValueByName("h"));
    message.pose.orientation = tf2::toMsg(quat);
}
