#include "sensors/pointCloudSensor.h"

#include <sensor_msgs/msg/channel_float32.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using sensor_msgs::msg::ChannelFloat32;
using geometry_msgs::msg::Point32;
using geometry_msgs::msg::PointStamped;
using geometry_msgs::msg::TransformStamped;

PointCloudSensor::PointCloudSensor(PointSensorBaseConfiguration& conf)
    : PointSensorBase<PointCloud>(conf){
    reading.header.frame_id = conf.frameId;
    ChannelFloat32 rChannel, gChannel, bChannel;
    rChannel.name = "r";
    gChannel.name = "g";
    bChannel.name = "b";
    reading.channels.push_back(rChannel);
    reading.channels.push_back(gChannel);
    reading.channels.push_back(bChannel);
}

PointCloud PointCloudSensor::aggregatePoints(){
    reading.points.clear();
    for(auto c = reading.channels.begin(); c != reading.channels.end(); c++){
        c->values.clear();
    }

    auto pts = world->getPoints();
    for(auto it = pts.begin(); it != pts.end(); it++){
        std::shared_ptr<SensablePoint> pt = *it;
        TransformStamped tf = tfBuffer->lookupTransform(frameId, pt->toMarker().header.frame_id, rclcpp::Time(0));

        PointStamped ptTf1, ptTf2;
        ptTf1.point = pt->toMarker().pose.position;
        ptTf1.header = pt->toMarker().header;
        tf2::doTransform(ptTf1, ptTf2, tf);

        Eigen::Vector2f noisyPtVec;
        noisyPtVec << noise.samples(1);

        Point32 point;
        point.x = ptTf2.point.x + noisyPtVec(0);
        point.y = ptTf2.point.y + noisyPtVec(1);
        point.z = 0;
        ColorRGBA color = pt->getColor();
        reading.channels[0].values.push_back(color.r);
        reading.channels[1].values.push_back(color.g);
        reading.channels[2].values.push_back(color.b);
    }

    return reading;
}