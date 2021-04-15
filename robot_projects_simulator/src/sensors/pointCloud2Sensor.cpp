#include "sensors/pointCloud2Sensor.h"

#include <geometry_msgs/msg/point32.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <pcl_conversions/pcl_conversions.h>

using geometry_msgs::msg::Point32;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::PointStamped;
using sensor_msgs::msg::PointCloud2;
using geometry_msgs::msg::TransformStamped;

PointCloud2Sensor::PointCloud2Sensor(PointSensorBaseConfiguration conf)
    : PointSensorBase<PointCloud2>(conf) {}

PointCloud2 PointCloud2Sensor::aggregatePoints(){
    auto pts = world->getPoints();
    reading.points.clear();
    for(auto it = pts.begin(); it != pts.end(); it++){
        std::shared_ptr<SensablePoint> pt = *it;
        TransformStamped tf = tfBuffer->lookupTransform(frameId, pt->toMarker().header.frame_id, rclcpp::Time(0));

        PointStamped ptTf1, ptTf2;
        ptTf1.point = pt->toMarker().pose.position;
        ptTf1.header = pt->toMarker().header;
        tf2::doTransform(ptTf1, ptTf2, tf);

        Eigen::Vector2f noisyPtVec;
        noisyPtVec << noise.samples(1);

        // add points to cloud
        pcl::PointXYZRGB pclpt(pt->toMarker().color.r, pt->toMarker().color.g, pt->toMarker().color.b);
        pclpt.x = ptTf2.point.x + noisyPtVec(0);
        pclpt.y = ptTf2.point.y + noisyPtVec(1);
        pclpt.z = 0;
        reading.points.push_back(pclpt);
    }

    PointCloud2 pc2_msg;
    pcl::toROSMsg(reading, pc2_msg);
    pc2_msg.header.frame_id = frameId;
    pc2_msg.header.stamp = clock->now();
    return pc2_msg;
}
