#include "sensors/rangeLimitedPointSensor.h"

#include <geometry_msgs/msg/point32.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <iostream>

using geometry_msgs::msg::Point32;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::PointStamped;
using sensor_msgs::msg::PointCloud2;
using geometry_msgs::msg::TransformStamped;

RangeLimitedPointSensor::RangeLimitedPointSensor(RangeLimitedPointSensorConfiguration &conf)
    : PointSensorBase<PointCloud2>(conf), fov(conf.fov), minRange(conf.minRange), maxRange(conf.maxRange) {
    rangeVisualizer.header.frame_id = conf.frameId;

    // fill polygon points
    // for now, just 3 points inside and outside
    // later on should be dependent on fov (i.e 1 point for every 30 degrees)
    Point32 pt;
    pt.x = cos(-fov / 2) * minRange;
    pt.y = sin(-fov / 2) * minRange;
    rangeVisualizer.polygon.points.push_back(pt);
    pt.x = cos(0) * minRange;
    pt.y = sin(0) * minRange;
    rangeVisualizer.polygon.points.push_back(pt);
    pt.x = cos(fov / 2) * minRange;
    pt.y = sin(fov / 2) * minRange;
    rangeVisualizer.polygon.points.push_back(pt);
    pt.x = cos(fov / 2) * maxRange;
    pt.y = sin(fov / 2) * maxRange;
    rangeVisualizer.polygon.points.push_back(pt);
    pt.x = cos(0) * maxRange;
    pt.y = sin(0) * maxRange;
    rangeVisualizer.polygon.points.push_back(pt);
    pt.x = cos(-fov / 2) * maxRange;
    pt.y = sin(-fov / 2) * maxRange;
    rangeVisualizer.polygon.points.push_back(pt);
}

void RangeLimitedPointSensor::visualize(){
    rangeVisualizerPublisher->publish(rangeVisualizer);
}

PointCloud2 RangeLimitedPointSensor::aggregatePoints(){
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
        
        double theta = atan2(ptTf2.point.y, ptTf2.point.x);
        double range = sqrt(ptTf2.point.x * ptTf2.point.x + ptTf2.point.y * ptTf2.point.y);
        if(theta < fov/2 && theta > -fov/2 && range > minRange && range < maxRange){
            // add point to cloud
            pclpt.x = ptTf2.point.x;
            pclpt.y = ptTf2.point.y;
            pclpt.z = 0;
            reading.points.push_back(pclpt);
        }

        PointCloud2 pc2_msg;
        pcl::toROSMsg(reading, pc2_msg);
        pc2_msg.header.frame_id = frameId;
        pc2_msg.header.stamp = clock->now();
        return pc2_msg;
    }
}

void RangeLimitedPointSensor::attach(Node::SharedPtr n){
    PointSensorBase<PointCloud2>::attach(n);
    rangeVisualizerPublisher = n->create_publisher<PolygonStamped>(name + "/visualizer", 1);
}