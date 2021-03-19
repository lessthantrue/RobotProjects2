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
    : PointSensor(conf), fov(conf.fov), minRange(conf.minRange), maxRange(conf.maxRange) {
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

void RangeLimitedPointSensor::processPoint(std::shared_ptr<SensablePoint> p){
    // this is where one would use dynamic_cast<p> to test for type
    pcl::PointXYZRGB pclpt(p->toMarker().color.r, p->toMarker().color.g, p->toMarker().color.b);
    TransformStamped tf = tfBuffer->lookupTransform(frameId, p->toMarker().header.frame_id, rclcpp::Time(0));

    PointStamped ptTf1, ptTf2;
    ptTf1.point = p->toMarker().pose.position;
    ptTf1.header = p->toMarker().header;
    tf2::doTransform(ptTf1, ptTf2, tf);

    Point ptTf = ptTf2.point;
    // test if point is in range
    double theta = atan2(ptTf.y, ptTf.x);
    double range = sqrt(ptTf.x * ptTf.x + ptTf.y * ptTf.y);
    if(theta < fov/2 && theta > -fov/2 && range > minRange && range < maxRange){
        // add point to cloud
        pclpt.x = ptTf.x;
        pclpt.y = ptTf.y;
        pclpt.z = 0;
        reading.points.push_back(pclpt);
    }
}

void RangeLimitedPointSensor::attach(Node::SharedPtr n){
    PointSensor::attach(n);
    rangeVisualizerPublisher = n->create_publisher<PolygonStamped>(name + "/visualizer", 1);
}