#include "sensors/pointSensor.h"

#include <geometry_msgs/msg/point32.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <iostream>

using geometry_msgs::msg::Point32;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::PointStamped;
using sensor_msgs::msg::PointCloud2;
using geometry_msgs::msg::TransformStamped;

PointSensor::PointSensor(SimObjectConfiguration conf)
    : SimObject(conf) { }

void PointSensor::timerCallback(){
    reading.points.clear();
    auto points = world->getPoints();
    for(auto v = points.begin(); v != points.end(); v++){
        try{
            tfBuffer->canTransform(frameId, (*v)->toMarker().header.frame_id, rclcpp::Time(0));
        } catch(tf2::LookupException e) { 
            continue;
        }
        processPoint(*v);
    }
    
    PointCloud2 pc2_msg;
    pcl::toROSMsg(reading, pc2_msg);
    pc2_msg.header.frame_id = frameId;
    pc2_msg.header.stamp = clock->now();
    readingPublisher->publish(pc2_msg);
    visualize();
}

void PointSensor::processPoint(std::shared_ptr<SensablePoint> p){
    // in this case, transform all points to camera frame and add them as is
    TransformStamped tf = tfBuffer->lookupTransform(frameId, p->toMarker().header.frame_id, rclcpp::Time(0));

    PointStamped ptTf1, ptTf2;
    ptTf1.point = p->toMarker().pose.position;
    ptTf1.header = p->toMarker().header;
    tf2::doTransform(ptTf1, ptTf2, tf);

    Point ptTf = ptTf2.point;
    // add point to cloud
    pcl::PointXYZRGB pclpt(p->toMarker().color.r, p->toMarker().color.g, p->toMarker().color.b);
    pclpt.x = ptTf.x;
    pclpt.y = ptTf.y;
    pclpt.z = 0;
    reading.points.push_back(pclpt);
}

void PointSensor::attach(Node::SharedPtr n){
    SimObject::attach(n);
    readingPublisher = n->create_publisher<PointCloud2>(name + "/reading", 10);
}

void PointSensor::attachWorld(std::shared_ptr<SensableWorld> newWorld){
    this->world = newWorld;
}

void PointSensor::attachTf(std::shared_ptr<tf2_ros::Buffer> buf, Node::SharedPtr n){
    SimObject::attachTf(buf, n);
    tfPublisher = std::make_shared<tf2_ros::StaticTransformBroadcaster>(n);
    TransformStamped ts;
    ts.header.stamp = clock->now();
    RCLCPP_INFO(n->get_logger(), "%s -> %s", frameId.c_str(), parentFrameId.c_str());
    ts.transform = transform;
    ts.child_frame_id = frameId;
    ts.header.frame_id = parentFrameId;
    tfPublisher->sendTransform(ts);
}