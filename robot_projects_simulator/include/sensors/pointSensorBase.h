#ifndef POINT_SENSOR_BASE_H
#define POINT_SENSOR_BASE_H

#include "sensableWorld.h"
#include "sensablePoint.h"
#include "simObject.h"
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

using geometry_msgs::msg::TransformStamped;
using namespace rclcpp;

class PointSensorBaseConfiguration : public SimObjectConfiguration{
    public:
    // for now, simple multivariate gaussian noise for relative position
    Eigen::Matrix2f covariance;
    
    PointSensorBaseConfiguration(){
        // default zero covariance
        covariance << 0, 0, 0, 0;
    }
};

template<typename MsgType>
class PointSensorBase : public SimObject{
protected:
    std::shared_ptr<SensableWorld> world;
    Transform transform; // transform from parent_frame to frame_id
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tfPublisher;
    typename Publisher<MsgType>::SharedPtr readingPublisher;
    Eigen::MultivariateNormal<float> noise;

    virtual MsgType aggregatePoints() = 0;
    void timerCallback() override;
    virtual void visualize() override {};
public:    
    PointSensorBase(PointSensorBaseConfiguration &);
    virtual void attach(Node::SharedPtr n) override;
    void attachTf(std::shared_ptr<tf2_ros::Buffer>, Node::SharedPtr) override;
    void attachWorld(std::shared_ptr<SensableWorld> world);
};

template<typename msgType>
PointSensorBase<msgType>::PointSensorBase(PointSensorBaseConfiguration & conf)
    : SimObject(conf), noise(Eigen::Vector2f::Zero(), conf.covariance) {}

template<typename msgType>
void PointSensorBase<msgType>::timerCallback(){
    msgType msg = aggregatePoints();;
    readingPublisher->publish(msg);
    visualize();
}

template<typename msgType>
void PointSensorBase<msgType>::attach(Node::SharedPtr n){
    SimObject::attach(n);
    readingPublisher = n->create_publisher<msgType>(name + "/reading", 10);
}

template<typename msgType>
void PointSensorBase<msgType>::attachWorld(std::shared_ptr<SensableWorld> newWorld){
    this->world = newWorld;
}

template<typename msgType>
void PointSensorBase<msgType>::attachTf(std::shared_ptr<tf2_ros::Buffer> buf, Node::SharedPtr n){
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

#endif