#ifndef RANGE_LIMITED_POINT_SENSOR_H
#define RANGE_LIMITED_POINT_SENSOR_H

#include "sensors/pointSensor.h"
#include "sensablePoint.h"
#include "sensableWorld.h"
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>

using geometry_msgs::msg::PolygonStamped;
using geometry_msgs::msg::Transform;
using sensor_msgs::msg::PointCloud2;
using namespace rclcpp;

class RangeLimitedPointSensorConfiguration : public SimObjectConfiguration {
public:
    float fov, minRange, maxRange;
};

class RangeLimitedPointSensor : public PointSensor {
protected:
    PolygonStamped rangeVisualizer;
    float fov, minRange, maxRange;
    Publisher<PolygonStamped>::SharedPtr rangeVisualizerPublisher;
    virtual void processPoint(std::shared_ptr<SensablePoint>);
    virtual void visualize() override;
public:
    RangeLimitedPointSensor(RangeLimitedPointSensorConfiguration &);
    void attach(Node::SharedPtr n) override;
};

#endif