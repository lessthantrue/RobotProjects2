#ifndef POINTSENSOR_H
#define POINTSENSOR_H

#include "simObject.h"
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

class PointSensorConfiguration : public SimObjectConfiguration {
public:
    string parentFrame;
    float fov, minRange, maxRange;
};

class PointSensor : public SimObject {
private:
    void resetPointCloud();
protected:
    std::shared_ptr<SensableWorld> world;
    PolygonStamped rangeVisualizer;
    pcl::PointCloud<pcl::PointXYZRGB> reading;
    float fov, minRange, maxRange;
    string parentFrame; // the frame this camera is attached to
    Transform transform; // transform from parent_frame to frame_id
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tfPublisher;
    Publisher<PolygonStamped>::SharedPtr rangeVisualizerPublisher;
    Publisher<PointCloud2>::SharedPtr readingPublisher;
    virtual void processPoint(std::shared_ptr<SensablePoint>);
    void timerCallback() override;
    void visualize() override;
public:
    PointSensor(PointSensorConfiguration &);
    void attach(Node::SharedPtr n) override;
    void attachTf(std::shared_ptr<tf2_ros::Buffer>, Node::SharedPtr) override;
    void attachWorld(std::shared_ptr<SensableWorld> world);
};

#endif