#ifndef POINT_CLOUD_2_SENSOR_H
#define POINT_CLOUD_2_SENSOR_H

#include "pointSensorBase.h"
#include "sensablePoint.h"
#include "sensableWorld.h"
#include <geometry_msgs/msg/transform.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <eigen3/Eigen/Eigen>
#include "eigenmvn.h"

using geometry_msgs::msg::Transform;
using sensor_msgs::msg::PointCloud2;
using namespace rclcpp;

class PointCloud2Sensor : public PointSensorBase<PointCloud2> {
protected:
    pcl::PointCloud<pcl::PointXYZRGB> reading;

    PointCloud2 aggregatePoints() override;
public:
    string getParentFrameId() { return parentFrameId; } // may not need this anymore?
    PointCloud2Sensor(PointSensorBaseConfiguration);
};

#endif