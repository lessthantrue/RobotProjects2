#ifndef POINT_SENSOR_H
#define POINT_SENSOR_H

#include "simObject.h"
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

class PointSensorConfiguration : public SimObjectConfiguration {
public:
    // for now, simple multivariate gaussian noise for relative position
    Eigen::Matrix2f covariance;
    
    PointSensorConfiguration(){
        // default zero covariance
        covariance << 0, 0, 0, 0;
    }
};

class PointSensor : public SimObject{
private:
    void resetPointCloud();
protected:
    std::shared_ptr<SensableWorld> world;
    pcl::PointCloud<pcl::PointXYZRGB> reading;
    Transform transform; // transform from parent_frame to frame_id
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tfPublisher;
    Publisher<PointCloud2>::SharedPtr readingPublisher;
    Eigen::MultivariateNormal<float> noise;

    virtual void processPoint(std::shared_ptr<SensablePoint>);
    void timerCallback() override;
    virtual void visualize() override {};
public:
    string getParentFrameId() { return parentFrameId; }
    PointSensor(PointSensorConfiguration);
    virtual void attach(Node::SharedPtr n) override;
    void attachTf(std::shared_ptr<tf2_ros::Buffer>, Node::SharedPtr) override;
    void attachWorld(std::shared_ptr<SensableWorld> world);
};

#endif