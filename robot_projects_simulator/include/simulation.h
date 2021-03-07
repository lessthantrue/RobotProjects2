#ifndef SIMULATION_H
#define SIMULATION_H

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <tf2_ros/transform_broadcaster.h>
#include "simObject.h"
#include "sensableWorld.h"
#include "ipublisher.h"
#include "systems/controlAffineSystem.h"
#include "sensablePoint.h"
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>

using visualization_msgs::msg::MarkerArray;
using std::vector;
using namespace rclcpp;
using std::string;

class Simulation : public Node {
private:
    Duration defaultTimeStep;
    vector<std::shared_ptr<SimObject>> simObjects;
    std::shared_ptr<SensableWorld> sworld;
    std::shared_ptr<tf2_ros::Buffer> buffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;
public:
    Simulation(Duration timeStep);
    void addObject(SimObject::SharedPtr obj);
    void timeStep();
    std::shared_ptr<SensableWorld> sensableWorld();
};

#endif