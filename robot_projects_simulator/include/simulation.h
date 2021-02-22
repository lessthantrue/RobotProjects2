#ifndef SIMULATION_H
#define SIMULATION_H

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <tf2_ros/transform_broadcaster.h>
#include "simObject.h"
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
    vector<SensablePoint> points;
    Publisher<MarkerArray>::SharedPtr markerPub;
    MarkerArray markerMsg;
    std::shared_ptr<tf2_ros::Buffer> buffer;
    // make this happen later when I turn simulation into a node
    // std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcast;
public:
    Simulation(Duration timeStep);
    void addObject(SimObjectConfiguration conf, ControlAffineSystem * sys, IPublisher * viz);
    void addPoint(SensablePoint p);
    void addPoints(vector<SensablePoint>);
    void timeStep();
};

#endif