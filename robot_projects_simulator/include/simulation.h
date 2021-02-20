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

using visualization_msgs::msg::MarkerArray;
using std::vector;
using namespace rclcpp;
using std::string;

class Simulation : public Node {
private:
    Duration defaultTimeStep;
    vector<SimObject> simObjects;
    vector<SensablePoint> points;
    Publisher<MarkerArray>::SharedPtr markerPub;
    MarkerArray markerMsg;
    // make this happen later when I turn simulation into a node
    // std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcast;
public:
    Simulation(Duration timeStep);
    void addObject(string name, ControlAffineSystem * sys, IPublisher * viz);
    void addPoint(SensablePoint p);
    void addPoints(vector<SensablePoint>);
    void timeStep();
};

#endif