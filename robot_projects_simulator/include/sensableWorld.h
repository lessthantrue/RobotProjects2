#ifndef SENSABLE_WORLD_H
#define SENSABLE_WORLD_H

#include <rclcpp/rclcpp.hpp>
#include "sensablePoint.h"
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace rclcpp;
using std::vector;
using visualization_msgs::msg::MarkerArray;

// the outside world as it pertains to any external sensor
// external sensors being cameras, 2D lidar, light/temperature/humidity
class SensableWorld {
private:
    MarkerArray pointsViz;
    Publisher<MarkerArray>::SharedPtr markerPub;
    vector<std::shared_ptr<SensablePoint>> points;
    bool attached;
public:
    vector<std::shared_ptr<SensablePoint>> getPoints();
    void addPoint(std::shared_ptr<SensablePoint> p);
    void addPoints(vector<std::shared_ptr<SensablePoint>> pts);
    void attach(Node::SharedPtr n);
    bool isAttached();
};

#endif 