#ifndef SENSABLE_POINT_H
#define SENSABLE_POINT_H

#include <visualization_msgs/msg/marker.hpp>
#include <string>

using visualization_msgs::msg::Marker;
using std_msgs::msg::ColorRGBA;
using std::string;

class SensablePoint {
private:
    Marker marker_base;
    static int marker_count;
public:
    SensablePoint(float, float, string);
    float x();
    void setX(float);
    float y();
    void setY(float);
    Marker toMarker();
};

#endif