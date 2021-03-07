#ifndef SENSABLE_POINT_H
#define SENSABLE_POINT_H

#include <visualization_msgs/msg/marker.hpp>
#include <string>

using visualization_msgs::msg::Marker;
using std_msgs::msg::ColorRGBA;
using geometry_msgs::msg::Point;
using std::string;

class SensablePoint {
private:
    static int marker_count;
protected:
    Marker marker_base;
    ColorRGBA color;
public:
    SensablePoint(float, float, string);
    float x();
    void setX(float);
    float y();
    void setY(float);
    Marker toMarker();
    Point toPoint(string destFrame);
};

#endif