#include "sensablePoint.h"

int SensablePoint::marker_count = 0;

SensablePoint::SensablePoint(float x, float y, string frame_id) {
    marker_base.type = Marker::SPHERE;
    ColorRGBA color;
    color.r = 1;
    color.g = 0;
    color.b = 0;
    color.a = 1;
    marker_base.color = color;
    marker_base.header.frame_id = frame_id;
    marker_base.scale.x = 0.1;
    marker_base.scale.y = 0.1;
    marker_base.scale.z = 0.1;
    marker_base.id = marker_count;
    marker_count++;
    setX(x);
    setY(y);
}

float SensablePoint::x(){
    return marker_base.pose.position.x;
}

float SensablePoint::y(){
    return marker_base.pose.position.y;
}

void SensablePoint::setX(float x){
    marker_base.pose.position.x = x;
}

void SensablePoint::setY(float y){
    marker_base.pose.position.y = y;
}

Marker SensablePoint::toMarker(){
    return marker_base;
}
