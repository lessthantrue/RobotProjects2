#include "sensableWorld.h"

vector<std::shared_ptr<SensablePoint>> SensableWorld::getPoints(){
    return this->points;
}

void SensableWorld::addPoints(vector<std::shared_ptr<SensablePoint>> newPoints){
    // points.insert(points.end(), newPoints.begin(), newPoints.end());
    for(auto p = newPoints.begin(); p != newPoints.end(); p++){
        points.push_back(*p);
        pointsViz.markers.push_back((*p)->toMarker());
    }
    markerPub->publish(pointsViz);
}

void SensableWorld::addPoint(std::shared_ptr<SensablePoint> p){
    points.push_back(p);
    pointsViz.markers.push_back(p->toMarker());
    markerPub->publish(pointsViz);
}

void SensableWorld::attach(Node::SharedPtr n){
    rclcpp::QoS qos(1);
    qos.transient_local();
    markerPub = n->create_publisher<MarkerArray>("simulation/points", qos);
    attached = true;
}

bool SensableWorld::isAttached(){
    return attached;
}
