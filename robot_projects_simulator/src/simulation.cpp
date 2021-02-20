#include "simulation.h"

using std::string;

Simulation::Simulation(Duration timeStep) 
    : Node("simulation"), defaultTimeStep(timeStep){
    // tfBroadcast = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    // need to find some way to make this a latching publisher
    // rclcpp::QoSInitialization qos = QoSInitialization(rclcpp::QoS::transient_local());
    rclcpp::QoS qos(1);
    qos.transient_local();
    markerPub = this->create_publisher<MarkerArray>("simulation/points", qos);
}

void Simulation::addObject(string name, ControlAffineSystem * sys, IPublisher * viz){
    simObjects.emplace_back(name, viz, sys, defaultTimeStep);
}

void Simulation::timeStep(){
    rclcpp::spin_some(this->shared_from_this());
}

void Simulation::addPoints(vector<SensablePoint> newPoints){
    points.insert(points.end(), newPoints.begin(), newPoints.end());
    for(auto p = newPoints.begin(); p != newPoints.end(); p++){
        markerMsg.markers.push_back(p->toMarker());
    }
    markerPub->publish(markerMsg);
}

void Simulation::addPoint(SensablePoint p){
    points.push_back(p);
    markerMsg.markers.push_back(p.toMarker());
    RCLCPP_INFO(this->get_logger(), "We have %d points in the world", markerMsg.markers.size());
    markerPub->publish(markerMsg);
}
