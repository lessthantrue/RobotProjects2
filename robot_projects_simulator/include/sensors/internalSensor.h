#ifndef INTERNAL_SENSOR_H
#define INTERNAL_SENSOR_H

#include "systems/system.h"
#include "simObject.h"
#include <rclcpp/rclcpp.hpp>
// #include <rcl/rcl.h>

using namespace rclcpp;

/**
 * A sensor that publishes information derived from a system's state
 * Examples: accelerometer, gyroscope, (simplified) gps
 **/
template <class T>
class InternalSensor : public SimObject {
protected:
    std::shared_ptr<System> sys;
    std::shared_ptr<Publisher<T>> publisher;
    virtual T getMessage() = 0;
    virtual void timerCallback() override;
public:
    virtual void attach(std::shared_ptr<Node> n) override;
    InternalSensor<T>(SimObjectConfiguration conf) : SimObject(conf) {}
};

template <class T>
void InternalSensor<T>::timerCallback(){
    publisher->publish(getMessage());
}

template <class T>
void InternalSensor<T>::attach(std::shared_ptr<Node> n){
    SimObject::attach(n);
    publisher = n->create_publisher<T>(name + "/reading", 10);
}

#endif