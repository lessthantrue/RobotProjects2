#ifndef RVIZABLE_H
#define RVIZABLE_H

#include <rcl/rcl.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include "ipublisher.h"

template <class T>
class RVizable : public IPublisher {
private:
    typename rclcpp::Publisher<T>::SharedPtr rvizPublisher;
protected:
    virtual T getPublishData() = 0;
public:
    void attach(std::shared_ptr<Node>, std::string name) override;
    void publish() override;
};

template <class T>
void RVizable<T>::attach(std::shared_ptr<Node> node, std::string name){
    rvizPublisher = node->create_publisher<T>(name, 10);
}

template <class T>
void RVizable<T>::publish(){
    rvizPublisher->publish(this->getPublishData());
}

#endif