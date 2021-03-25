#ifndef IPUBLISHER_H
#define IPUBLISHER_H

#include <rclcpp/rclcpp.hpp>
#include <string>
#include "systems/system.h"

using std::string;
using namespace rclcpp;

class VisualizerConfiguration {
public:
    string topicName;
    string frameId;
};

// rename to something like 'IStatePublisher' in the future
class IPublisher{
protected:
    string frameId;
public:
    virtual void publish() = 0;
    virtual void attach(std::shared_ptr<Node>, string) = 0;
    virtual void setMessage(System *) = 0;
    void setFrameId(string s) { frameId = s; }
};

#endif