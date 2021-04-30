#ifndef SIMPLE_SE2_H
#define SIMPLE_SE2_H

#include "controlAffineSystem.h"
#include <geometry_msgs/msg/twist.hpp>
#include "rosControlInterface.h"

using namespace geometry_msgs::msg;
// X = (x, y, h)
// U = (v, w)

class SimpleSE2 : public ControlAffineSystem {
public:
    SimpleSE2(double x, double y, double h);
    Eigen::MatrixXd g() override;
    Transform getTransform() override;

    class CmdVelInterface : public RosControlInterface<Twist> {
        Eigen::VectorXd get() {
            return Eigen::Vector2d(latest.linear.x, latest.angular.z);
        }
    };
};
    
#endif