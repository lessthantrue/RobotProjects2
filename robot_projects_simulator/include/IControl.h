#ifndef I_CONTROL_H
#define I_CONTROL_H

#include <eigen3/Eigen/Eigen>

class IControl {
public:
    virtual Eigen::VectorXd get() = 0;
};

#endif