#ifndef SYSTEM_H
#define SYSTEM_H

#include <eigen3/Eigen/Eigen>
#include <map>
#include <string>
#include <geometry_msgs/msg/transform.hpp>
#include "eigenmvn.h"

using std::string;
using std::map;
using Eigen::VectorXd;
using geometry_msgs::msg::Transform;

class System {
protected:
    /**
     * Size of the state and control vectors
     **/
    int _dimX, _dimU;

    /**
     * Time derivative of the system state
     **/
    virtual VectorXd dxdt(VectorXd u) = 0;

    /**
     * External noise or disturbance
     **/
    virtual VectorXd noise(VectorXd u);

    /**
     * A map relating individual state variable names
     * to the indices containing them in state
     **/
    map<string, int> semanticStateMap;

    /**
     * True if the system's inputs have bounds
     **/
    bool hasInputBounds;

    /** 
     * High and low limits on the system's inputs
     **/
    VectorXd uLimitHigh; 
    VectorXd uLimitLow;
    
    /**
     * noise generator for the system
     **/
    Eigen::MultivariateNormal<double> noiseGenerator;

public:
    int dimX();
    int dimU();

    /**
     * The system state vector
     **/
    VectorXd state;

    /** 
     * Construct a new system with a given initial state
     **/
    System(VectorXd initial);

    /**
     * Steps the system forwards through time with numerical integration
     **/ 
    void step(VectorXd u, double dt);

    /**
     * Gets the state variable corresponding to the given name
     **/
    double getValueByName(string name);

    /**
     * Returns a transform from the parent frame to the system frame
     **/
    virtual Transform getTransform() = 0;

    /** 
     * set the process noise covariance
     **/
    void setNoiseCovariance(Eigen::MatrixXd cov);
};

#endif