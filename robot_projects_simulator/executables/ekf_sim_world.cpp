#include <rcl/rcl.h>
#include "dynamicSimObject.h"
#include "visualizers/poseVisualizer.h"
#include "systems/simpleSE2.h"
#include "simulation.h"
#include "sensors/pointCloud2Sensor.h"
#include "sensors/poseSensor.h"
#include "sensablePoint.h"
#include "sensors/imuSensor.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <chrono>
#include <iostream>

using namespace std::chrono_literals;
using geometry_msgs::msg::PoseStamped;
using std::vector;

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<Simulation> sim = std::make_shared<Simulation>(rclcpp::Duration(33 * 1000 * 1000));

    vector<std::shared_ptr<SensablePoint>> points;
    points.push_back(std::make_shared<SensablePoint>(3, -2, "map"));
    sim->sensableWorld()->addPoints(points);

    DynamicSimObjectConfiguration conf;
    conf.name = "diff_drive";
    conf.frameId = "base_link";
    conf.parentFrameId = "map";
    conf.loopHz = 50;

    PointSensorBaseConfiguration sensConf;
    sensConf.frameId = "camera";
    sensConf.parentFrameId = "base_link";
    sensConf.name = "pt_sensor";
    sensConf.loopHz = 20;
    sensConf.covariance << 0.01, 0, 0, 0.01;

    SimObjectConfiguration poseSensorConf;
    poseSensorConf.name = "pose_sensor";
    poseSensorConf.frameId = "base_link";
    poseSensorConf.loopHz = 20;
    poseSensorConf.parentFrameId = "map";

    ImuSensorConfiguration imuConf;
    imuConf.frameId = "base_link";
    imuConf.name = "imu";
    imuConf.loopHz = 50;
    imuConf.covariance << 
        0.01 * M_PI, 0, 0, 0,
        0, 0, 0, 0, 
        0, 0, 0, 0,
        0, 0, 0, 0;

    PoseVisualizer viz;
    SimpleSE2 dyn(0, 0, 0);
    Eigen::MatrixXd cov(3, 3);
    cov << 
        0.01, 0, 0,
        0, 0.01, 0,
        0, 0, 0.1;
    dyn.setNoiseCovariance(cov);

    // the next line is extremely dumb and I hate it and I should change it
    std::shared_ptr<SimpleSE2::CmdVelInterface> input = std::make_shared<SimpleSE2::CmdVelInterface>();
    input->attach(sim, "/cmd_vel");

    std::shared_ptr<PointCloud2Sensor> sensor = std::make_shared<PointCloud2Sensor>(sensConf);
    sensor->attachWorld(sim->sensableWorld());
    std::shared_ptr<DynamicSimObject> obj = std::make_shared<DynamicSimObject>(conf, &viz, &dyn);
    obj->setControl(input);
    std::shared_ptr<PoseSensor> poseSensor = std::make_shared<PoseSensor>(poseSensorConf);
    SimObject::SharedPtr imu = std::make_shared<ImuSensor>(imuConf);
    sim->addObject(obj);
    sim->addObject(sensor);
    sim->addObject(imu);
    sim->addObject(poseSensor);

    for(;;){
        sim->timeStep();
    }

    return 0;
}
