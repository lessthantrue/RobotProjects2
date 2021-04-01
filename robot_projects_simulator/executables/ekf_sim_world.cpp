#include <rcl/rcl.h>
#include "dynamicSimObject.h"
#include "visualizers/poseVisualizer.h"
#include "systems/simpleSE2.h"
#include "simulation.h"
#include "sensors/pointSensor.h"
#include "sensors/poseSensor.h"
#include "sensors/rangeLimitedPointSensor.h"
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
  conf.loopHz = 20;

  PointSensorConfiguration sensConf;
  sensConf.frameId = "camera";
  sensConf.parentFrameId = "base_link";
  sensConf.name = "pt_sensor";
  sensConf.loopHz = 10;
  sensConf.covariance << 0.05, 0, 0, 0.05;

  ImuSensorConfiguration imuConf;
  imuConf.frameId = "base_link";
  imuConf.name = "imu";
  imuConf.loopHz = 50;
  imuConf.covariance << 0.1 * M_PI, 0, 0, 0,
                        0, 0, 0, 0, 
                        0, 0, 0, 0,
                        0, 0, 0, 0;

  PoseVisualizer viz;
  SimpleSE2 dyn(0, 0, 0);

  std::shared_ptr<PointSensor> sensor = std::make_shared<PointSensor>(sensConf);
  sensor->attachWorld(sim->sensableWorld());
  SimObject::SharedPtr obj = std::make_shared<DynamicSimObject>(conf, &viz, &dyn);
  SimObject::SharedPtr imu = std::make_shared<ImuSensor>(imuConf);
  sim->addObject(obj);
  sim->addObject(sensor);
  sim->addObject(imu);

  for(;;){
    sim->timeStep();
  }

  return 0;
}
