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

  // std::shared_ptr<Simulation> sim = std::make_shared<Simulation>(rclcpp::Duration(33 * 1000 * 1000));

  // vector<std::shared_ptr<SensablePoint>> points;
  // points.push_back(std::make_shared<SensablePoint>(0, 1, "map"));
  // points.push_back(std::make_shared<SensablePoint>(-2, 1, "map"));
  // points.push_back(std::make_shared<SensablePoint>(3, -2, "map"));
  // points.push_back(std::make_shared<SensablePoint>(-1, 1, "map"));
  // points.push_back(std::make_shared<SensablePoint>(2, 1, "map"));
  // sim->sensableWorld()->addPoints(points);

  // DynamicSimObjectConfiguration conf;
  // conf.name = "diff_drive";
  // conf.frameId = "base_link";
  // conf.parentFrameId = "map";
  // conf.loopHz = 20;

  // PointSensorBaseConfiguration sensConf;
  // sensConf.frameId = "camera";
  // sensConf.parentFrameId = "base_link";
  // sensConf.name = "pt_sensor";
  // sensConf.loopHz = 10;
  // sensConf.covariance << 0.05, 0, 0, 0.05;

  // SimObjectConfiguration poseConf;
  // poseConf.frameId = "base_link";
  // poseConf.name = "pose_sensor";
  // poseConf.loopHz = 1;

  // PoseVisualizer viz;
  // SimpleSE2 dyn(0, 0, 0);

  // std::shared_ptr<PointCloud2Sensor> sensor = std::make_shared<PointCloud2Sensor>(sensConf);
  // sensor->attachWorld(sim->sensableWorld());
  // SimObject::SharedPtr obj = std::make_shared<DynamicSimObject>(conf, &viz, &dyn);
  // SimObject::SharedPtr pose = std::make_shared<PoseSensor>(poseConf);
  // sim->addObject(obj);
  // sim->addObject(sensor);
  // sim->addObject(pose);

  // for(;;){
  //   sim->timeStep();
  // }

  return 0;
}
