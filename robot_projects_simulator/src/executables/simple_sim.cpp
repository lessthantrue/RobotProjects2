#include <rcl/rcl.h>
#include "dynamicSimObject.h"
#include "visualizers/poseVisualizer.h"
#include "systems/simpleSE2.h"
#include "simulation.h"
#include "sensors/pointSensor.h"
#include "sensablePoint.h"
#include "sensors/imuSensor.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <chrono>

using namespace std::chrono_literals;
using geometry_msgs::msg::PoseStamped;
using std::vector;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<Simulation> sim = std::make_shared<Simulation>(rclcpp::Duration(33 * 1000 * 1000));

  vector<std::shared_ptr<SensablePoint>> points;
  points.push_back(std::make_shared<SensablePoint>(0, 1, "map"));
  points.push_back(std::make_shared<SensablePoint>(-2, 1, "map"));
  points.push_back(std::make_shared<SensablePoint>(3, -2, "map"));
  points.push_back(std::make_shared<SensablePoint>(-1, 1, "map"));
  points.push_back(std::make_shared<SensablePoint>(2, 1, "map"));
  sim->sensableWorld()->addPoints(points);

  DynamicSimObjectConfiguration conf;
  conf.name = "diff_drive";
  conf.frameId = "base_link";
  conf.parentFrame = "map";
  conf.loopHz = 20;

  PointSensorConfiguration sensConf;
  sensConf.frameId = "camera";
  sensConf.parentFrame = "base_link";
  sensConf.fov = M_PI_2;
  sensConf.minRange = 1;
  sensConf.maxRange = 5;
  sensConf.name = "pt_camera";
  sensConf.loopHz = 10; // readings per second

  SimObjectConfiguration imuConf;
  imuConf.frameId = "base_link";
  imuConf.name = "imu";
  imuConf.loopHz = 50;

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
