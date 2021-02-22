#include <rcl/rcl.h>
#include "simObject.h"
#include "visualizers/poseVisualizer.h"
#include "systems/simpleSE2.h"
#include "simulation.h"
#include "sensablePoint.h"
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

  vector<SensablePoint> points;
  points.emplace_back(0, 1, "map");
  points.emplace_back(2, 0, "map");
  points.emplace_back(-3, -2, "map");
  points.emplace_back(-1, 2, "map");
  points.emplace_back(3, 2, "map");
  sim->addPoints(points);

  SimObjectConfiguration conf;
  conf.name = "diff_drive";
  conf.frameId = "base_link";
  conf.parentFrame = "map";
  conf.loopHz = 20;

  PoseVisualizer viz;
  SimpleSE2 dyn(0, 0, 0);
  sim->addObject(conf, &dyn, &viz);


  for(;;){
    sim->timeStep();
  }

  return 0;
}
