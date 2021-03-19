// #include "systems/simpleSE2.h"
#include "systems/simpleSE2.h"
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>

SimpleSE2::SimpleSE2(double x, double y, double h)
    : ControlAffineSystem(Eigen::Vector3d(x, y, h)){
    semanticStateMap["x"] = 0;
    semanticStateMap["y"] = 1;
    semanticStateMap["h"] = 2;
    _dimX = 3;
    _dimU = 2;
    uLimitHigh = Eigen::Vector2d(1, 1);
    uLimitLow = Eigen::Vector2d(-1, -1);
}

Eigen::MatrixXd SimpleSE2::g() {
    Eigen::MatrixXd gx(dimX(), dimU());
    gx << cos(state(2)), 0,
          sin(state(2)), 0,
          0, 1;
    return gx;
}

geometry_msgs::msg::Transform SimpleSE2::getTransform(){
    geometry_msgs::msg::Transform t;
    t.translation.x = getValueByName("x");
    t.translation.y = getValueByName("y");
    t.translation.z = 0;

    tf2::Quaternion quat;
    quat.setRPY(0, 0, getValueByName("h"));
    t.rotation = tf2::toMsg(quat);
    return t;
}
