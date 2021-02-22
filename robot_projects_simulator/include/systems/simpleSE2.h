#include "controlAffineSystem.h"

// X = (x, y, h)
// U = (v, w)

class SimpleSE2 : public ControlAffineSystem {
public:
    SimpleSE2(double x, double y, double h);
    Eigen::MatrixXd g() override;
    geometry_msgs::msg::Transform getTransform() override;
};
    