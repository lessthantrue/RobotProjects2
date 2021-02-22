#include "../rvizable.h"
#include <geometry_msgs/msg/pose_stamped.hpp>

using std::string;
using geometry_msgs::msg::PoseStamped;

class PoseVisualizer : public RVizable<PoseStamped>{
private:
    PoseStamped message;
protected:
    PoseStamped getPublishData() override;
public:
    void setMessage(ControlAffineSystem *) override;
};