#include "../rvizable.h"
#include <geometry_msgs/msg/pose_stamped.hpp>

using std::string;
using geometry_msgs::msg::PoseStamped;

class PoseVisualizer : public RVizable<PoseStamped>{
private:
    PoseStamped message;
protected:
    string frameId;
    PoseStamped getPublishData() override;
public:
    void setFrame(string s);
    void setMessage(ControlAffineSystem *) override;
};