#ifndef POINT_CLOUD_SENSOR_H
#define POINT_CLOUD_SENSOR_H

#include <sensor_msgs/msg/point_cloud.hpp>
#include "sensors/pointSensorBase.h"

using sensor_msgs::msg::PointCloud;

class PointCloudSensor : public PointSensorBase<PointCloud> {
protected:
    PointCloud reading;
    PointCloud aggregatePoints() override;
public:
    PointCloudSensor(PointSensorBaseConfiguration&);
};

#endif