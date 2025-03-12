#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "control_common/utils.h"

namespace control {

class Visualization {
 public:
  explicit Visualization(ros::NodeHandle& nh);
  ~Visualization() = default;
  void publishPointCloud(const PointCloud& pointcloud);
  void publishPointDiscretePointCloud(
      const DiscretePointCloud& discrete_pointcloud);

  ros::NodeHandle nh_;
  ros::Publisher pub_pointcloud_, pub_discrete_pointcloud_;
};

}  // namespace control
