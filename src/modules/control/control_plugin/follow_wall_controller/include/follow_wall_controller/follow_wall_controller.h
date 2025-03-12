#pragma once

// 控制插件、控制算法、控制状态机相互解耦
#include "control_common/control_base.h"
#include "control_common/hsm.h"
#include "control_common/utils.h"

namespace control {

class FollowWallController : public PluginBase {
 public:
  FollowWallController() = default;
  ~FollowWallController() = default;

  bool init(const std::string& name,
            const DependencyInjector::Ptr& injector) override;
  PluginStage run() override;

 private:
  double getLateralDistanceFromScan(const double& angle1, const double& angle2);
  void publishPointCloud();
  void publishPointDiscretePointCloud();

 private:
  std::shared_ptr<hsm::StateMachine> state_machine_;

  ros::NodeHandle nh_;
  Eigen::Isometry3d base_to_laser_;
  PointCloud pointcloud_;                   //激光点云
  DiscretePointCloud discrete_pointcloud_;  //离散的激光点云
  ros::Publisher pub_pointcloud_, pub_discrete_pointcloud_;
  //雷达数据去畸变、雷达数据坐标系变换、雷达数据稀疏化
};

}  // namespace control