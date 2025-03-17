#include "follow_wall_controller/follow_wall_controller.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(control::FollowWallController, control::PluginBase)

namespace control {

bool FollowWallController::init(const std::string& name,
                                const DependencyInjector::Ptr& injector) {
  std::cout << "沿墙初始化" << std::endl;
  name_ = name;
  injector_ = injector;
  std::cout << "沿墙创建状态机" << std::endl;
  state_machine_ = std::make_shared<hsm::StateMachine>();
  std::cout << "沿墙初始化状态机" << std::endl;
  state_machine_->Initialize<StateRoot>(this);

  follow_dir_ = -1.0;
  //插件初始化中获得ROS参数，
  //单独增加一个setGoal的函数在插件中
  //创建发布器
  ros::NodeHandle nh;
  params_ = boost::make_shared<Params>(nh);
  visualization_ = boost::make_shared<Visualization>(nh);

  base_to_laser_ = Eigen::Isometry3d::Identity();
  return true;
}

PluginStage FollowWallController::run() {
  ros::Time start_time = ros::Time::now();
  bool ok = toBaselink(injector_->scan_, base_to_laser_, injector_->odom_deque_,
                       pointcloud_);
  discretePointcloud(pointcloud_, discrete_pointcloud_);
  visualization_->publishPointCloud(pointcloud_);
  visualization_->publishPointDiscretePointCloud(discrete_pointcloud_);

  static int i = 0;
  PluginStage stage;
  i++;
  if (i < 10000) {
    stage.stage = PluginStage::Stage::RUNNING;
  } else {
    stage.stage = PluginStage::Stage::SUCCEEDED;
  }
  state_machine_->ProcessStateTransitions();
  state_machine_->UpdateStates();
  ros::Time end_time = ros::Time::now();
  double dt = (end_time - start_time).toSec();
  ROS_INFO("dt = %f", dt);
  usleep(1000 * 50);  // TODO:check this
  return stage;
}

double FollowWallController::getFollowDir() const { return follow_dir_; }

double FollowWallController::getLateralDistanceFromScan(
    const double& angle1, const double& angle2) const {
  double lateral_distance = INVALID_VALUE;
  double min = std::min(angle1, angle2);
  double max = std::max(angle1, angle2);
  for (double angle = min; angle <= max; angle += 1.0f) {
    int angle180 = round(angle);
    unsigned int angle360 = to360(angle180);
    lateral_distance = std::min(fabs(discrete_pointcloud_[angle360].second.y()),
                                lateral_distance);
  }

  return lateral_distance;
}

bool FollowWallController::loseFollowObject(const double& angle1,
                                            const double& angle2) const {
  double dir = getFollowDir();
  // double lateral_distance =
  //     GetLateralDistanceFromScanAndLaserRight(50.0 * dir, 85.0 * dir);
  double lateral_distance =
      getLateralDistanceFromScan(angle1 * dir, angle2 * dir);
  if (lateral_distance >=
      (params()->RobotRadius() + params()->FollowWallDistance() +
       params()->FollowWallTolerance()))
    return true;
  else
    return false;
}

boost::shared_ptr<Params> FollowWallController::params() const {
  return params_;
}

boost::shared_ptr<Visualization> FollowWallController::visual() const {
  return visualization_;
}

}  // namespace control
