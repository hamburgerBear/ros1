#include "follow_wall_controller/follow_wall_controller.h"

#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/Marker.h>

#include "follow_wall_controller/state_root.h"

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

  //插件初始化中获得ROS参数，
  //单独增加一个setGoal的函数在插件中
  //创建发布器
  ros::NodeHandle nh;
  pub_pointcloud_ = nh.advertise<visualization_msgs::Marker>("/pointcloud", 1);
  pub_discrete_pointcloud_ =
      nh.advertise<visualization_msgs::Marker>("/discrete_pointcloud", 1);

  base_to_laser_ = Eigen::Isometry3d::Identity();
  return true;
}

PluginStage FollowWallController::run() {
  ROS_INFO("RUNNING....");

  std::cout << "沿墙运行中" << std::endl;
  bool ok = toBaselink(injector_->scan_, base_to_laser_, pointcloud_);
  discretePointcloud(pointcloud_, discrete_pointcloud_);
  publishPointCloud();
  publishPointDiscretePointCloud();

  static int i = 0;
  PluginStage stage;
  i++;
  if (i < 1000) {
    stage.stage = PluginStage::Stage::RUNNING;
  } else {
    stage.stage = PluginStage::Stage::SUCCEEDED;
  }
  state_machine_->ProcessStateTransitions();
  state_machine_->UpdateStates();
  sleep(1);  // TODO:check this
  return stage;
}

double FollowWallController::getLateralDistanceFromScan(const double& angle1,
                                                        const double& angle2) {
  double lateral_distance = INVALID_VALUE;
  double min = std::min(angle1, angle2);
  double max = std::max(angle1, angle2);
  for (double angle = min; angle <= max; angle += 1.0f) {
    int angle180 = round(angle);
    unsigned int angle360 = to360(angle180);
    lateral_distance =
        std::min(fabs(discrete_pointcloud_[angle360].first), lateral_distance);
  }

  return lateral_distance;
}

void FollowWallController::publishPointCloud() {
  visualization_msgs::Marker point_marker;
  point_marker.header.frame_id = "base_link";
  point_marker.header.stamp = ros::Time::now();
  point_marker.ns = "scan";
  point_marker.id = 0;
  point_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  point_marker.action = visualization_msgs::Marker::ADD;
  point_marker.scale.x = 0.05;  // TODO:AUTOWARE.AI使用SCALE和COLOR
  point_marker.scale.y = 0.05;
  point_marker.scale.z = 0.05;
  point_marker.color.r = 1.0f;
  point_marker.color.g = 0.0f;
  point_marker.color.b = 0.0f;
  point_marker.color.a = 1.0f;
  geometry_msgs::Point p;
  for (const auto& point : pointcloud_) {
    p.x = point.x();
    p.y = point.y();
    point_marker.points.push_back(p);
  }

  pub_pointcloud_.publish(point_marker);
}

void FollowWallController::publishPointDiscretePointCloud() {
  visualization_msgs::Marker point_marker;
  point_marker.header.frame_id = "base_link";
  point_marker.header.stamp = ros::Time::now();
  point_marker.ns = "scan";
  point_marker.id = 0;
  point_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  point_marker.action = visualization_msgs::Marker::ADD;
  point_marker.scale.x = 0.05;  // TODO:AUTOWARE.AI使用SCALE和COLOR
  point_marker.scale.y = 0.05;
  point_marker.scale.z = 0.05;
  point_marker.color.r = 0.0f;
  point_marker.color.g = 1.0f;
  point_marker.color.b = 0.0f;
  point_marker.color.a = 1.0f;
  geometry_msgs::Point p;
  for (const auto& point : discrete_pointcloud_) {
    p.x = point.second.x();
    p.y = point.second.y();
    point_marker.points.push_back(p);
  }

  pub_discrete_pointcloud_.publish(point_marker);
}

}  // namespace control
