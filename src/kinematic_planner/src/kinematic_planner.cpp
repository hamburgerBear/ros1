#include "kinematic_planner/kinematic_planner.h"

#include <chrono>
#include <memory>

#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/MarkerArray.h>

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(kinematic_planner::KinematicPlanner,
                       nav_core::BaseGlobalPlanner)

using namespace std::chrono;
namespace kinematic_planner {

KinematicPlanner::KinematicPlanner()
    : costmap_ros_(NULL), initialized_(false) {}

KinematicPlanner::KinematicPlanner(std::string name,
                                   costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros_(NULL), initialized_(false) {
  initialize(name, costmap_ros);
}

void KinematicPlanner::initialize(std::string name,
                                  costmap_2d::Costmap2DROS* costmap_ros) {
  if (!initialized_) {
    name_ = name;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();

    ros::NodeHandle private_nh("~/" + name);
    private_nh.param("step_size", step_size_, costmap_->getResolution());
    private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("kinematic_plan", 1);
    pose_array_pub_ =
        private_nh.advertise<geometry_msgs::PoseArray>("expansions", 1);
    footprints_pub_ =
        private_nh.advertise<visualization_msgs::MarkerArray>("footprints", 1);
    planner_core_ = std::make_shared<kinematicPlannerCore>(costmap_ros_);
    planner_core_->initialize();
    ROS_WARN("This planner initialized");
  } else
    ROS_WARN("This planner has already been initialized... doing nothing");
}

bool KinematicPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                const geometry_msgs::PoseStamped& goal,
                                std::vector<geometry_msgs::PoseStamped>& plan) {
  std::vector<Eigen::Vector3f> expands;
  steady_clock::time_point a = steady_clock::now();
  bool ret = planner_core_->makePlan(start, goal, plan, expands);
  steady_clock::time_point b = steady_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(b - a);
  ROS_WARN("Planning cost{%f}", static_cast<double>(time_span.count()));

  nav_msgs::Path gui_path;
  gui_path.poses.resize(plan.size());
  gui_path.header.frame_id = goal.header.frame_id;
  gui_path.poses = plan;
  plan_pub_.publish(gui_path);

  auto getWorldOrientation =
      [=](const float& theta) -> geometry_msgs::Quaternion {
    tf2::Quaternion q;
    q.setEuler(0.0, 0.0, theta);
    return tf2::toMsg(q);
  };
  geometry_msgs::PoseArray msg;
  geometry_msgs::Pose msg_pose;
  msg.header.frame_id = "map";
  for (auto& e : expands) {
    msg_pose.position.x = e.x();
    msg_pose.position.y = e.y();
    msg_pose.orientation = getWorldOrientation(e.z());
    msg.poses.push_back(msg_pose);
  }
  pose_array_pub_.publish(msg);

  {
    visualization_msgs::Marker pathVehicle;
    visualization_msgs::MarkerArray pathVehicles;
    int i = 0;
    for (auto point : plan) {
      pathVehicle.header.frame_id = "map";
      pathVehicle.header.stamp = ros::Time::now();
      // marker.ns = "ns"; //marker_name_space;
      pathVehicle.id = i++;  // marker_id;
      pathVehicle.type = visualization_msgs::Marker::CUBE;
      pathVehicle.action = visualization_msgs::Marker::ADD;
      pathVehicle.scale.x = 0.8;
      pathVehicle.scale.y = 0.6;
      pathVehicle.scale.z = 0.0;
      pathVehicle.color.r = 1.0;
      pathVehicle.color.g = 0.0;
      pathVehicle.color.b = 0.0;
      pathVehicle.color.a = 0.05;

      pathVehicle.pose = point.pose;
      // pathVehicle.pose.position.x = point.x();
      // pathVehicle.pose.position.y = point.y();
      // tf2::Quaternion quaternion;
      // quaternion.setRPY(0.0, 0.0, point.z());
      // pathVehicle.pose.orientation = tf2::toMsg(quaternion);
      pathVehicles.markers.push_back(pathVehicle);
    }
    footprints_pub_.publish(pathVehicles);
  }

  return ret;
}

};  // namespace kinematic_planner
