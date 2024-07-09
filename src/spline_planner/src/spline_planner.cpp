#include "spline_planner/spline_planner.h"

#include <memory>

#include <pluginlib/class_list_macros.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "spline_planner/dubins.h"
#include "spline_planner/reed_shepp.h"
#include "spline_planner/cubic_spline.h"

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(spline_planner::SplinePlanner,
                       nav_core::BaseGlobalPlanner)

namespace spline_planner {

SplinePlanner::SplinePlanner() : costmap_ros_(NULL), initialized_(false) {}

SplinePlanner::SplinePlanner(std::string name,
                             costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros_(NULL), initialized_(false) {
  initialize(name, costmap_ros);
}

void SplinePlanner::initialize(std::string name,
                               costmap_2d::Costmap2DROS* costmap_ros) {
  if (!initialized_) {
    name_ = name;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();

    ros::NodeHandle private_nh("~/" + name);
    private_nh.param("step_size", step_size_, costmap_->getResolution());
    private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
    world_model_ = new base_local_planner::CostmapModel(*costmap_);

    initialized_ = true;
  } else
    ROS_WARN("This planner has already been initialized... doing nothing");
}

// we need to take the footprint of the robot into account when we calculate
// cost to obstacles
double SplinePlanner::footprintCost(double x_i, double y_i, double theta_i) {
  if (!initialized_) {
    ROS_ERROR(
        "The planner has not been initialized, please call initialize() to use "
        "the planner");
    return -1.0;
  }

  std::vector<geometry_msgs::Point> footprint =
      costmap_ros_->getRobotFootprint();
  // if we have no footprint... do nothing
  if (footprint.size() < 3) return -1.0;

  // check if the footprint is legal
  double footprint_cost =
      world_model_->footprintCost(x_i, y_i, theta_i, footprint);
  return footprint_cost;
}

bool SplinePlanner::makePlan(const geometry_msgs::PoseStamped& start,
                             const geometry_msgs::PoseStamped& goal,
                             std::vector<geometry_msgs::PoseStamped>& plan) {
  ROS_WARN("This planner need to use the waypoints as input.");
  return false;
}

bool SplinePlanner::makePlan(
    const std::string& spline_type,
    const double& spline_resolution,
    const std::vector<geometry_msgs::PoseStamped>& waypoints,
    std::vector<geometry_msgs::PoseStamped>& plan) {

  if(spline_type == "dubins") {
    std::shared_ptr<Dubins> dubins = std::make_shared<Dubins>();
    if(spline_resolution > 0.0)
      plan = dubins->generate(waypoints, spline_resolution);
    else 
      plan = dubins->generate(waypoints, spline_resolution);
  } else if(spline_type == "reed_shepp") {
    std::shared_ptr<ReedShepp> rs = std::make_shared<ReedShepp>();
    if(spline_resolution > 0.0)
      plan = rs->generate(waypoints, spline_resolution);
    else
      plan = rs->generate(waypoints);
  } else if(spline_type == "cubic_spline") {
    std::shared_ptr<CubicSpline> cs = std::make_shared<CubicSpline>();
    if(spline_resolution > 0.0)
      plan = cs->generate(waypoints, spline_resolution);
    else 
      plan = cs->generate(waypoints);
  } else {
    std::cout << "未定义的样条规划器类型" << std::endl;
  }

  return (plan.empty()) ? false : true;
}

};  // namespace spline_planner
