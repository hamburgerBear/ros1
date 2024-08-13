#ifndef RPP_LOCAL_PLANNER_H
#define RPP_LOCAL_PLANNER_H

#include <base_local_planner/goal_functions.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <angles/angles.h>

//参考Nav2:https://github.com/ros-navigation/navigation2/tree/main/nav2_regulated_pure_pursuit_controller
namespace rpp_local_planner {

enum class ControlStage { IDLE = 0, START, TRACK, GOAL, SUCCESS, FAIL };

class RPPPlanner : public nav_core::BaseLocalPlanner {
 public:
  RPPPlanner();
  ~RPPPlanner();

  void initialize(std::string name, tf2_ros::Buffer* tf2,
                  costmap_2d::Costmap2DROS* costmap_ros) override;
  bool setPlan(
      const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) override;
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
  bool isGoalReached() override;

 private:
  void transitionState(const ControlStage& stage);
  void idleStage(geometry_msgs::Twist& cmd_vel);
  void startStage(geometry_msgs::Twist& cmd_vel);
  void trackStage(geometry_msgs::Twist& cmd_vel);
  void goalStage(geometry_msgs::Twist& cmd_vel);
  void successStage(geometry_msgs::Twist& cmd_vel);
  void failStage(geometry_msgs::Twist& cmd_vel);

  void setAngleBasedOnPositionDerivative(std::vector<geometry_msgs::PoseStamped>& path);
  double sign(const double& value) const;
  double calcDistance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2) const;
  unsigned int findNearstIndex(const std::vector<geometry_msgs::PoseStamped>& path) const;
  double getLookAheadDistance();
  geometry_msgs::PoseStamped getLookAheadPoint(const double &);
  static geometry_msgs::Point circleSegmentIntersection(
    const geometry_msgs::Point & p1,
    const geometry_msgs::Point & p2,
    double r);

  tf2_ros::Buffer* tf2_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  base_local_planner::OdometryHelperRos odom_helper_;
  base_local_planner::LocalPlannerUtil planner_util_;
  ros::Publisher global_plan_pub_, local_plan_pub_, lookahead_point_pub_;

  std::vector<geometry_msgs::PoseStamped> global_plan_;
  std::vector<geometry_msgs::PoseStamped> local_plan_;
  std::vector<geometry_msgs::PoseStamped> base_plan_;
  geometry_msgs::PoseStamped local_pose_;
  geometry_msgs::PoseStamped base_pose_;

  bool initialized_;
  bool goal_arrived_;
  ControlStage stage_; 
  bool stage_initialized_;  

  double xy_goal_tolerance_, yaw_goal_tolerance_;
  double min_velocity_x_, max_velocity_x_;
  double min_velocity_theta_, max_velocity_theta_;
  
  // double rotate_to_heading_angular_vel_;
  double lookahead_dist_ = 0.15;
  double max_lookahead_dist_ = 0.15;
  double min_lookahead_dist_ = 0.05;
  double lookahead_time_ = 1.0;
  bool use_interpolation_ = false; 
};                      // end of RPPPlanner
};                      // namespace rpp_local_planner
#endif

// #include <dynamic_reconfigure/server.h>
// #include <dwa_local_planner/DWAPlannerConfig.h>
// #include <nav_msgs/Odometry.h>
// #include <angles/angles.h>

/* 定义参数
  desired_linear_vel 期望速度
  lookahead_dist
  min_lookahead_dist
  max_lookahead_dist
  lookahead_time
*/
// void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);

// void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);
// #include <boost/shared_ptr.hpp>
// #include <boost/thread.hpp>

// #include <base_local_planner/latched_stop_rotate_controller.h>

// #include <dwa_local_planner/dwa_planner.h>