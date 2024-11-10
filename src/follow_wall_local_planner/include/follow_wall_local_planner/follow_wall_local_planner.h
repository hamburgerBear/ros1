#ifndef FW_LOCAL_PLANNER_H
#define FW_LOCAL_PLANNER_H

#include <base_local_planner/goal_functions.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <tf2_ros/buffer.h>
#include <follow_wall_local_planner/state/state_machine.h>

namespace follow_wall_local_planner {

class FollowWallLocalPlanner : public nav_core::BaseLocalPlanner {
 public:
  FollowWallLocalPlanner();
  ~FollowWallLocalPlanner();

  void initialize(std::string name, tf2_ros::Buffer* tf2,
                  costmap_2d::Costmap2DROS* costmap_ros) override;
  bool setPlan( const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) override;
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
  bool isGoalReached() override;

 private:
  tf2_ros::Buffer* tf2_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  base_local_planner::OdometryHelperRos odom_helper_;
  base_local_planner::LocalPlannerUtil planner_util_;
  ros::Publisher global_plan_pub_, local_plan_pub_, lookpoint_pub_;
  ros::Subscriber laser_sub_, linear_laser_sub_;

  std::vector<geometry_msgs::PoseStamped> global_plan_;
  std::vector<geometry_msgs::PoseStamped> local_plan_;
  std::vector<geometry_msgs::PoseStamped> base_plan_;

  bool initialized_;
  std::shared_ptr<DataManager> data_manager_ptr_;
  std::shared_ptr<StateMachine> state_machine_ptr_;
};                      // end of FollowWallLocalPlanner
};                      // namespace follow_wall_local_planner
#endif