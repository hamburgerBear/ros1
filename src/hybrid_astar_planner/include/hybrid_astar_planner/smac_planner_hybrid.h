/*********************************************************************
 * Author: Gaojie
 *********************************************************************/
#ifndef SPLINE_PLANNER_H
#define SPLINE_PLANNER_H

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_global_planner.h>
#include <ros/ros.h>

#include <angles/angles.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <base_local_planner/costmap_model.h>
#include <base_local_planner/world_model.h>
#include "hybrid_astar_planner/a_star.h"
// #include "hybrid_astar_planner/collision_checker.h"

namespace hybrid_astar_planner {

class SmacPlannerHybrid : public nav_core::BaseGlobalPlanner {
 public:
  /**
   * @brief  Constructor for the SmacPlannerHybrid
   */
  SmacPlannerHybrid();
  /**
   * @brief  Constructor for the SmacPlannerHybrid
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for
   * planning
   */
  SmacPlannerHybrid(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief  Initialization function for the SmacPlannerHybrid
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for
   * planning
   */
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Given a goal pose in the world, compute a plan
   * @param start The start pose
   * @param goal The goal pose
   * @param plan The plan... filled by the planner
   * @return True if a valid plan was found, false otherwise
   */
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);

 private:
  costmap_2d::Costmap2DROS* costmap_ros_;
  double step_size_, min_dist_from_robot_;
  costmap_2d::Costmap2D* costmap_;
  base_local_planner::WorldModel*
      world_model_;  ///< @brief The world model that the controller will use

  /**
   * @brief  Checks the legality of the robot footprint at a position and
   * orientation using the world model
   * @param x_i The x position of the robot
   * @param y_i The y position of the robot
   * @param theta_i The orientation of the robot
   * @return
   */
  double footprintCost(double x_i, double y_i, double theta_i);

  bool initialized_;

  std::unique_ptr<AStarAlgorithm<NodeHybrid>> _a_star;
  GridCollisionChecker _collision_checker;
  //   std::unique_ptr<Smoother> _smoother;
  //   rclcpp::Clock::SharedPtr _clock;
  //   rclcpp::Logger _logger{rclcpp::get_logger("SmacPlannerHybrid")};
  // costmap_2d::Costmap2D* _costmap;
  std::shared_ptr<costmap_2d::Costmap2DROS> _costmap_ros;
  //   std::unique_ptr<CostmapDownsampler> _costmap_downsampler;  // todo:this
  std::string _global_frame, _name;
  float _lookup_table_dim;
  float _tolerance;
  bool _downsample_costmap;
  int _downsampling_factor;
  double _angle_bin_size;
  unsigned int _angle_quantizations;
  bool _allow_unknown;
  int _max_iterations;
  SearchInfo _search_info;
  double _max_planning_time;
  double _lookup_table_size;
  double _minimum_turning_radius_global_coords;
  std::string _motion_model_for_search;
  MotionModel _motion_model;

  ros::Publisher pose_array_pub_;
  //   rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr
  //       _raw_plan_publisher;
  // std::mutex _mutex;
};
};  // namespace hybrid_astar_planner

#endif
