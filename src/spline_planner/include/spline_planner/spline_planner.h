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

#include <base_local_planner/costmap_model.h>
#include <base_local_planner/world_model.h>

namespace spline_planner {

enum SplineType {
  NONE = 0,
  LINE,
  ARC,
  ROUND,
  CUBIC_BEZIER,
  QUINTIC_BEZIER,
  CUBIC_SPLINE,
  QUINTIC_SPLINE,
  DUBINS,
  REED_SHEEP
};

/**
 * @class SplinePlanner
 * @brief 根据起终点或waypoints生成一段样条轨迹。其中样条规划器的类型包含了
 * 直线、圆弧、直角圆弧、三阶贝塞尔、五阶贝塞尔、三次样条、五次样条、Dubins曲线、RS曲线等。
 */
class SplinePlanner : public nav_core::BaseGlobalPlanner {
 public:
  /**
   * @brief  Constructor for the SplinePlanner
   */
  SplinePlanner();
  /**
   * @brief  Constructor for the SplinePlanner
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for
   * planning
   */
  SplinePlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief  Initialization function for the SplinePlanner
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

  /**
   * @brief Given waypoints in the world, compute a plan
   * @param spline_type
   * @param waypoints
   * @param plan The plan... filled by the planner
   * @return True if a valid plan was found, false otherwise
   */
  bool makePlan(const std::string& spline_type,
                const double& spline_resolution,
                const std::vector<geometry_msgs::PoseStamped>& waypoints,
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
};
};  // namespace spline_planner

#endif
