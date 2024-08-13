/*********************************************************************
 * Author: Gaojie
 *********************************************************************/
#ifndef KINEMATIC_PLANNER_H
#define KINEMATIC_PLANNER_H

#include <base_local_planner/costmap_model.h>
#include <base_local_planner/world_model.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_global_planner.h>
#include <ros/ros.h>
#include "kinematic_planner/kinematic_planner_core.h"

namespace kinematic_planner {

class KinematicPlanner : public nav_core::BaseGlobalPlanner {
 public:
  /**
   * @brief  Constructor for the KinematicPlanner
   */
  KinematicPlanner();
  /**
   * @brief  Constructor for the KinematicPlanner
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for
   * planning
   */
  KinematicPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief  Initialization function for the KinematicPlanner
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
  costmap_2d::Costmap2D* costmap_;
  double step_size_, min_dist_from_robot_;
  base_local_planner::WorldModel*
      world_model_;  ///< @brief The world model that the controller will use
  bool initialized_;
  std::shared_ptr<kinematicPlannerCore> planner_core_;

  ros::Publisher plan_pub_, pose_array_pub_, footprints_pub_;
};
};  // namespace kinematic_planner

#endif
