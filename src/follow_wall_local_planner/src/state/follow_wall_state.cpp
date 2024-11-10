/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <follow_wall_local_planner/state/follow_wall_state.h>
#include <follow_wall_local_planner/state/data_manager.h>
// #include <lane_change_planner/route_handler.h>
// #include <lane_change_planner/state/common_functions.h>
// #include <lane_change_planner/state/following_lane.h>
// #include <lane_change_planner/utilities.h>
// #include <lanelet2_extension/utility/message_conversion.h>
// #include <lanelet2_extension/utility/utilities.h>

namespace follow_wall_local_planner
{
FollowWallState::FollowWallState(
  const Status & status, const std::shared_ptr<DataManager> & data_manager_ptr/*,
  const std::shared_ptr<RouteHandler> & route_handler_ptr*/)
: StateBase(status, data_manager_ptr/*, route_handler_ptr*/)
{
}

State FollowWallState::getCurrentState() const { return State::FOLLOW_WALL; }

void FollowWallState::entry()
{
  ROS_INFO("FollowWallState::entry");
  // ros_parameters_ = data_manager_ptr_->getLaneChangerParameters();
  // lane_change_approved_ = false;
  // force_lane_change_ = false;
  // status_.lane_change_available = false;
  // status_.lane_change_ready = false;
}

// autoware_planning_msgs::PathWithLaneId FollowWallState::getPath() const
// {
//   return status_.lane_follow_path;
// }

void FollowWallState::update(geometry_msgs::Twist& cmd_vel)
{
  double section_1 = data_manager_ptr_->getLateralDistanceFromScan(angles::from_degrees(-50.0), angles::from_degrees(-60.0));
  double section_2 = data_manager_ptr_->getLateralDistanceFromScan(angles::from_degrees(-60.0), angles::from_degrees(-70.0));
  double section_3 = data_manager_ptr_->getLateralDistanceFromScan(angles::from_degrees(-70.0), angles::from_degrees(-80.0));
  double section_4 = data_manager_ptr_->getLateralDistanceFromScan(angles::from_degrees(-70.0), angles::from_degrees(-85.0));
  double line_laser_range = data_manager_ptr_->getLinearScan(angles::from_degrees(-75.0));

  double desired_distance = data_manager_ptr_->getRobotRadius() + data_manager_ptr_->getFollowWallDistance();
  double lateral_distance;
  if(section_1 < desired_distance + 0.05)
    lateral_distance = section_1;
  else if(section_2 < desired_distance + 0.05)
    lateral_distance = section_2;
  else if(section_3 < desired_distance + 0.05)
    lateral_distance = section_3;
  else {
    bool use_line_laser = true;
    if(use_line_laser) 
      lateral_distance = std::min(section_4, line_laser_range);
    else 
      lateral_distance = section_4;
  }

  // double linear_scan = data_manager_ptr_->getLinearScan(angles::from_degrees(-85.0)); 
  // linear_scan = sin(angles::from_degrees(85.0)) * linear_scan;
  // ROS_WARN("linear scan : %f", linear_scan);
  ROS_INFO("lateral_distance: %f  desired_distance = %f", lateral_distance, desired_distance);
  double kp = 8.0;
  double w = (desired_distance - lateral_distance) * kp;
  cmd_vel.angular.z = w;
 
  cmd_vel.linear.x = data_manager_ptr_->getDesiredSpeed();
  // cmd_vel.linear.x = std::pow(-3.75, 2) * fabs(cmd_vel.angular.z) + 2.83 * fabs(cmd_vel.angular.z) + 0.39;
  if(fabs(cmd_vel.angular.z) >= 0.4)
    cmd_vel.linear.x *= 0.5;
  else if(fabs(cmd_vel.angular.z) >= 0.3)
    cmd_vel.linear.x *= 0.65;
  else if(fabs(cmd_vel.angular.z) >= 0.2)
    cmd_vel.linear.x *= 0.8;
  else if(fabs(cmd_vel.angular.z) >= 0.1)
    cmd_vel.linear.x *= 0.9;
  ROS_INFO("x: %f  y = %f", cmd_vel.linear.x, cmd_vel.angular.z);
  //Debug 
  // cmd_vel.linear.x  = 0.0;
  // cmd_vel.angular.z = 0.0;
}

State FollowWallState::getNextState() const
{
  if(isLongitudinalApproach()) {
    return State::ROTATE_TO_WALL;
  } else if(isLateralDeviation()) {
    return State::ARC_TO_WALL;
  } else 
    return State::FOLLOW_WALL;
}

bool FollowWallState::isLongitudinalApproach() const {
  //debug 
  return false;

  double section_left = data_manager_ptr_->getScan(angles::from_degrees(25.0), angles::from_degrees(45.0));
  double section_middle = data_manager_ptr_->getScan(angles::from_degrees(-25.0), angles::from_degrees(25.0));
  double section_right = data_manager_ptr_->getScan(angles::from_degrees(-45.0), angles::from_degrees(-25.0));

  bool is_approach = false;
  if(section_right < data_manager_ptr_->getRobotRadius() + data_manager_ptr_->getFollowWallDistance() + 0.05) {
    data_manager_ptr_->rotateAngleCallback(angles::from_degrees(20.0));
    is_approach = true;
  }
  else if(section_middle < data_manager_ptr_->getRobotRadius() + data_manager_ptr_->getFollowWallDistance() + 0.05) {
    data_manager_ptr_->rotateAngleCallback(angles::from_degrees(60.0));
    is_approach = true;
  } else if(section_left < data_manager_ptr_->getRobotRadius() + data_manager_ptr_->getFollowWallDistance() + 0.05)  {
    data_manager_ptr_->rotateAngleCallback(angles::from_degrees(80.0));
    is_approach = true;
  } else 
    is_approach = false;

  return is_approach;
}

bool FollowWallState::isLateralDeviation() const {
  double line_laser_range = data_manager_ptr_->getLinearScan(angles::from_degrees(-75.0));
  double lateral_distance = data_manager_ptr_->getLateralDistanceFromScan(angles::from_degrees(-50.0), angles::from_degrees(-85.0));
  lateral_distance = std::min(lateral_distance, line_laser_range);
  //if only line laser, range improve
  if(lateral_distance >= (data_manager_ptr_->getRobotRadius() + data_manager_ptr_->getFollowWallDistance() + 0.08))  //0.05->0.08
    return true;
  else 
    return false;
}

}  // namespace follow_wall_local_planner

