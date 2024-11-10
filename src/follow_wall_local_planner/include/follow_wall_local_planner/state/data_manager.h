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

#ifndef DATA_MANAGER_H
#define DATA_MANAGER_H

// ROS
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <angles/angles.h>

#include <memory>

namespace follow_wall_local_planner
{
class DataManager
{
private:
	sensor_msgs::LaserScan scan_;             //sensor_link系下的scan数据
  sensor_msgs::LaserScan linear_scan_;      //sensor_link系下的lienar_scan数据
	geometry_msgs::PoseStamped global_pose_;  //map系下的pose数据
  double desired_speed_;                    //期望的沿边速度
  double follow_wall_distance_;             //沿边距离
  double robot_radius_;                     //机器人半径
  double rotation_angle_;                   //旋轉角度

public:
  DataManager();
  ~DataManager() = default;
  
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  sensor_msgs::LaserScan getScan();
  double getScan(const double angle) const;
  double getScan(double angle_min, double angle_max) const;
  double getLateralDistanceFromScan(double angle_min, double angle_max) const;

  void linearScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  double getLinearScan(const double angle) const;

  void globalPoseCallback(const geometry_msgs::PoseStamped& msg);
  geometry_msgs::PoseStamped getGlobalPose();

  void desiredSpeedCallback(const double& speed);
  double getDesiredSpeed();

  void followWallDistanceCallback(const double& dist);
  double getFollowWallDistance() const;

  void robotRadiusCallback(const double& radius);
  double getRobotRadius() const;

  void rotateAngleCallback(const double& angle);
  double getRotateAngle() const;
};
}  // namespace follow_wall_local_planner

#endif  // LANE_CHANGE_PLANNER_DATA_MANAGER_H