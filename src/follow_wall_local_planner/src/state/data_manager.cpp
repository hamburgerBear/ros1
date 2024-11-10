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

#include <follow_wall_local_planner/state/data_manager.h>

namespace follow_wall_local_planner
{
DataManager::DataManager() {}

void DataManager::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  scan_ = *msg;
}

sensor_msgs::LaserScan DataManager::getScan() {
  return scan_;
}

double DataManager::getScan(const double angle) const {
  double range = -1.0;
  if(angle >= scan_.angle_min && angle <= scan_.angle_max) {
    double diff = angles::normalize_angle(angle - scan_.angle_min);
    int n = fabs(diff) / scan_.angle_increment; 
    range = scan_.ranges[n];
  }
  return range;  
}

double DataManager::getScan(double angle_min, double angle_max) const {
  double range = -1.0;
  double angle_min_tmp = std::min(angle_min, angle_max);
  double angle_max_tmp = std::max(angle_min, angle_max);
  angle_min = angle_min_tmp;
  angle_max = angle_max_tmp;

  if(angle_min >= scan_.angle_min && angle_min <= scan_.angle_max
  && angle_max >= scan_.angle_min && angle_max <= scan_.angle_max) {

    int n_min = fabs(angles::normalize_angle(angle_min - scan_.angle_min)) / scan_.angle_increment; 
    int n_max = fabs(angles::normalize_angle(angle_max - scan_.angle_min)) / scan_.angle_increment; 

    range = 1e3;
    for(int i = n_min; i <= n_max; i++) 
      range = std::min(range, (double)scan_.ranges[i]);
  }
  return range;
}

double DataManager::getLateralDistanceFromScan(double angle_min, double angle_max) const {
  double range = -1.0;
  double angle_min_tmp = std::min(angle_min, angle_max);
  double angle_max_tmp = std::max(angle_min, angle_max);
  angle_min = angle_min_tmp;
  angle_max = angle_max_tmp;

  if(angle_min >= scan_.angle_min && angle_min <= scan_.angle_max
  && angle_max >= scan_.angle_min && angle_max <= scan_.angle_max) {

    int n_min = fabs(angles::normalize_angle(angle_min - scan_.angle_min)) / scan_.angle_increment; 
    int n_max = fabs(angles::normalize_angle(angle_max - scan_.angle_min)) / scan_.angle_increment; 

    range = 1e3;
    for(int i = n_min; i <= n_max; i++) {
      double angle = scan_.angle_min + i * scan_.angle_increment;
      range = std::min(fabs(sin(angle) * scan_.ranges[i]), range);
    }
  }
  return range;
}

void DataManager::linearScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  linear_scan_ = *msg;
}

double DataManager::getLinearScan(const double angle) const {
  double range = -1.0;
  if(angle >= linear_scan_.angle_min && angle <= linear_scan_.angle_max) {
    double diff = angles::normalize_angle(angle - linear_scan_.angle_min);
    int n = fabs(diff) / linear_scan_.angle_increment; 
    range = linear_scan_.ranges[n];
    range = fabs(sin(angle) * range);
  }
  return range;  
}

void DataManager::globalPoseCallback(const geometry_msgs::PoseStamped& msg) {
  global_pose_ = msg;
}

geometry_msgs::PoseStamped DataManager::getGlobalPose() {
  return global_pose_;
}

void DataManager::desiredSpeedCallback(const double& speed) {
  desired_speed_ = speed;
}

double DataManager::getDesiredSpeed() {
  return desired_speed_;
}

void DataManager::followWallDistanceCallback(const double& dist) {
  follow_wall_distance_ = dist;
}

double DataManager::getFollowWallDistance() const {
  return follow_wall_distance_;
}

void DataManager::robotRadiusCallback(const double& radius) {
  robot_radius_ = radius;
}

double DataManager::getRobotRadius() const {
  return robot_radius_;
}

void DataManager::rotateAngleCallback(const double& angle) {
  rotation_angle_ = angle;
}

double DataManager::getRotateAngle() const {
  return rotation_angle_;
}

}  // namespace follow_wall_local_planner
