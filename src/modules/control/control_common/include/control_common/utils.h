#pragma once

#include <angles/angles.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

#include </usr/include/eigen3/Eigen/Eigen>  //TODO:this
#include <array>
#include <vector>

namespace control {

#define INVALID_VALUE 1e6
using PointCloud = std::vector<Eigen::Vector3d>;
using DiscretePointCloud = std::array<std::pair<double, Eigen::Vector2d>, 360>;

double sign(double value);
unsigned int to360(int angle);
int to180(unsigned int angle);
double toRad(double deg);
double toDeg(double rad);
bool toBaselink(const sensor_msgs::LaserScanPtr& sensor,
                const Eigen::Isometry3d& baselink_to_sensor,
                PointCloud& points_at_base);
void discretePointcloud(const PointCloud& points_at_base,
                        DiscretePointCloud& discrete_points_at_base);

}  // namespace control
