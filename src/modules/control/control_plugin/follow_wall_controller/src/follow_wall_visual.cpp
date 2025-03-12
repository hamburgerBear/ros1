#pragma once

#include "follow_wall_controller/follow_wall_visual.h"

namespace control {

Visualization::Visualization(ros::NodeHandle& nh) : nh_(nh) {
  pub_pointcloud_ = nh_.advertise<visualization_msgs::Marker>("/pointcloud", 1);
  pub_discrete_pointcloud_ =
      nh_.advertise<visualization_msgs::Marker>("/discrete_pointcloud", 1);
}

void Visualization::publishPointCloud(const PointCloud& pointcloud) {
  visualization_msgs::Marker point_marker;
  point_marker.header.frame_id = "base_link";
  point_marker.header.stamp = ros::Time::now();
  point_marker.ns = "scan";
  point_marker.id = 0;
  point_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  point_marker.action = visualization_msgs::Marker::ADD;
  point_marker.scale.x = 0.05;  // TODO:AUTOWARE.AI使用SCALE和COLOR
  point_marker.scale.y = 0.05;
  point_marker.scale.z = 0.05;
  point_marker.color.r = 1.0f;
  point_marker.color.g = 0.0f;
  point_marker.color.b = 0.0f;
  point_marker.color.a = 1.0f;
  geometry_msgs::Point p;
  for (const auto& point : pointcloud) {
    p.x = point.x();
    p.y = point.y();
    point_marker.points.push_back(p);
  }

  pub_pointcloud_.publish(point_marker);
}

void Visualization::publishPointDiscretePointCloud(
    const DiscretePointCloud& discrete_pointcloud) {
  visualization_msgs::Marker point_marker;
  point_marker.header.frame_id = "base_link";
  point_marker.header.stamp = ros::Time::now();
  point_marker.ns = "scan";
  point_marker.id = 0;
  point_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  point_marker.action = visualization_msgs::Marker::ADD;
  point_marker.scale.x = 0.05;  // TODO:AUTOWARE.AI使用SCALE和COLOR
  point_marker.scale.y = 0.05;
  point_marker.scale.z = 0.05;
  point_marker.color.r = 0.0f;
  point_marker.color.g = 1.0f;
  point_marker.color.b = 0.0f;
  point_marker.color.a = 1.0f;
  geometry_msgs::Point p;
  for (const auto& point : discrete_pointcloud) {
    p.x = point.second.x();
    p.y = point.second.y();
    point_marker.points.push_back(p);
  }

  pub_discrete_pointcloud_.publish(point_marker);
}

}  // namespace control