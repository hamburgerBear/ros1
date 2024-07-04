#include "move_base/visualization.h"

namespace move_base {

Visualization* Visualization::instance_ = nullptr;
std::mutex Visualization::mtx_;

Visualization* Visualization::getInstance() {
  if (instance_ == nullptr) {
    std::unique_lock<std::mutex> lock(mtx_);
    if (instance_ == nullptr) {
      volatile auto temp = new (std::nothrow) Visualization();
      instance_ = temp;
    }
  }

  return instance_;
}

void Visualization::deleteInstance() {
  std::unique_lock<std::mutex> lock(mtx_);
  if (instance_) {
    delete instance_;
    instance_ = nullptr;
  }
}

Visualization::Visualization() : initialized_(false) {}

Visualization::~Visualization() {}

void Visualization::initialize(ros::NodeHandle nh) {
  initialized_ = true;
  start_pub_ = nh.advertise<visualization_msgs::Marker>("visual/start", 1);
  goal_pub_ = nh.advertise<visualization_msgs::Marker>("visual/goal", 1);
  waypoints_pub_ = nh.advertise<visualization_msgs::MarkerArray>("visual/waypoints", 1);
  global_plan_pub_ = nh.advertise<nav_msgs::Path>("visual/global_plan", 1);
}

void Visualization::publishStart(const geometry_msgs::PoseStamped& start) {
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "map";
  // marker.ns = "ns"; //marker_name_space;
  // marker.id = 1;//marker_id;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = start.pose;
  marker.scale.x = 0.1;
  marker.scale.y = 0.03;
  marker.scale.z = 0.0;
  marker.color.r = 0.0f;
  marker.color.g = 0.988235294f;
  marker.color.b = 0.0f;
  marker.color.a = 0.8;
  // marker.lifetime = ros::Duration(1.0);
  start_pub_.publish(marker);
}

void Visualization::publishGoal(const geometry_msgs::PoseStamped& goal) {
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "map";
  // marker.ns = "ns"; //marker_name_space;
  // marker.id = 1;//marker_id;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = goal.pose;
  marker.scale.x = 0.1;
  marker.scale.y = 0.03;
  marker.scale.z = 0.0;
  marker.color.r = 0.992156863f;
  marker.color.g = 0.0f;
  marker.color.b = 0.992156863f;
  marker.color.a = 0.8;
  // marker.lifetime = ros::Duration(1.0);
  goal_pub_.publish(marker);
}

void Visualization::publishWaypoints(
    const std::vector<geometry_msgs::PoseStamped>& waypoints) {
  visualization_msgs::Marker mark;
  visualization_msgs::MarkerArray marker_array;
  unsigned int i = 0;
  for(auto waypoint : waypoints) {
    mark.header.frame_id = "map";
    mark.header.stamp = ros::Time::now();
    // marker.ns = "ns"; //marker_name_space;
    mark.id = i++;  // marker_id;
    mark.type = visualization_msgs::Marker::SPHERE;
    mark.action = visualization_msgs::Marker::ADD;
    mark.scale.x = 0.1;
    mark.scale.y = 0.1;
    mark.scale.z = 0.1;
    mark.color.r = 0.5;
    mark.color.g = 0.5;
    mark.color.b = 0.5;
    mark.color.a = 0.5;
    mark.pose = waypoint.pose;
    marker_array.markers.push_back(mark);
  }
  waypoints_pub_.publish(marker_array);
}

void Visualization::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
  nav_msgs::Path gui_path;
  gui_path.poses.resize(path.size());
  gui_path.header.frame_id = "map";
  gui_path.header.stamp = ros::Time::now();
  for (unsigned int i = 0; i < path.size(); i++) {
    gui_path.poses[i] = path[i];
  }

  global_plan_pub_.publish(gui_path);
}

}  // namespace move_base
