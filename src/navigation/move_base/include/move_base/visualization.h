#ifndef NAV_MOVE_BASE_VISUALIZATION_H
#define NAV_MOVE_BASE_VISUALIZATION_H

#include <mutex>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>

namespace move_base {

class Visualization {
 public:
  static Visualization *getInstance();
  static void deleteInstance();

 private:
  Visualization();
  ~Visualization();
  Visualization(const Visualization &signal);
  const Visualization &operator=(const Visualization &signal);

  static Visualization *instance_;
  static std::mutex mtx_;

 public:
  void initialize(ros::NodeHandle nh);
  void publishStart(const geometry_msgs::PoseStamped& start);
  void publishGoal(const geometry_msgs::PoseStamped& goal);
  void publishWaypoints(const std::vector<geometry_msgs::PoseStamped>& waypoints);
  void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

 private:
  bool initialized_;
  ros::NodeHandle nh_;
  ros::Publisher start_pub_, goal_pub_, waypoints_pub_;
  ros::Publisher global_plan_pub_;
};
}  // namespace move_base

#endif
