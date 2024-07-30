#include "hybrid_astar_planner/smac_planner_hybrid.h"

#include <memory>

#include <pluginlib/class_list_macros.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(hybrid_astar_planner::SmacPlannerHybrid,
                       nav_core::BaseGlobalPlanner)

namespace hybrid_astar_planner {

SmacPlannerHybrid::SmacPlannerHybrid() : costmap_ros_(NULL), initialized_(false), 
  _collision_checker(nullptr, 1) {}

SmacPlannerHybrid::SmacPlannerHybrid(std::string name,
                                 costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros_(NULL), initialized_(false),
      _collision_checker(nullptr, 1) {
  initialize(name, costmap_ros);
}

void SmacPlannerHybrid::initialize(std::string name,
                                   costmap_2d::Costmap2DROS* costmap_ros) {
  if (!initialized_) {
    name_ = name;
    costmap_ros_ = costmap_ros;
    _costmap_ros.reset(costmap_ros);
    costmap_ = costmap_ros_->getCostmap();

    ros::NodeHandle private_nh("~/" + name);
    private_nh.param("step_size", step_size_, costmap_->getResolution());
    private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
    world_model_ = new base_local_planner::CostmapModel(*costmap_);

    pose_array_pub_ = private_nh.advertise<geometry_msgs::PoseArray>("expansions", 1);
   
    // ===== 初始化hybrid astar参数
    // _costmap = costmap_ros;
    _downsample_costmap = false;
    _downsampling_factor = 1;
    initialized_ = true;
    int angle_quantizations = 72;
    _angle_bin_size = 2.0 * M_PI / angle_quantizations;
    _angle_quantizations = angle_quantizations;
    _allow_unknown = false;
    _max_iterations = 10000000;   //1000000
    bool smooth_path = false;
    _minimum_turning_radius_global_coords = 0.8;
    _search_info.cache_obstacle_heuristic = false;
    _search_info.reverse_penalty = 2.0;
    _search_info.change_penalty = 0.0;
    _search_info.non_straight_penalty = 1.2;
    _search_info.cost_penalty = 2.0;
    _search_info.retrospective_penalty = 0.015;
    _search_info.analytic_expansion_ratio = 3.5;
    double analytic_expansion_max_length_m = 3.0;
    _search_info.analytic_expansion_max_length =
        analytic_expansion_max_length_m / costmap_->getResolution();
    _max_planning_time = 50000.0;
    _lookup_table_size = 20.0;
    _motion_model_for_search = "DUBIN";
    _motion_model = fromString(_motion_model_for_search);
    _search_info.minimum_turning_radius =
        _minimum_turning_radius_global_coords /
        (costmap_->getResolution() * _downsampling_factor);

    _lookup_table_dim =
        static_cast<float>(_lookup_table_size) /
        static_cast<float>(costmap_->getResolution() * _downsampling_factor);
    _lookup_table_dim = static_cast<float>(static_cast<int>(_lookup_table_dim));
    if(static_cast<int>(_lookup_table_dim) % 2 == 0) _lookup_table_dim += 1.0;

    _collision_checker = GridCollisionChecker(_costmap_ros, _angle_quantizations);
    _collision_checker.setFootprint(_costmap_ros->getRobotFootprint(), false, 100.0);
    _a_star =
        std::make_unique<AStarAlgorithm<NodeHybrid>>(_motion_model, _search_info);
    _a_star->initialize(_allow_unknown,    
                        _max_iterations,   
                        std::numeric_limits<int>::max(),
                        10,   
                        _max_planning_time,                
                        _lookup_table_dim,                
                        _angle_quantizations); 
  } else
    ROS_WARN("This planner has already been initialized... doing nothing");
}

double SmacPlannerHybrid::footprintCost(double x_i, double y_i, double theta_i) {
  if (!initialized_) {
    ROS_ERROR(
        "The planner has not been initialized, please call initialize() to use "
        "the planner");
    return -1.0;
  }

  std::vector<geometry_msgs::Point> footprint =
      costmap_ros_->getRobotFootprint();
  // if we have no footprint... do nothing
  if (footprint.size() < 3) return -1.0;

  // check if the footprint is legal
  double footprint_cost =
      world_model_->footprintCost(x_i, y_i, theta_i, footprint);
  return footprint_cost;
}

bool SmacPlannerHybrid::makePlan(const geometry_msgs::PoseStamped& start,
                             const geometry_msgs::PoseStamped& goal,
                             std::vector<geometry_msgs::PoseStamped>& plan) {
    // Downsample costmap, if required
  // nav2_costmap_2d::Costmap2D * costmap = _costmap;
  // if (_costmap_downsampler) {
  //   costmap = _costmap_downsampler->downsample(_downsampling_factor);
  //   _collision_checker.setCostmap(costmap);
  // }
  std::cout << "开始规划" << std::endl;
  // Set collision checker and costmap information
  _collision_checker.setFootprint(
    _costmap_ros->getRobotFootprint(),
    /*_costmap_ros->getUseRadius(),*/ false,
    /*findCircumscribedCost(_costmap_ros) 使用代价地图的膨胀层计算外切半径处的代价*/ 100);
  _a_star->setCollisionChecker(&_collision_checker);
  std::cout << "设置碰撞检测器" << std::endl;
  // Set starting point, in A* bin search coordinates
  float mx, my;
  if (!costmap_->worldToMapContinuous(start.pose.position.x, start.pose.position.y, mx, my)) {
    // throw nav2_core::StartOutsideMapBounds(
    //         "Start Coordinates of(" + std::to_string(start.pose.position.x) + ", " +
    //         std::to_string(start.pose.position.y) + ") was outside bounds");
  }

  double orientation_bin = tf2::getYaw(start.pose.orientation) / _angle_bin_size;
  while (orientation_bin < 0.0) {
    orientation_bin += static_cast<float>(_angle_quantizations);
  }
  // This is needed to handle precision issues
  if (orientation_bin >= static_cast<float>(_angle_quantizations)) {
    orientation_bin -= static_cast<float>(_angle_quantizations);
  }
  unsigned int orientation_bin_id = static_cast<unsigned int>(floor(orientation_bin));
  _a_star->setStart(mx, my, orientation_bin_id);
  std::cout << "设置起点" << std::endl;
  // // Set goal point, in A* bin search coordinates
  if (!costmap_->worldToMapContinuous(goal.pose.position.x, goal.pose.position.y, mx, my)) {
    // throw nav2_core::GoalOutsideMapBounds(
    //         "Goal Coordinates of(" + std::to_string(goal.pose.position.x) + ", " +
    //         std::to_string(goal.pose.position.y) + ") was outside bounds");
  }
  orientation_bin = tf2::getYaw(goal.pose.orientation) / _angle_bin_size;
  while (orientation_bin < 0.0) {
    orientation_bin += static_cast<float>(_angle_quantizations);
  }
  // This is needed to handle precision issues
  if (orientation_bin >= static_cast<float>(_angle_quantizations)) {
    orientation_bin -= static_cast<float>(_angle_quantizations);
  }
  orientation_bin_id = static_cast<unsigned int>(floor(orientation_bin));
  _a_star->setGoal(mx, my, orientation_bin_id);
  std::cout << "设置终点" << std::endl;
  // // Setup message
  // nav_msgs::Path plan;
  // plan.header.stamp = _clock->now();
  // plan.header.frame_id = _global_frame;
  geometry_msgs::PoseStamped pose = goal;
  // pose.header = plan.header;
  // pose.pose.position.z = 0.0;
  // pose.pose.orientation.x = 0.0;
  // pose.pose.orientation.y = 0.0;
  // pose.pose.orientation.z = 0.0;
  // pose.pose.orientation.w = 1.0;

  // // Compute plan
  NodeHybrid::CoordinateVector path;
  int num_iterations = 0;
  std::string error;
  std::unique_ptr<std::vector<std::tuple<float, float, float>>> expansions = nullptr;
  // if (_debug_visualizations) {
    expansions = std::make_unique<std::vector<std::tuple<float, float, float>>>();
  // }
  std::cout << "设置开始规划路径" << std::endl;
  // // Note: All exceptions thrown are handled by the planner server and returned to the action
  if (!_a_star->createPath(
      path, num_iterations,
      _tolerance / static_cast<float>(costmap_->getResolution()), nullptr,/*cancel_checker*/ expansions.get()))
  {
    // if (_debug_visualizations) {
    //   geometry_msgs::msg::PoseArray msg;
    //   geometry_msgs::msg::Pose msg_pose;
    //   msg.header.stamp = _clock->now();
    //   msg.header.frame_id = _global_frame;
    //   for (auto & e : *expansions) {
    //     msg_pose.position.x = std::get<0>(e);
    //     msg_pose.position.y = std::get<1>(e);
    //     msg_pose.orientation = getWorldOrientation(std::get<2>(e));
    //     msg.poses.push_back(msg_pose);
    //   }
    //   _expansions_publisher->publish(msg);
    // }

    // Note: If the start is blocked only one iteration will occur before failure
    if (num_iterations == 1) {
      // throw nav2_core::StartOccupied("Start occupied");
    }

    if (num_iterations < _a_star->getMaxIterations()) {
      // throw nav2_core::NoValidPathCouldBeFound("no valid path found");
    } else {
      // throw nav2_core::PlannerTimedOut("exceeded maximum iterations");
    }
  }

auto  getWorldOrientation = [=](
  const float & theta) -> geometry_msgs::Quaternion
{
  // theta is in radians already
  tf2::Quaternion q;
  q.setEuler(0.0, 0.0, theta);
  return tf2::toMsg(q);
};

  if (1) {
      geometry_msgs::PoseArray msg;
      geometry_msgs::Pose msg_pose;
      // msg.header.stamp = _clock->now();
      msg.header.frame_id = "map";
      for (auto & e : *expansions) {
        msg_pose.position.x = std::get<0>(e);
        msg_pose.position.y = std::get<1>(e);
        msg_pose.orientation = getWorldOrientation(std::get<2>(e));
        msg.poses.push_back(msg_pose);
      }
      pose_array_pub_.publish(msg);
    }




  auto  getWorldCoords = [=] (
  const float & mx, const float & my, const costmap_2d::Costmap2D * costmap) -> geometry_msgs::Pose
{
  geometry_msgs::Pose msg;
  msg.position.x =
    static_cast<float>(costmap->getOriginX()) + (mx + 0.5) * costmap->getResolution();
  msg.position.y =
    static_cast<float>(costmap->getOriginY()) + (my + 0.5) * costmap->getResolution();
  return msg;
};
  // // Convert to world coordinates
  // plan.poses.reserve(path.size());
  plan.clear();
  for (int i = path.size() - 1; i >= 0; --i) {
    pose.pose = getWorldCoords(path[i].x, path[i].y, costmap_);
    // std::cout << "POSE = " << pose.pose.position.x << " " << pose.pose.position.y << std::endl;
    // pose.pose.orientation = getWorldOrientation(path[i].theta);
    // plan.poses.push_back(pose);
    plan.push_back(pose);
  }
  
  return true;
}

};  // namespace hybrid_astar_planner
