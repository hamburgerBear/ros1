// Copyright (c) 2020, Samsung Research America
// Copyright (c) 2020, Applied Electric Vehicles Pty Ltd
// Copyright (c) 2023, Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#include <math.h>
#include <chrono>
#include <vector>
#include <memory>
#include <algorithm>
#include <queue>
#include <limits>
#include <utility>

// #include "ompl/base/ScopedState.h"
// #include "ompl/base/spaces/DubinsStateSpace.h"
// #include "ompl/base/spaces/ReedsSheppStateSpace.h"
#include "hybrid_astar_planner/curve/dubins.h"
#include "hybrid_astar_planner/curve/reed_shepp.h"

#include "hybrid_astar_planner/node_hybrid.h"

using namespace std::chrono;  // NOLINT

namespace hybrid_astar_planner
{

// defining static member for all instance to share
LookupTable NodeHybrid::obstacle_heuristic_lookup_table;
float NodeHybrid::travel_distance_cost = sqrtf(2.0f);
HybridMotionTable NodeHybrid::motion_table;
float NodeHybrid::size_lookup = 25;
LookupTable NodeHybrid::dist_heuristic_lookup_table;
std::shared_ptr<costmap_2d::Costmap2DROS> NodeHybrid::costmap_ros = nullptr;
// std::shared_ptr<costmap_2d::InflationLayer> NodeHybrid::inflation_layer = nullptr;

ObstacleHeuristicQueue NodeHybrid::obstacle_heuristic_queue;

// Each of these tables are the projected motion models through
// time and space applied to the search on the current node in
// continuous map-coordinates (e.g. not meters but partial map cells)
// Currently, these are set to project *at minimum* into a neighboring
// cell. Though this could be later modified to project a certain
// amount of time or particular distance forward.

// http://planning.cs.uiuc.edu/node821.html
// Model for ackermann style vehicle with minimum radius restriction
void HybridMotionTable::initDubin(
  unsigned int & size_x_in,
  unsigned int & /*size_y_in*/,
  unsigned int & num_angle_quantization_in,
  SearchInfo & search_info)
{
  size_x = size_x_in;
  change_penalty = search_info.change_penalty;
  non_straight_penalty = search_info.non_straight_penalty;
  cost_penalty = search_info.cost_penalty;
  reverse_penalty = search_info.reverse_penalty;
  travel_distance_reward = 1.0f - search_info.retrospective_penalty;
  downsample_obstacle_heuristic = search_info.downsample_obstacle_heuristic;
  use_quadratic_cost_penalty = search_info.use_quadratic_cost_penalty;

  // if nothing changed, no need to re-compute primitives
  if (num_angle_quantization_in == num_angle_quantization &&
    min_turning_radius == search_info.minimum_turning_radius &&
    motion_model == MotionModel::DUBIN)
  {
    return;
  }

  num_angle_quantization = num_angle_quantization_in;
  num_angle_quantization_float = static_cast<float>(num_angle_quantization);
  min_turning_radius = search_info.minimum_turning_radius;
  motion_model = MotionModel::DUBIN;

  // angle must meet 3 requirements:
  // 1) be increment of quantized bin size
  // 2) chord length must be greater than sqrt(2) to leave current cell
  // 3) maximum curvature must be respected, represented by minimum turning angle
  // Thusly:
  // On circle of radius minimum turning angle, we need select motion primatives
  // with chord length > sqrt(2) and be an increment of our bin size
  //
  // chord >= sqrt(2) >= 2 * R * sin (angle / 2); where angle / N = quantized bin size
  // Thusly: angle <= 2.0 * asin(sqrt(2) / (2 * R))
  float angle = 2.0 * asin(sqrt(2.0) / (2 * min_turning_radius));
  std::cout << "angle1 = " << angle << std::endl; 
  // Now make sure angle is an increment of the quantized bin size
  // And since its based on the minimum chord, we need to make sure its always larger
  bin_size =
    2.0f * static_cast<float>(M_PI) / static_cast<float>(num_angle_quantization);
  float increments;
  if (angle < bin_size) {
    increments = 1.0f;
  } else {
    // Search dimensions are clean multiples of quantization - this prevents
    // paths with loops in them
    increments = ceil(angle / bin_size);
  }
  angle = increments * bin_size;
  std::cout << "angle2 = " << angle << std::endl;
  /*
    angle1的5.1度变成10度了。相当于步长扩大一倍，所以弦长从设想的sqrt(2)=1.414扩大至了2.78898
    弧长为2.79253，那么在地图分辨率为0.05的情况下。步长约为10cm。
    angle1 = 0.0884171
    angle2 = 0.174533
  */
  std::cout << "min_turning_radius = " << min_turning_radius << std::endl;
  // find deflections
  // If we make a right triangle out of the chord in circle of radius
  // min turning angle, we can see that delta X = R * sin (angle)
  const float delta_x = min_turning_radius * sin(angle);
  // Using that same right triangle, we can see that the complement
  // to delta Y is R * cos (angle). If we subtract R, we get the actual value
  const float delta_y = min_turning_radius - (min_turning_radius * cos(angle));
  const float delta_dist = hypotf(delta_x, delta_y);

  projections.clear();
  projections.reserve(3);
  projections.emplace_back(delta_dist, 0.0, 0.0, TurnDirection::FORWARD);  // Forward
  projections.emplace_back(delta_x, delta_y, increments, TurnDirection::LEFT);  // Left
  projections.emplace_back(delta_x, -delta_y, -increments, TurnDirection::RIGHT);  // Right
  std::cout << "delta_x = " << delta_x << std::endl;
  std::cout << "delta_y = " << delta_y << std::endl;

  if (search_info.allow_primitive_interpolation && increments > 1.0f) {
    // Create primitives that are +/- N to fill in search space to use all set angular quantizations
    // Allows us to create N many primitives so that each search iteration can expand into any angle
    // bin possible with the minimum turning radius constraint, not just the most extreme turns.
    projections.reserve(3 + (2 * (increments - 1)));
    for (unsigned int i = 1; i < static_cast<unsigned int>(increments); i++) {
      const float angle_n = static_cast<float>(i) * bin_size;
      const float turning_rad_n = delta_dist / (2.0f * sin(angle_n / 2.0f));
      const float delta_x_n = turning_rad_n * sin(angle_n);
      const float delta_y_n = turning_rad_n - (turning_rad_n * cos(angle_n));
      projections.emplace_back(
        delta_x_n, delta_y_n, static_cast<float>(i), TurnDirection::LEFT);  // Left
      projections.emplace_back(
        delta_x_n, -delta_y_n, -static_cast<float>(i), TurnDirection::RIGHT);  // Right
    }
  }

  // Create the correct OMPL state space
  state_space = std::make_shared<DubinsStateSpace>(min_turning_radius);

  // Precompute projection deltas
  delta_xs.resize(projections.size());
  delta_ys.resize(projections.size());
  trig_values.resize(num_angle_quantization);

  for (unsigned int i = 0; i != projections.size(); i++) {
    delta_xs[i].resize(num_angle_quantization);
    delta_ys[i].resize(num_angle_quantization);

    for (unsigned int j = 0; j != num_angle_quantization; j++) {
      double cos_theta = cos(bin_size * j);
      double sin_theta = sin(bin_size * j);
      if (i == 0) {
        // if first iteration, cache the trig values for later
        trig_values[j] = {cos_theta, sin_theta};
      }
      delta_xs[i][j] = projections[i]._x * cos_theta - projections[i]._y * sin_theta;
      delta_ys[i][j] = projections[i]._x * sin_theta + projections[i]._y * cos_theta;
    }
  }


  // Precompute travel costs for each motion primitive
  travel_costs.resize(projections.size());
  for (unsigned int i = 0; i != projections.size(); i++) {
    const TurnDirection turn_dir = projections[i]._turn_dir;
    if (turn_dir != TurnDirection::FORWARD && turn_dir != TurnDirection::REVERSE) {
      // Turning, so length is the arc length
      //弦长 = 2sin(a/2)*R  弧长 = AR = L/2sin(a/2) * A
      const float angle = projections[i]._theta * bin_size;                //角度
      const float turning_rad = delta_dist / (2.0f * sin(angle / 2.0f));   //
      travel_costs[i] = turning_rad * angle;                               //arc距离
      std::cout << "travel_costs[i] = " << travel_costs[i] << std::endl;
      std::cout << "delta_dist = " << delta_dist << std::endl;               
    } else {
      travel_costs[i] = delta_dist;                                        //欧式距离
    }
  }
}

// http://planning.cs.uiuc.edu/node822.html
// Same as Dubin model but now reverse is valid
// See notes in Dubin for explanation
void HybridMotionTable::initReedsShepp(
  unsigned int & size_x_in,
  unsigned int & /*size_y_in*/,
  unsigned int & num_angle_quantization_in,
  SearchInfo & search_info)
{
  size_x = size_x_in;
  change_penalty = search_info.change_penalty;
  non_straight_penalty = search_info.non_straight_penalty;
  cost_penalty = search_info.cost_penalty;
  reverse_penalty = search_info.reverse_penalty;
  travel_distance_reward = 1.0f - search_info.retrospective_penalty;
  downsample_obstacle_heuristic = search_info.downsample_obstacle_heuristic;
  use_quadratic_cost_penalty = search_info.use_quadratic_cost_penalty;

  // if nothing changed, no need to re-compute primitives
  if (num_angle_quantization_in == num_angle_quantization &&
    min_turning_radius == search_info.minimum_turning_radius &&
    motion_model == MotionModel::REEDS_SHEPP)
  {
    return;
  }

  num_angle_quantization = num_angle_quantization_in;
  num_angle_quantization_float = static_cast<float>(num_angle_quantization);
  min_turning_radius = search_info.minimum_turning_radius;
  motion_model = MotionModel::REEDS_SHEPP;

  float angle = 2.0 * asin(sqrt(2.0) / (2 * min_turning_radius));
  bin_size =
    2.0f * static_cast<float>(M_PI) / static_cast<float>(num_angle_quantization);
  float increments;
  if (angle < bin_size) {
    increments = 1.0f;
  } else {
    increments = ceil(angle / bin_size);
  }
  angle = increments * bin_size;

  const float delta_x = min_turning_radius * sin(angle);
  const float delta_y = min_turning_radius - (min_turning_radius * cos(angle));
  const float delta_dist = hypotf(delta_x, delta_y);

  projections.clear();
  projections.reserve(6);
  projections.emplace_back(delta_dist, 0.0, 0.0, TurnDirection::FORWARD);  // Forward
  projections.emplace_back(
    delta_x, delta_y, increments, TurnDirection::LEFT);  // Forward + Left
  projections.emplace_back(
    delta_x, -delta_y, -increments, TurnDirection::RIGHT);  // Forward + Right
  projections.emplace_back(-delta_dist, 0.0, 0.0, TurnDirection::REVERSE);  // Backward
  projections.emplace_back(
    -delta_x, delta_y, -increments, TurnDirection::REV_LEFT);  // Backward + Left
  projections.emplace_back(
    -delta_x, -delta_y, increments, TurnDirection::REV_RIGHT);  // Backward + Right

  if (search_info.allow_primitive_interpolation && increments > 1.0f) {
    // Create primitives that are +/- N to fill in search space to use all set angular quantizations
    // Allows us to create N many primitives so that each search iteration can expand into any angle
    // bin possible with the minimum turning radius constraint, not just the most extreme turns.
    projections.reserve(6 + (4 * (increments - 1)));
    for (unsigned int i = 1; i < static_cast<unsigned int>(increments); i++) {
      const float angle_n = static_cast<float>(i) * bin_size;
      const float turning_rad_n = delta_dist / (2.0f * sin(angle_n / 2.0f));
      const float delta_x_n = turning_rad_n * sin(angle_n);
      const float delta_y_n = turning_rad_n - (turning_rad_n * cos(angle_n));
      projections.emplace_back(
        delta_x_n, delta_y_n, static_cast<float>(i), TurnDirection::LEFT);  // Forward + Left
      projections.emplace_back(
        delta_x_n, -delta_y_n, -static_cast<float>(i), TurnDirection::RIGHT);  // Forward + Right
      projections.emplace_back(
        -delta_x_n, delta_y_n, -static_cast<float>(i),
        TurnDirection::REV_LEFT);  // Backward + Left
      projections.emplace_back(
        -delta_x_n, -delta_y_n, static_cast<float>(i),
        TurnDirection::REV_RIGHT);  // Backward + Right
    }
  }

  // Create the correct OMPL state space
  state_space = std::make_shared<ReedsSheppStateSpace>(min_turning_radius);

  // Precompute projection deltas
  delta_xs.resize(projections.size());
  delta_ys.resize(projections.size());
  trig_values.resize(num_angle_quantization);

  for (unsigned int i = 0; i != projections.size(); i++) {
    delta_xs[i].resize(num_angle_quantization);
    delta_ys[i].resize(num_angle_quantization);

    for (unsigned int j = 0; j != num_angle_quantization; j++) {
      double cos_theta = cos(bin_size * j);
      double sin_theta = sin(bin_size * j);
      if (i == 0) {
        // if first iteration, cache the trig values for later
        trig_values[j] = {cos_theta, sin_theta};
      }
      delta_xs[i][j] = projections[i]._x * cos_theta - projections[i]._y * sin_theta;
      delta_ys[i][j] = projections[i]._x * sin_theta + projections[i]._y * cos_theta;
    }
  }

  // Precompute travel costs for each motion primitive
  travel_costs.resize(projections.size());
  for (unsigned int i = 0; i != projections.size(); i++) {
    const TurnDirection turn_dir = projections[i]._turn_dir;
    if (turn_dir != TurnDirection::FORWARD && turn_dir != TurnDirection::REVERSE) {
      // Turning, so length is the arc length
      const float angle = projections[i]._theta * bin_size;
      const float turning_rad = delta_dist / (2.0f * sin(angle / 2.0f));
      travel_costs[i] = turning_rad * angle;
    } else {
      travel_costs[i] = delta_dist;
    }
  }
}

MotionPoses HybridMotionTable::getProjections(const NodeHybrid * node)
{
  MotionPoses projection_list;
  projection_list.reserve(projections.size());

  for (unsigned int i = 0; i != projections.size(); i++) {
    const MotionPose & motion_model = projections[i];

    // normalize theta, I know its overkill, but I've been burned before...
    const float & node_heading = node->pose.theta;
    float new_heading = node_heading + motion_model._theta;

    if (new_heading < 0.0) {
      new_heading += num_angle_quantization_float;
    }

    if (new_heading >= num_angle_quantization_float) {
      new_heading -= num_angle_quantization_float;
    }

    projection_list.emplace_back(
      delta_xs[i][node_heading] + node->pose.x,
      delta_ys[i][node_heading] + node->pose.y,
      new_heading, motion_model._turn_dir);
  }

  return projection_list;
}

unsigned int HybridMotionTable::getClosestAngularBin(const double & theta)
{
  return static_cast<unsigned int>(floor(theta / static_cast<double>(bin_size))) %
         num_angle_quantization;
}

float HybridMotionTable::getAngleFromBin(const unsigned int & bin_idx)
{
  return bin_idx * bin_size;
}

NodeHybrid::NodeHybrid(const uint64_t index)
: parent(nullptr),
  pose(0.0f, 0.0f, 0.0f),
  _cell_cost(std::numeric_limits<float>::quiet_NaN()),
  _accumulated_cost(std::numeric_limits<float>::max()),
  _index(index),
  _was_visited(false),
  _motion_primitive_index(std::numeric_limits<unsigned int>::max())
{
}

NodeHybrid::~NodeHybrid()
{
  parent = nullptr;
}

void NodeHybrid::reset()
{
  parent = nullptr;
  _cell_cost = std::numeric_limits<float>::quiet_NaN();
  _accumulated_cost = std::numeric_limits<float>::max();
  _was_visited = false;
  _motion_primitive_index = std::numeric_limits<unsigned int>::max();
  pose.x = 0.0f;
  pose.y = 0.0f;
  pose.theta = 0.0f;
}

bool NodeHybrid::isNodeValid(
  const bool & traverse_unknown,
  GridCollisionChecker * collision_checker)
{
  if (collision_checker->inCollision(
      this->pose.x, this->pose.y, this->pose.theta /*bin number*/, traverse_unknown))
  {
    return false;
  }

  _cell_cost = collision_checker->getCost();
  return true;
}

float NodeHybrid::getTraversalCost(const NodePtr & child)
{
  const float normalized_cost = child->getCost() / 252.0f;
  if (std::isnan(normalized_cost)) {
    throw std::runtime_error(
            "Node attempted to get traversal "
            "cost without a known SE2 collision cost!");
  }

  // this is the first node
  //当前节点没有运动基元(方向)索引，表示第一点，旅行代价都默认为第一步旅行代价，约=sqrt(2.0)
  //之后扩展的节点运动基元(方向)索引，会在getNeighbors函数中，调用setMotionPrimitiveIndex设置。

  if (getMotionPrimitiveIndex() == std::numeric_limits<unsigned int>::max()) { 
    return NodeHybrid::travel_distance_cost;
  }

  const TurnDirection & child_turn_dir = child->getTurnDirection(); //child子节点为邻域节点，得到邻域节点的转向
  float travel_cost_raw = motion_table.travel_costs[child->getMotionPrimitiveIndex()]; //分别得到左/中/右的旅行代价
  float travel_cost = 0.0;

  //使用二次代价惩戒  
  /*
  高级特性：默认关闭
  这允许使用者去指定一个二次旅行和启发代价,而非线性。
  它将更快速在最优通道，对于可行搜索更深如，并且裁剪搜索分支更积极
  这也将创建更平滑的路径，因为搜索不会尝试改进自身以保持在宽阔过道或开放空间的中心，以降低有限的低成本
  然而，平顺性和对成本的不太敏感也使它在某种程度上更接近障碍物。
  一般来说，在使用此功能时，可以禁用更改和非直接处罚。
  在启用该参数以创建最佳性能时，可能需要调整成本惩罚和膨胀层参数。
  */

  // retrospective_penalty 和 travel_distance_reward 参数不是很理解
  // 回顾性惩罚和旅行距离奖励?????
  if (motion_table.use_quadratic_cost_penalty) {
    travel_cost_raw *=
      (motion_table.travel_distance_reward +                            //旅行距离奖励=0.98，这个参数不是很清晰
      (motion_table.cost_penalty * normalized_cost * normalized_cost)); //地形代价二次项
  } else {
    travel_cost_raw *=
      (motion_table.travel_distance_reward + motion_table.cost_penalty * normalized_cost); //
  }
  // travel_cost_raw 原始旅行代价
  // kp = travel_distance_reward + 地形代价*地形代价系数
  // 如果同向旋转，考虑非直线旋转系数
  // 如果非同乡旋转，考虑非直线旋转系数+转向惩戒系数

  //如果子节点旋转方向是前进或者后退
  if (child_turn_dir == TurnDirection::FORWARD || child_turn_dir == TurnDirection::REVERSE) {
    // New motion is a straight motion, no additional costs to be applied
    travel_cost = travel_cost_raw;
  } 
  //如果子节点的旋转方向是转弯
  //non_straight_penalty[1.0]
  //change_penalty[0.0]
  else {
    //当前转向相同
    if (getTurnDirection() == child_turn_dir) {
      // Turning motion but keeps in same direction: encourages to commit to turning if starting it
      travel_cost = travel_cost_raw * motion_table.non_straight_penalty;
    } 
    //当前节点的转向和上一个节点转向不同
    else {
      // Turning motion and changing direction: penalizes wiggling
      travel_cost = travel_cost_raw *
        (motion_table.non_straight_penalty + motion_table.change_penalty);
    }
  }

  //如果当前方向是后退，那么还需要考虑增加后退的惩罚[2.0]
  if (child_turn_dir == TurnDirection::REV_RIGHT ||
    child_turn_dir == TurnDirection::REV_LEFT ||
    child_turn_dir == TurnDirection::REVERSE)
  {
    // reverse direction
    travel_cost *= motion_table.reverse_penalty;
  }

  return travel_cost;
}

float NodeHybrid::getHeuristicCost(
  const Coordinates & node_coords,
  const Coordinates & goal_coords)
{
  const float obstacle_heuristic =
    getObstacleHeuristic(node_coords, goal_coords, motion_table.cost_penalty);
  const float dist_heuristic = getDistanceHeuristic(node_coords, goal_coords, obstacle_heuristic);
  return std::max(obstacle_heuristic, dist_heuristic);
}

void NodeHybrid::initMotionModel(
  const MotionModel & motion_model,
  unsigned int & size_x,
  unsigned int & size_y,
  unsigned int & num_angle_quantization,
  SearchInfo & search_info)
{
  // find the motion model selected
  switch (motion_model) {
    case MotionModel::DUBIN:
      motion_table.initDubin(size_x, size_y, num_angle_quantization, search_info);
      break;
    case MotionModel::REEDS_SHEPP:
      motion_table.initReedsShepp(size_x, size_y, num_angle_quantization, search_info);
      break;
    default:
      throw std::runtime_error(
              "Invalid motion model for Hybrid A*. Please select between"
              " Dubin (Ackermann forward only),"
              " Reeds-Shepp (Ackermann forward and back).");
  }

  travel_distance_cost = motion_table.projections[0]._x;
}

inline float distanceHeuristic2D(
  const uint64_t idx, const unsigned int size_x,
  const unsigned int target_x, const unsigned int target_y)
{
  int dx = static_cast<int>(idx % size_x) - static_cast<int>(target_x);
  int dy = static_cast<int>(idx / size_x) - static_cast<int>(target_y);
  return std::sqrt(dx * dx + dy * dy);
}

void NodeHybrid::resetObstacleHeuristic(
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros_i,
  const unsigned int & start_x, const unsigned int & start_y,
  const unsigned int & goal_x, const unsigned int & goal_y)
{
  // Downsample costmap 2x to compute a sparse obstacle heuristic. This speeds up
  // the planner considerably to search through 75% less cells with no detectable
  // erosion of path quality after even modest smoothing. The error would be no more
  // than 0.05 * normalized cost. Since this is just a search prior, there's no loss in generality
  costmap_ros = costmap_ros_i;
  // inflation_layer = costmap_2d::InflationLayer::getInflationLayer(costmap_ros);
  auto costmap = costmap_ros->getCostmap();

  // Clear lookup table
  unsigned int size = 0u;
  unsigned int size_x = 0u;
  if (motion_table.downsample_obstacle_heuristic) {
    size_x = ceil(static_cast<float>(costmap->getSizeInCellsX()) / 2.0f);
    size = size_x *
      ceil(static_cast<float>(costmap->getSizeInCellsY()) / 2.0f);
  } else {
    size_x = costmap->getSizeInCellsX();
    size = size_x * costmap->getSizeInCellsY();
  }

  if (obstacle_heuristic_lookup_table.size() == size) {
    // must reset all values
    std::fill(
      obstacle_heuristic_lookup_table.begin(),
      obstacle_heuristic_lookup_table.end(), 0.0f);
  } else {
    unsigned int obstacle_size = obstacle_heuristic_lookup_table.size();
    obstacle_heuristic_lookup_table.resize(size, 0.0f);
    // must reset values for non-constructed indices
    std::fill_n(
      obstacle_heuristic_lookup_table.begin(), obstacle_size, 0.0f);
  }

  obstacle_heuristic_queue.clear();
  obstacle_heuristic_queue.reserve(size);

  // Set initial goal point to queue from. Divided by 2 due to downsampled costmap.
  unsigned int goal_index;
  if (motion_table.downsample_obstacle_heuristic) {
    goal_index = floor(goal_y / 2.0f) * size_x + floor(goal_x / 2.0f);
  } else {
    goal_index = floor(goal_y) * size_x + floor(goal_x);
  }

  obstacle_heuristic_queue.emplace_back(
    distanceHeuristic2D(goal_index, size_x, start_x, start_y), goal_index);

  // initialize goal cell with a very small value to differentiate it from 0.0 (~uninitialized)
  // the negative value means the cell is in the open set
  obstacle_heuristic_lookup_table[goal_index] = -0.00001f;
}

float NodeHybrid::adjustedFootprintCost(const float & cost)
{
  // if (!inflation_layer) {
  //   return cost;
  // }

  const auto layered_costmap = costmap_ros->getLayeredCostmap();
  // const float scale_factor = inflation_layer->getCostScalingFactor();
  // TODO:gaojie 
  const float scale_factor = 1.0;
  const float min_radius = layered_costmap->getInscribedRadius();
  float dist_to_obj = (scale_factor * min_radius - log(cost) + log(253.0f)) / scale_factor;

  // Subtract minimum radius for edge cost
  dist_to_obj -= min_radius;
  if (dist_to_obj < 0.0f) {
    dist_to_obj = 0.0f;
  }

  // Compute cost at this value
  //TODO:GAOJIE
  return 1.0;
  // return static_cast<float>(
  //   inflation_layer->computeCost(dist_to_obj / layered_costmap->getCostmap()->getResolution()));
}


float NodeHybrid::getObstacleHeuristic(
  const Coordinates & node_coords,
  const Coordinates & goal_coords,
  const float & cost_penalty)
{
  // If already expanded, return the cost
  auto costmap = costmap_ros->getCostmap();
  const bool is_circular = false; //TODO GAOJIEcostmap_ros->getUseRadius();
  unsigned int size_x = 0u;
  unsigned int size_y = 0u;
  if (motion_table.downsample_obstacle_heuristic) {
    size_x = ceil(static_cast<float>(costmap->getSizeInCellsX()) / 2.0f);
    size_y = ceil(static_cast<float>(costmap->getSizeInCellsY()) / 2.0f);
  } else {
    size_x = costmap->getSizeInCellsX();
    size_y = costmap->getSizeInCellsY();
  }

  // Divided by 2 due to downsampled costmap.
  unsigned int start_y, start_x;
  const bool & downsample_H = motion_table.downsample_obstacle_heuristic;
  if (downsample_H) {
    start_y = floor(node_coords.y / 2.0f);
    start_x = floor(node_coords.x / 2.0f);
  } else {
    start_y = floor(node_coords.y);
    start_x = floor(node_coords.x);
  }

  const unsigned int start_index = start_y * size_x + start_x;
  const float & requested_node_cost = obstacle_heuristic_lookup_table[start_index];
  if (requested_node_cost > 0.0f) {
    // costs are doubled due to downsampling
    return downsample_H ? 2.0f * requested_node_cost : requested_node_cost;
  }

  // If not, expand until it is included. This dynamic programming ensures that
  // we only expand the MINIMUM spanning set of the costmap per planning request.
  // Rather than naively expanding the entire (potentially massive) map for a limited
  // path, we only expand to the extent required for the furthest expansion in the
  // search-planning request that dynamically updates during search as needed.

  // start_x and start_y have changed since last call
  // we need to recompute 2D distance heuristic and reprioritize queue
  for (auto & n : obstacle_heuristic_queue) {
    n.first = -obstacle_heuristic_lookup_table[n.second] +
      distanceHeuristic2D(n.second, size_x, start_x, start_y);
  }
  std::make_heap(
    obstacle_heuristic_queue.begin(), obstacle_heuristic_queue.end(),
    ObstacleHeuristicComparator{});

  const int size_x_int = static_cast<int>(size_x);
  const float sqrt2 = sqrtf(2.0f);
  float c_cost, cost, travel_cost, new_cost, existing_cost;
  unsigned int mx, my;
  unsigned int idx, new_idx = 0;

  const std::vector<int> neighborhood = {1, -1,  // left right
    size_x_int, -size_x_int,  // up down
    size_x_int + 1, size_x_int - 1,  // upper diagonals
    -size_x_int + 1, -size_x_int - 1};  // lower diagonals

  while (!obstacle_heuristic_queue.empty()) {
    idx = obstacle_heuristic_queue.front().second;
    std::pop_heap(
      obstacle_heuristic_queue.begin(), obstacle_heuristic_queue.end(),
      ObstacleHeuristicComparator{});
    obstacle_heuristic_queue.pop_back();
    c_cost = obstacle_heuristic_lookup_table[idx];
    if (c_cost > 0.0f) {
      // cell has been processed and closed, no further cost improvements
      // are mathematically possible thanks to euclidean distance heuristic consistency
      continue;
    }
    c_cost = -c_cost;
    obstacle_heuristic_lookup_table[idx] = c_cost;  // set a positive value to close the cell

    // find neighbors
    for (unsigned int i = 0; i != neighborhood.size(); i++) {
      new_idx = static_cast<unsigned int>(static_cast<int>(idx) + neighborhood[i]);

      // if neighbor path is better and non-lethal, set new cost and add to queue
      if (new_idx < size_x * size_y) {
        if (downsample_H) {
          // Get costmap values as if downsampled
          unsigned int y_offset = (new_idx / size_x) * 2;
          unsigned int x_offset = (new_idx - ((new_idx / size_x) * size_x)) * 2;
          cost = costmap->getCost(x_offset, y_offset);
          for (unsigned int i = 0; i < 2u; ++i) {
            unsigned int mxd = x_offset + i;
            if (mxd >= costmap->getSizeInCellsX()) {
              continue;
            }
            for (unsigned int j = 0; j < 2u; ++j) {
              unsigned int myd = y_offset + j;
              if (myd >= costmap->getSizeInCellsY()) {
                continue;
              }
              if (i == 0 && j == 0) {
                continue;
              }
              cost = std::min(cost, static_cast<float>(costmap->getCost(mxd, myd)));
            }
          }
        } else {
          cost = static_cast<float>(costmap->getCost(new_idx));
        }

        if (!is_circular) {
          // Adjust cost value if using SE2 footprint checks
          cost = adjustedFootprintCost(cost);
          if (cost >= OCCUPIED) {
            continue;
          }
        } else if (cost >= INSCRIBED) {
          continue;
        }

        my = new_idx / size_x;
        mx = new_idx - (my * size_x);

        if (mx >= size_x - 3 || mx <= 3) {
          continue;
        }
        if (my >= size_y - 3 || my <= 3) {
          continue;
        }

        existing_cost = obstacle_heuristic_lookup_table[new_idx];
        if (existing_cost <= 0.0f) {
          if (motion_table.use_quadratic_cost_penalty) {
            travel_cost =
              (i <= 3 ? 1.0f : sqrt2) * (1.0f + (cost_penalty * cost * cost / 63504.0f));  // 252^2
          } else {
            travel_cost =
              ((i <= 3) ? 1.0f : sqrt2) * (1.0f + (cost_penalty * cost / 252.0f));
          }

          new_cost = c_cost + travel_cost;
          if (existing_cost == 0.0f || -existing_cost > new_cost) {
            // the negative value means the cell is in the open set
            obstacle_heuristic_lookup_table[new_idx] = -new_cost;
            obstacle_heuristic_queue.emplace_back(
              new_cost + distanceHeuristic2D(new_idx, size_x, start_x, start_y), new_idx);
            std::push_heap(
              obstacle_heuristic_queue.begin(), obstacle_heuristic_queue.end(),
              ObstacleHeuristicComparator{});
          }
        }
      }
    }

    if (idx == start_index) {
      break;
    }
  }

  // #include "nav_msgs/msg/occupancy_grid.hpp"
  // static auto node = std::make_shared<rclcpp::Node>("test");
  // static auto pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>("test", 1);
  // nav_msgs::msg::OccupancyGrid msg;
  // msg.info.height = size_y;
  // msg.info.width = size_x;
  // msg.info.origin.position.x = -33.6;
  // msg.info.origin.position.y = -26;
  // msg.info.resolution = 0.05;
  // msg.header.frame_id = "map";
  // msg.header.stamp = node->now();
  // msg.data.resize(size_x * size_y, 0);
  // for (unsigned int i = 0; i != size_y * size_x; i++) {
  //   msg.data.at(i) = obstacle_heuristic_lookup_table[i] / 10.0;
  // }
  // pub->publish(std::move(msg));

  // return requested_node_cost which has been updated by the search
  // costs are doubled due to downsampling
  return downsample_H ? 2.0f * requested_node_cost : requested_node_cost;
}

float NodeHybrid::getDistanceHeuristic(
  const Coordinates & node_coords,
  const Coordinates & goal_coords,
  const float & obstacle_heuristic)
{
  // rotate and translate node_coords such that goal_coords relative is (0,0,0)
  // Due to the rounding involved in exact cell increments for caching,
  // this is not an exact replica of a live heuristic, but has bounded error.
  // (Usually less than 1 cell)

  // This angle is negative since we are de-rotating the current node
  // by the goal angle; cos(-th) = cos(th) & sin(-th) = -sin(th)
  const TrigValues & trig_vals = motion_table.trig_values[goal_coords.theta];
  const float cos_th = trig_vals.first;
  const float sin_th = -trig_vals.second;
  const float dx = node_coords.x - goal_coords.x;
  const float dy = node_coords.y - goal_coords.y;

  double dtheta_bin = node_coords.theta - goal_coords.theta;
  if (dtheta_bin < 0) {
    dtheta_bin += motion_table.num_angle_quantization;
  }
  if (dtheta_bin > motion_table.num_angle_quantization) {
    dtheta_bin -= motion_table.num_angle_quantization;
  }

  Coordinates node_coords_relative(
    round(dx * cos_th - dy * sin_th),
    round(dx * sin_th + dy * cos_th),
    round(dtheta_bin));

  // Check if the relative node coordinate is within the localized window around the goal
  // to apply the distance heuristic. Since the lookup table is contains only the positive
  // X axis, we mirror the Y and theta values across the X axis to find the heuristic values.
  float motion_heuristic = 0.0;
  const int floored_size = floor(size_lookup / 2.0);
  const int ceiling_size = ceil(size_lookup / 2.0);
  const float mirrored_relative_y = abs(node_coords_relative.y);
  if (abs(node_coords_relative.x) < floored_size && mirrored_relative_y < floored_size) {
    // Need to mirror angle if Y coordinate was mirrored
    int theta_pos;
    if (node_coords_relative.y < 0.0) {
      theta_pos = motion_table.num_angle_quantization - node_coords_relative.theta;
    } else {
      theta_pos = node_coords_relative.theta;
    }
    const int x_pos = node_coords_relative.x + floored_size;
    const int y_pos = static_cast<int>(mirrored_relative_y);
    const int index =
      x_pos * ceiling_size * motion_table.num_angle_quantization +
      y_pos * motion_table.num_angle_quantization +
      theta_pos;
    motion_heuristic = dist_heuristic_lookup_table[index];
  } else if (obstacle_heuristic <= 0.0) {
    // If no obstacle heuristic value, must have some H to use
    // In nominal situations, this should never be called.
    // static ompl::base::ScopedState<> from(motion_table.state_space), to(motion_table.state_space);
    double from[3] = { node_coords.x, node_coords.y, node_coords.theta * motion_table.num_angle_quantization };
    double to[3] = { goal_coords.x, goal_coords.y, goal_coords.theta * motion_table.num_angle_quantization };
    motion_table.state_space->init(from, to);
    motion_heuristic = motion_table.state_space->distance();
  }

  return motion_heuristic;
}

void NodeHybrid::precomputeDistanceHeuristic(
  const float & lookup_table_dim,
  const MotionModel & motion_model,
  const unsigned int & dim_3_size,
  const SearchInfo & search_info)
{
  // Dubin or Reeds-Shepp shortest distances
  if (motion_model == MotionModel::DUBIN) {
    motion_table.state_space = std::make_shared<DubinsStateSpace>(
      search_info.minimum_turning_radius);
  } else if (motion_model == MotionModel::REEDS_SHEPP) {
    motion_table.state_space = std::make_shared<ReedsSheppStateSpace>(
      search_info.minimum_turning_radius);
  } else {
    throw std::runtime_error(
            "Node attempted to precompute distance heuristics "
            "with invalid motion model!");
  }

  double from[3], to[3];
  // ompl::base::ScopedState<> from(motion_table.state_space), to(motion_table.state_space);
  to[0] = 0.0;
  to[1] = 0.0;
  to[2] = 0.0;
  size_lookup = lookup_table_dim;
  float motion_heuristic = 0.0;
  unsigned int index = 0;
  int dim_3_size_int = static_cast<int>(dim_3_size);
  float angular_bin_size = 2 * M_PI / static_cast<float>(dim_3_size);

  // Create a lookup table of Dubin/Reeds-Shepp distances in a window around the goal
  // to help drive the search towards admissible approaches. Deu to symmetries in the
  // Heuristic space, we need to only store 2 of the 4 quadrants and simply mirror
  // around the X axis any relative node lookup. This reduces memory overhead and increases
  // the size of a window a platform can store in memory.
  dist_heuristic_lookup_table.resize(size_lookup * ceil(size_lookup / 2.0) * dim_3_size_int);
  for (float x = ceil(-size_lookup / 2.0); x <= floor(size_lookup / 2.0); x += 1.0) {
    for (float y = 0.0; y <= floor(size_lookup / 2.0); y += 1.0) {
      for (int heading = 0; heading != dim_3_size_int; heading++) {
        from[0] = x;
        from[1] = y;
        from[2] = heading * angular_bin_size;
        motion_table.state_space->init(from, to);
        motion_heuristic = motion_table.state_space->distance();
        dist_heuristic_lookup_table[index] = motion_heuristic;
        index++;
      }
    }
  }
}

void NodeHybrid::getNeighbors(
  std::function<bool(const uint64_t &,
  hybrid_astar_planner::NodeHybrid * &)> & NeighborGetter,
  GridCollisionChecker * collision_checker,
  const bool & traverse_unknown,
  NodeVector & neighbors)
{
  uint64_t index = 0;
  NodePtr neighbor = nullptr;
  Coordinates initial_node_coords;
  const MotionPoses motion_projections = motion_table.getProjections(this);

  for (unsigned int i = 0; i != motion_projections.size(); i++) {
    index = NodeHybrid::getIndex(
      static_cast<unsigned int>(motion_projections[i]._x),
      static_cast<unsigned int>(motion_projections[i]._y),
      static_cast<unsigned int>(motion_projections[i]._theta),
      motion_table.size_x, motion_table.num_angle_quantization);

    //从graph中得到index对应的元素，如果没有则添加到图中。如果已经添加过了，判断该元素节点是否已经被访问过了。
    if (NeighborGetter(index, neighbor) && !neighbor->wasVisited()) {
      //缓存初始姿态，以防它被访问但有效
      //不希望破坏连续的坐标扩展
      // Cache the initial pose in case it was visited but valid
      // don't want to disrupt continuous coordinate expansion
      //得到邻域的初始化坐标，可能是000，如果新添加的情况
      initial_node_coords = neighbor->pose; 
      //设置该邻域节点的格栅坐标
      neighbor->setPose(
        Coordinates(
          motion_projections[i]._x,
          motion_projections[i]._y,
          motion_projections[i]._theta));
      //结合代价地图和机器人足迹轮廓，判断该邻域节点是否发生膨胀，如果没有发生碰撞。
      if (neighbor->isNodeValid(traverse_unknown, collision_checker)) {
      //设置运动基元索引
        neighbor->setMotionPrimitiveIndex(i, motion_projections[i]._turn_dir);
        neighbors.push_back(neighbor);
      } else {
        //这个邻域可能存在其他的有效位置，因为neighbor是index，所以可能存在多个不同的pose，在相同的index上。
      //当碰撞检测发生，当邻区是不可访问的区域，位置还原。如000
        //<尝试>TODO：应该只能是000
        neighbor->setPose(initial_node_coords);
                auto costmap = costmap_ros->getCostmap();
        // std::cout << "应该只能是000 = " 
        //   << costmap->getOriginX() + ((initial_node_coords.x + 0.5) * costmap->getResolution()) << " " 
        //   << costmap->getOriginY() + ((initial_node_coords.y + 0.5) * costmap->getResolution()) << std::endl;


      }
    }
  }
}

bool NodeHybrid::backtracePath(CoordinateVector & path)
{
  if (!this->parent) {
    return false;
  }

  NodePtr current_node = this;

  while (current_node->parent) {
    path.push_back(current_node->pose);
    // Convert angle to radians
    path.back().theta = NodeHybrid::motion_table.getAngleFromBin(path.back().theta);
    current_node = current_node->parent;
  }

  // add the start pose
  path.push_back(current_node->pose);
  // Convert angle to radians
  path.back().theta = NodeHybrid::motion_table.getAngleFromBin(path.back().theta);

  return true;
}

}  // namespace hybrid_astar_planner
