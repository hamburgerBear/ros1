#include "kinematic_planner/kinematic_planner_core.h"
#include <ros/ros.h>

namespace kinematic_planner {
kinematicPlannerCore::kinematicPlannerCore(
    costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros_(costmap_ros), collision_checker_(nullptr) {}

kinematicPlannerCore::~kinematicPlannerCore() {}

bool kinematicPlannerCore::initialize() {
  motion_primitives_filepath_ =
      "/home/gaojie/hybrid_1cm_7.json";  // lattice.json
  if (!initMotionPrimitivesTable(motion_primitives_filepath_)) return false;

  resolution_ = costmap_ros_->getCostmap()->getResolution();
  costmap_ = costmap_ros_->getCostmap();
  collision_checker_ = std::make_shared<GridCollisionChecker>(
      std::shared_ptr<costmap_2d::Costmap2DROS>(costmap_ros_),
      angle_quantizations_);
  collision_checker_->setFootprint(costmap_ros_->getRobotFootprint(), false,
                                   1.0);
  tolerance_xy_ = 0.03;
  tolerance_yaw_ = 0.03;
  return true;
}

bool kinematicPlannerCore::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& plan,
    std::vector<Eigen::Vector3f>& expands) {
  // boost::lock_guard<boost::mutex> lock(
  //     costmap_->getMutex());  // Lock is acquired here
  std::unordered_map<unsigned int, KinematicNode> empty_graph;
  std::swap(graph_, empty_graph);
  std::vector<QueueElement> empty_queue;
  std::swap(queue_, empty_queue);

  auto outlineMap = [](unsigned char* costarr, int nx, int ny,
                       unsigned char value) {
    unsigned char* pc;
    int outline_num = 3;
    for (int j = 0; j < outline_num; j++) {
      // down
      pc = costarr + (j)*nx;
      for (int i = 0; i < nx; i++) *pc++ = value;
      // up
      pc = costarr + (ny - 1 - j) * nx;
      for (int i = 0; i < nx; i++) *pc++ = value;
      // left
      pc = costarr + j;
      for (int i = 0; i < ny; i++, pc += nx) *pc = value;
      // right
      pc = costarr + nx - 1 - j;
      for (int i = 0; i < ny; i++, pc += nx) *pc = value;
    }
  };
  int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();
  // outlineMap(costmap_->getCharMap(), nx, ny, costmap_2d::LETHAL_OBSTACLE);

  unsigned int size_x = costmap_->getSizeInCellsX();

  float mx, my;
  float float_orientation;
  int int_orientation;
  float angle_gap = 2 * M_PI / static_cast<float>(angle_quantizations_);
  costmap_->worldToMapLoseless(start.pose.position.x, start.pose.position.y, mx,
                               my);
  float_orientation = tf2::getYaw(start.pose.orientation) / angle_gap;
  int_orientation = static_cast<int>(floor(float_orientation + 0.5));
  if (int_orientation >= static_cast<int>(angle_quantizations_))
    int_orientation -= angle_quantizations_;
  else if (int_orientation < 0) {
    int_orientation += angle_quantizations_;
  } else {
  }
  ROS_INFO("Start pose = {%f, %f, %d}.", mx, my, int_orientation);
  unsigned int index =
      static_cast<unsigned int>(my) * angle_quantizations_ * size_x +
      static_cast<unsigned int>(mx) * angle_quantizations_ + int_orientation;
  start_node_ = KinematicNode(false, 0.0, mx, my, int_orientation,
                              MotionDir::NONE, nullptr);
  graph_[index] = start_node_;
  queue_.push_back(QueueElement(index, 0.0));

  costmap_->worldToMapLoseless(goal.pose.position.x, goal.pose.position.y, mx,
                               my);
  float_orientation = tf2::getYaw(goal.pose.orientation) / angle_gap;
  int_orientation = static_cast<int>(floor(float_orientation + 0.5));
  if (int_orientation >= static_cast<int>(angle_quantizations_))
    int_orientation -= angle_quantizations_;
  else if (int_orientation < 0) {
    int_orientation += angle_quantizations_;
  } else {
  }
  ROS_INFO("Goal pose = {%f, %f, %d}.", mx, my, int_orientation);
  index = static_cast<unsigned int>(my) * angle_quantizations_ * size_x +
          static_cast<unsigned int>(mx) * angle_quantizations_ +
          int_orientation;
  goal_ = goal;
  goal_node_ = KinematicNode(false, std::numeric_limits<float>::max(), mx, my,
                             int_orientation, MotionDir::NONE, nullptr);
  graph_[index] = goal_node_;

  float wx, wy;
  int cycle = 0;
  float tolerance_mxy_ = tolerance_xy_ / resolution_;
  while (queue_.size() > 0 && cycle < 500000) {
    cycle++;

    QueueElement top = queue_[0];
    std::pop_heap(queue_.begin(), queue_.end(), greater1());
    queue_.pop_back();
    KinematicNode* current_node = &graph_[top.index];
    ROS_DEBUG("Current node{%f, %f, %d}", current_node->mx, current_node->my,
              current_node->mtheta);
    costmap_->mapToWorldLossless(current_node->mx, current_node->my, wx, wy);
    expands.push_back({wx, wy, current_node->mtheta * angle_gap});

    auto normalized_bin = [&](int angle_bin) -> int {
      if (angle_bin < 0)
        return angle_bin + angle_quantizations_;
      else if (angle_bin >= angle_quantizations_)
        return angle_bin - angle_quantizations_;
      else
        return angle_bin;
    };

    //注意，当tolerance设置足够小时，因运动学步长的影响，可能无法如此精确的搜索到路径。
    //需要增加analyExtand逻辑，用于满足更精确的搜索。
    if (analyticExpansion(current_node) ||
        ((hypot(current_node->mx - goal_node_.mx,
                current_node->my - goal_node_.my) < tolerance_mxy_) &&
         (normalized_bin(static_cast<int>(goal_node_.mtheta) -
                         static_cast<int>(current_node->mtheta)) <= 10))) {
      KinematicNode* parent = current_node->parent_node;

      plan.clear();
      if (!analytic_plan_.empty())
        for (auto it = analytic_plan_.rbegin(); it != analytic_plan_.rend();
             ++it)
          plan.push_back(*it);

      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = goal.header.frame_id;
      unsigned int parent_angle, child_angle = current_node->mtheta;
      while (parent != nullptr) {
        parent_angle = parent->mtheta;
        for (auto& primitive :
             motion_primitives_table_->primitives[parent_angle]) {
          if (primitive.end_angle == child_angle) {
            std::vector<geometry_msgs::PoseStamped> tmp_plan;
            for (auto motion_pose : primitive.motion_poses) {
              costmap_->mapToWorldLossless(parent->mx, parent->my, wx, wy);
              pose.pose.position.x = wx + motion_pose.mx * resolution_;
              pose.pose.position.y = wy + motion_pose.my * resolution_;
              pose.pose.orientation = toQuaternion(toAngle(parent->mtheta));
              tmp_plan.push_back(pose);
            }
            for (auto it = tmp_plan.rbegin(); it != tmp_plan.rend(); ++it)
              plan.push_back(*it);

            break;
          }
        }

        child_angle = parent->mtheta;
        parent = parent->parent_node;
      }
      costmap_->mapToWorldLossless(start_node_.mx, start_node_.my, wx, wy);
      pose.pose.position.x = wx;
      pose.pose.position.y = wy;
      pose.pose.orientation = toQuaternion(toAngle(start_node_.mtheta));
      plan.push_back(pose);
      std::reverse(plan.begin(), plan.end());
      ROS_INFO("Find path, size{%d}, cycle{%d}", plan.size(), cycle);
      return true;
    }

    expand(current_node);
  }

  ROS_WARN("Can't to find path, cycle{%d}", cycle);
  return false;
}

void kinematicPlannerCore::expand(KinematicNode* current_node) {
  //注意:查找当前角度是否包含在运动基元表格内，理论上一定是包含的。
  float mx, my, mtheta;
  unsigned int index;

  for (auto& primitive :
       motion_primitives_table_->primitives[current_node->mtheta]) {
    mx = current_node->mx + primitive.motion_poses.back().mx /*/ 0.05*/;
    my = current_node->my + primitive.motion_poses.back().my /*/ 0.05*/;
    mtheta = primitive.end_angle;
    ROS_DEBUG("Neighbor node{%f, %f, %d}", mx, my, mtheta);

    KinematicNode* neighbor_node;
    index = static_cast<unsigned int>(my) * angle_quantizations_ *
                costmap_->getSizeInCellsX() +
            static_cast<unsigned int>(mx) * angle_quantizations_ + mtheta;
    auto it = graph_.find(index);
    if (it != graph_.end())
      neighbor_node = &(it->second);
    else
      neighbor_node = &graph_.emplace(index, KinematicNode()).first->second;

    if (neighbor_node->visited) {
      continue;
      ROS_DEBUG("Neighbor node is visited");
    }

    bool is_collistion = false;
    for (auto& pose : primitive.motion_poses) {
      float mx = current_node->mx + pose.mx /*/ 0.05*/;
      float my = current_node->my + pose.my /*/ 0.05*/;
      float mtheta = primitive.end_angle;  // pose.mtheta; ,,注意这个角度
      // TODO:判读碰撞前是否需要判断该节点是否可达、不可达的节点直接不判断碰撞检测？
      is_collistion =
          collision_checker_->inCollision(  // TODO:碰撞角度是整形还是浮点型
              mx, my, mtheta, false /*traverse_unknown*/);
      if (is_collistion) break;
    }
    if (is_collistion) {
      ROS_DEBUG("Neighbor node is collistion");
      continue;
    }

    unsigned char cost = costmap_->getCost(static_cast<unsigned int>(mx),
                                           static_cast<unsigned int>(my));
    float normalized_cost = static_cast<float>(cost) / 252.0;
    //暂时不考虑后退和原地旋转
    float delta_g = (primitive.motion_length) * (1.0 + normalized_cost * 1.5);
    if (current_node->motion_dir == FRONT_LEFT ||
        current_node->motion_dir == FRONT_RIGHT) {
      if (current_node->motion_dir == primitive.motion_dir)
        delta_g *= 1.1;  // 1.1;
      else
        delta_g *= 1.3;  // 1.3;
    }

    float g = current_node->g + delta_g;
    if (g < neighbor_node->g) {
      neighbor_node->g = g;
      neighbor_node->mx = mx;
      neighbor_node->my = my;
      neighbor_node->mtheta = primitive.end_angle;
      neighbor_node->motion_dir = primitive.motion_dir;
      neighbor_node->parent_node = current_node;
      //考虑死胡同情况
      float h = hypot(goal_node_.mx - neighbor_node->mx,
                      goal_node_.my - neighbor_node->my);
      queue_.push_back(QueueElement(index, g + h));
      std::push_heap(queue_.begin(), queue_.end(), greater1());
    }
    { ROS_DEBUG("Neighbor node is not shorest"); }
  }
}

bool kinematicPlannerCore::analyticExpansion(KinematicNode* current_node) {
  return false;  // TEST
  analytic_plan_.clear();
  float analytic_max_length = 3.0 / resolution_;
  if (hypot(current_node->mx - goal_node_.mx,
            current_node->my - goal_node_.my) > analytic_max_length)
    return false;

  static int iteration_interval = 3;
  static int iteration_num = 3;
  if (iteration_num >= iteration_interval) {
    iteration_num = 0;
    double q0[] = {current_node->mx, current_node->my,
                   toAngle(current_node->mtheta)};
    double q1[] = {goal_node_.mx, goal_node_.my,
                   tf2::getYaw(goal_.pose.orientation)};
    DubinsPath path;
    dubins_shortest_path(&path, q0, q1, min_turning_radius_);
    float d = dubins_path_length(&path);
    unsigned int num_intervals;
    bool is_collistion = false;
    if (d <= analytic_max_length) {
      static const float sqrt_2 = std::sqrt(2.);
      num_intervals = std::floor(d / sqrt_2);

      for (float i = 1; i < num_intervals; i++) {
        double q[3];
        dubins_path_sample(&path, i / num_intervals * d, q);
        is_collistion = collision_checker_->inCollision(
            q[0], q[1], toAngleBin(q[2]), false /*traverse_unknown*/);

        if (is_collistion) break;
      }

      if (!is_collistion) {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = goal_.header.frame_id;
        float wx, wy;
        for (float i = 1; i < num_intervals; i++) {
          double q[3];
          dubins_path_sample(&path, i / num_intervals * d, q);
          costmap_->mapToWorldLossless(q[0], q[1], wx, wy);
          pose.pose.position.x = wx;
          pose.pose.position.y = wy;
          pose.pose.orientation = toQuaternion(q[2]);
          analytic_plan_.push_back(pose);
        }

        ROS_DEBUG("Find analytic expansion!");
        return true;
      }
    }
  } else
    iteration_num++;

  return false;
}

bool kinematicPlannerCore::initMotionPrimitivesTable(
    const std::string& file_path) {
  std::ifstream file(file_path);
  if (!file.is_open()) throw std::runtime_error("Could not open file");

  motion_primitives_table_ = std::make_shared<MotionPrimitiveTable>();
  nlohmann::json json;
  file >> json;

  nlohmann::json json_metadata = json["metadata"];
  angle_quantizations_ = json_metadata["angle_quantizations"];
  min_turning_radius_ = json_metadata["turning_radius"];
  min_turning_radius_ /= 0.01;
  nlohmann::json json_primitives = json["primitives"];
  // MotionPrimitive primitive;
  for (auto& json_primitive : json_primitives) {
    MotionPrimitive primitive;
    primitive.start_angle = json_primitive["start_angle_index"];
    primitive.end_angle = json_primitive["end_angle_index"];
    primitive.motion_dir = json_primitive["motion_dir"];  //待修改
    primitive.motion_length = json_primitive["trajectory_length"];
    MotionPose motion_pose;
    for (auto& pose : json_primitive["motion_poses"]) {
      motion_pose.mx = pose[0]; /*/ 0.05*/
      ;                         // TODO,check this 0.05
      motion_pose.my = pose[1]; /*/ 0.05*/
      ;
      motion_pose.mtheta = pose[2]; /*/ 0.05*/
      ;
      primitive.motion_poses.emplace_back(motion_pose);
    }

    //计算运动方向：后续可以写到文件里
    // float angle2 = atan2(primitive.motion_poses.back().my,
    // primitive.motion_poses.back().mx); float angle1; if(primitive.start_angle
    // <= 8)
    // {
    //   angle1 = M_PI * static_cast<float>(primitive.start_angle) / 8.0;
    // } else {
    //   angle1 = -M_PI * static_cast<float>(16 -primitive.start_angle) / 8.0;
    // }

    // if(fabs(angle1 - angle2) < 0.03)
    // {
    //   primitive.motion_dir = FRONT;
    // } else {
    //   if(angles::shortest_angular_distance(angle1,angle2) > 0.0) {
    //     primitive.motion_dir = FRONT_LEFT;
    //   }
    //   else {
    //     primitive.motion_dir = FRONT_RIGHT;
    //   }
    // }

    // motion_primitives_table_->primitives.insert(std::make_pair<unsigned int,
    // std::vector<MotionPrimitive>>(primitive.start_angle, primitive));
    motion_primitives_table_->primitives[primitive.start_angle].emplace_back(
        primitive);
  }
  return true;
}

float kinematicPlannerCore::toAngle(const unsigned int& angle_bin) {
  float angle_gap = 2 * M_PI / static_cast<float>(angle_quantizations_);
  float angle = angle_bin * angle_gap;
  if (angle > M_PI)
    return angle - 2 * M_PI;
  else
    return angle;
}

unsigned int kinematicPlannerCore::toAngleBin(const float& angle) {
  float angle_gap = 2 * M_PI / static_cast<float>(angle_quantizations_);
  float float_angle_bin = angle / angle_gap;
  int angle_bin = static_cast<int>(floor(float_angle_bin + 0.5));
  if (angle_bin >= static_cast<int>(angle_quantizations_))
    angle_bin -= angle_quantizations_;
  else if (angle_bin < 0) {
    angle_bin += angle_quantizations_;
  } 
  return static_cast<unsigned int>(angle_bin);
}

geometry_msgs::Quaternion kinematicPlannerCore::toQuaternion(const float& theta)
{
  tf2::Quaternion q;
  q.setEuler(0.0, 0.0, theta);
  return tf2::toMsg(q);
}

}  // namespace kinematic_local_planner

