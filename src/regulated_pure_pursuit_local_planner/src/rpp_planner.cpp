#include <rpp_local_planner/rpp_planner.h>

PLUGINLIB_EXPORT_CLASS(rpp_local_planner::RPPPlanner,
                       nav_core::BaseLocalPlanner)

namespace rpp_local_planner {

RPPPlanner::RPPPlanner()
    : initialized_(false),
      goal_arrived_(false),
      odom_helper_("odom") /*, setup_(false)*/ {}

RPPPlanner::~RPPPlanner() {}

void RPPPlanner::initialize(std::string name, tf2_ros::Buffer* tf2,
                            costmap_2d::Costmap2DROS* costmap_ros) {
  if (!initialized_) {
    initialized_ = true;
    goal_arrived_ = false;
    tf2_ = tf2;
    costmap_ros_ = costmap_ros;
    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    planner_util_.initialize(tf2_, costmap, costmap_ros_->getGlobalFrameID());

    ros::NodeHandle nh("~/" + name);
    std::string odom_topic;
    if (nh.getParam("odom_topic", odom_topic))
      odom_helper_.setOdomTopic(odom_topic);

    global_plan_pub_ = nh.advertise<nav_msgs::Path>("global_plan", 1);
    local_plan_pub_ = nh.advertise<nav_msgs::Path>("local_plan", 1);
    lookahead_point_pub_ =
        nh.advertise<geometry_msgs::PointStamped>("lookahead_point", 1);

    transitionState(ControlStage::IDLE);
    ROS_INFO("Initialized RPP local planner.");
  }
}

bool RPPPlanner::setPlan(
    const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
  if (!initialized_) {
    ROS_ERROR(
        "This planner has not been initialized, please call initialize() "
        "before using this planner");
    return false;
  }

  if (orig_global_plan.empty()) {
    ROS_WARN("Plan is empty. Aborting local planner!");
    return false;
  }

  if (!tf2_->canTransform(costmap_ros_->getGlobalFrameID(),
                          orig_global_plan.front().header.frame_id,
                          ros::Time(0))) {
    ROS_WARN(
        "Could not transform the global plan to the frame of the controller");
    return false;
  }

  global_plan_.clear();
  global_plan_ = orig_global_plan;
  setAngleBasedOnPositionDerivative(global_plan_);
  planner_util_.setPlan(orig_global_plan);
  base_local_planner::publishPlan(global_plan_, global_plan_pub_);

  // TODO：注意频繁setPlan的情况
  goal_arrived_ = false;
  transitionState(ControlStage::IDLE);
  ROS_INFO("Got new plan");
  return true;
}

bool RPPPlanner::isGoalReached() {
  if (!initialized_) {
    ROS_ERROR(
        "This planner has not been initialized, please call initialize() "
        "before using this planner");
    return false;
  }

  return (goal_arrived_) ? true : false;
}

bool RPPPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
  if (!initialized_) {
    ROS_ERROR(
        "This planner has not been initialized, please call initialize() "
        "before using this planner");
    return false;
  }

  if (!costmap_ros_->getRobotPose(local_pose_)) {
    ROS_ERROR("Could not get robot pose");
    return false;
  }

  // if (!planner_util_.getLocalPlan(local_pose_, local_plan_)) {
  //   ROS_ERROR("Could not get local plan");
  //   return false;
  // }
  if (!tf2_->canTransform(costmap_ros_->getGlobalFrameID(),
                          global_plan_.front().header.frame_id,
                          ros::Time::now())) {
    ROS_ERROR("Could not get local plan");
    return false;
  } else {
    geometry_msgs::TransformStamped transform = tf2_->lookupTransform(
        costmap_ros_->getGlobalFrameID(), global_plan_.front().header.frame_id,
        ros::Time::now());
    local_plan_.resize(global_plan_.size());
    for (unsigned int i = 0; i < local_plan_.size(); i++)
      tf2::doTransform(global_plan_[i], local_plan_[i], transform);
  }
  base_local_planner::publishPlan(local_plan_, local_plan_pub_);

  if (!tf2_->canTransform(costmap_ros_->getBaseFrameID(),
                          costmap_ros_->getGlobalFrameID(), ros::Time::now())) {
    ROS_ERROR("Could not get base plan");
    return false;
  } else {
    geometry_msgs::TransformStamped transform = tf2_->lookupTransform(
        costmap_ros_->getBaseFrameID(), costmap_ros_->getGlobalFrameID(),
        ros::Time::now());
    base_plan_.resize(local_plan_.size());
    tf2::doTransform(local_pose_, base_pose_, transform);
    for (unsigned int i = 0; i < local_plan_.size(); i++)
      tf2::doTransform(local_plan_[i], base_plan_[i], transform);
  }
  setAngleBasedOnPositionDerivative(base_plan_);
  // cmd_vel.linear.x = 0.0;
  // cmd_vel.angular.z = 0.0;
  xy_goal_tolerance_ = 0.2;
  yaw_goal_tolerance_ = 0.2;
  switch (stage_) {
    case ControlStage::IDLE:
      idleStage(cmd_vel);
      break;
    case ControlStage::START:
      startStage(cmd_vel);
      break;
    case ControlStage::TRACK:
      trackStage(cmd_vel);
      break;
    case ControlStage::GOAL:
      goalStage(cmd_vel);
      break;
    case ControlStage::SUCCESS:
      successStage(cmd_vel);
      break;
    case ControlStage::FAIL:
      failStage(cmd_vel);
      break;
    default:
      failStage(cmd_vel);
      break;
  }

  ROS_DEBUG("Computed velocity command: (%f, %f, %f)", cmd_vel.linear.x,
            cmd_vel.linear.y, cmd_vel.angular.z);
  return true;
}

void RPPPlanner::transitionState(const ControlStage& stage) {
  stage_ = stage;
  stage_initialized_ = false;
}

void RPPPlanner::idleStage(geometry_msgs::Twist& cmd_vel) {
  auto init = [=]() {
    ROS_DEBUG("Stage idle init.");
    stage_initialized_ = true;
  };

  if (!stage_initialized_) init();

  transitionState(ControlStage::START);
  cmd_vel.linear.x = 0.0f;
  cmd_vel.angular.z = 0.0f;
}

void RPPPlanner::startStage(geometry_msgs::Twist& cmd_vel) {
  static double desired_angle_of_rotation, rotated_angle, last_angle;
  auto init = [=]() {
    ROS_DEBUG("Stage start init.");
    stage_initialized_ = true;
    double robot_angle = tf2::getYaw(local_pose_.pose.orientation);
    last_angle = robot_angle;
    // int index = findNearstIndex(base_plan_);
    double trajectory_angle =
        tf2::getYaw(local_plan_[0 /*TODO*/].pose.orientation);
    desired_angle_of_rotation =
        angles::shortest_angular_distance(robot_angle, trajectory_angle);
    rotated_angle = 0.0;
    ROS_INFO("Need to ratation = {%f}.", desired_angle_of_rotation);
  };

  auto isFinish = [=]() -> bool {
    double robot_angle = tf2::getYaw(local_pose_.pose.orientation);
    double diff_angle =
        angles::shortest_angular_distance(robot_angle, last_angle);
    last_angle = robot_angle;
    rotated_angle += diff_angle;
    ROS_DEBUG(
        "Start rotating desired_angle_of_rotation = {%f}, rotated_angle = "
        "{%f}.",
        fabs(desired_angle_of_rotation), fabs(rotated_angle));
    if (fabs(rotated_angle) >=
        (fabs(desired_angle_of_rotation) - yaw_goal_tolerance_)) {
      ROS_DEBUG("Stage start is finish.");
      return true;
    }
    return false;
  };

  auto velocityCalculate = [=](geometry_msgs::Twist& cmd_vel) {
    double diff = fabs(desired_angle_of_rotation) - fabs(rotated_angle);
    double desire_vel;
    if (fabs(diff) <= 0.5 /*param:slowly_angular_threshold*/) {
      desire_vel =
          0.02 /*param:min_angular_vel*/ +
          (fabs(diff) / 0.5 /*param:slowly_angular_threshold*/) *
              (0.8 /*param:max_angular_vel*/ - 0.02 /*param:min_angular_vel*/);
    } else
      desire_vel = 0.8 /*param:max_angular_vel*/;

    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = sign(desired_angle_of_rotation) * desire_vel;
  };

  auto velocityConstraint = [=](geometry_msgs::Twist& cmd_vel) {
    //暂不考虑障碍物/安全模块速度约束
    //暂不考虑加速度、减速度约束
    (void)cmd_vel;
    // cmd_vel.linear.x =
    // std::min(cmd_vel.linear.x, 1.0/*param:max_linear_vel*/);
    // cmd_vel.angular.z = std::max(cmd_vel.angular.z,
    // 0.02/*param:min_linear_vel*/);
  };

  if (!stage_initialized_) init();

  if (isFinish()) {
    transitionState(ControlStage::TRACK);
    cmd_vel.linear.x = 0.0f;
    cmd_vel.angular.z = 0.0f;
    return;
  }

  velocityCalculate(cmd_vel);
  velocityConstraint(cmd_vel);
}

void RPPPlanner::trackStage(geometry_msgs::Twist& cmd_vel) {
  auto pose = base_pose_;
  auto init = [=]() {
    ROS_DEBUG("Stage track init.");
    stage_initialized_ = true;
  };

  auto isFinish = [=]() -> bool {
    double dist = calcDistance(local_pose_, local_plan_.back());
    ROS_INFO("Approach Distance = {%f}",
             dist);  //由于坐标系不一致，robot是odom系的？
    if (dist <= xy_goal_tolerance_) {
      // if(local_plan_.size() < 2) {
      ROS_DEBUG("Stage track is finish.");
      return true;
    }
    return false;
  };

  auto isFail = [=]() -> bool {
    //暂不跟踪过程中的失败不考虑机器人卡死、打滑、轨迹偏离
    //暂不考虑轨迹角度偏差过大
    return false;
  };

  auto velocityCalculate = [=](geometry_msgs::Twist& cmd_vel) {
    double lookahead_dist = getLookAheadDistance();
    // auto msg = std_msgs::msg::Float64();
    // msg.data = lookahead_dist;
    // (void)msg;
    // lookahead_dist_pub_->publish(msg);
    // std::cout << "lookahead_dist = " << lookahead_dist << std::endl;
    auto carrot_pose = getLookAheadPoint(lookahead_dist);
    // carrot_pub_->publish(createCarrotMsg(carrot_pose));
    lookahead_point_pub_.publish(carrot_pose);

    // msg.data = calcCTE(local_plan_);
    // cte_pub_->publish(msg);

    const double carrot_dist2 =
        (carrot_pose.pose.position.x * carrot_pose.pose.position.x) +
        (carrot_pose.pose.position.y * carrot_pose.pose.position.y);

    double curvature = 0.0;
    if (carrot_dist2 > 0.001) {
      curvature = 2.0 * carrot_pose.pose.position.y / carrot_dist2;
    }

    // double linear_vel, angular_vel;

    double curvature_vel = cmd_vel.linear.x;
    const double radius = fabs(1.0 / curvature);
    const double min_rad = 2.0;  // param:regulated_linear_scaling_min_radius_;
    if (/*use_regulated_linear_velocity_scaling_*/ true && radius < min_rad)
      curvature_vel *= 1.0 - (fabs(radius - min_rad) / min_rad);
    // msg.data = curvature_vel;
    // curvature_speed_pub_->publish(msg);

    // TODO: check this.
    cmd_vel.linear.x = 0.1; /*parametry*/
    cmd_vel.angular.z = cmd_vel.linear.x * curvature;
    // applyCurvatureVelocityScaling(curvature, cmd_vel.linear.x);
    // applyApproachVelocityScaling(transformed_plan_, cmd_vel.linear.x);
  };

  auto velocityConstraint = [=](geometry_msgs::Twist& cmd_vel) {
    (void)cmd_vel;
    // cmd_vel.linear.x =
    // std::min(cmd_vel.linear.x, 1.0/*param:max_linear_vel*/);
    // cmd_vel.angular.z = std::max(cmd_vel.angular.z,
    // 0.02/*param:min_linear_vel*/);
  };

  if (!stage_initialized_) init();

  if (isFinish()) {
    transitionState(ControlStage::GOAL);
    cmd_vel.linear.x = 0.0f;
    cmd_vel.angular.z = 0.0f;
    return;
  }

  if (isFail()) {
  }

  velocityCalculate(cmd_vel);
  velocityConstraint(cmd_vel);
}

void RPPPlanner::goalStage(geometry_msgs::Twist& cmd_vel) {
  static double desired_angle_of_rotation, rotated_angle, last_angle;
  auto init = [=]() {
    ROS_DEBUG("Stage goal init.");
    stage_initialized_ = true;
    last_angle = tf2::getYaw(local_pose_.pose.orientation);
    double robot_angle = tf2::getYaw(local_pose_.pose.orientation);
    double trajectory_angle = tf2::getYaw(local_plan_.back().pose.orientation);
    desired_angle_of_rotation =
        angles::shortest_angular_distance(robot_angle, trajectory_angle);
    rotated_angle = 0.0;
  };

  auto isFinish = [=]() -> bool {
    double robot_angle = tf2::getYaw(local_pose_.pose.orientation);
    double diff_angle =
        angles::shortest_angular_distance(robot_angle, last_angle);
    last_angle = robot_angle;
    rotated_angle += diff_angle;
    if (rotated_angle >=
        (fabs(desired_angle_of_rotation) - xy_goal_tolerance_)) {
      ROS_DEBUG("Stage goal is finish.");
      transitionState(ControlStage::SUCCESS);
      return true;
    }
    return false;
  };

  auto velocityCalculate = [=](geometry_msgs::Twist& cmd_vel) {
    double diff = fabs(desired_angle_of_rotation) - fabs(rotated_angle);
    double desire_vel;
    if (fabs(diff) <= 0.5 /*param:slowly_angular_threshold*/) {
      desire_vel =
          0.02 /*param:min_angular_vel*/ +
          (fabs(diff) / 0.5 /*param:slowly_angular_threshold*/) *
              (1.0 /*param:max_angular_vel*/ - 0.02 /*param:min_angular_vel*/);
    } else
      desire_vel = 1.0 /*param:max_angular_vel*/;

    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = sign(desired_angle_of_rotation) * desire_vel;
  };

  auto velocityConstraint = [=](geometry_msgs::Twist& cmd_vel) {
    (void)cmd_vel;
    //暂不考虑障碍物/安全模块速度约束
    //暂不考虑加速度、减速度约束
    // cmd_vel.linear.x =
    // std::min(cmd_vel.linear.x, 1.0/*param:max_linear_vel*/);
    // cmd_vel.angular.z = std::max(cmd_vel.angular.z,
    // 0.02/*param:min_linear_vel*/);
  };

  if (!stage_initialized_) init();

  if (isFinish()) {
    transitionState(ControlStage::SUCCESS);
    cmd_vel.linear.x = 0.0f;
    cmd_vel.angular.z = 0.0f;
    return;
  }

  velocityCalculate(cmd_vel);
  velocityConstraint(cmd_vel);
}

void RPPPlanner::successStage(geometry_msgs::Twist& cmd_vel) {
  ROS_DEBUG("Stage is success");
  goal_arrived_ = true;
  cmd_vel.linear.x = 0.0f;
  cmd_vel.angular.z = 0.0f;
}

void RPPPlanner::failStage(geometry_msgs::Twist& cmd_vel) {
  ROS_DEBUG("Stage is fail");
  cmd_vel.linear.x = 0.0f;
  cmd_vel.angular.z = 0.0f;
}

void RPPPlanner::setAngleBasedOnPositionDerivative(
    std::vector<geometry_msgs::PoseStamped>& path) {
  auto set_angle = [=](geometry_msgs::PoseStamped& pose, double angle) {
    tf2::Quaternion q;
    q.setRPY(0, 0, angle);
    tf2::convert(q, pose.pose.orientation);
  };

  double angle = 0.0;
  for (unsigned int i = 1; i < path.size(); i++) {
    double x0 = path[i - 1].pose.position.x, y0 = path[i - 1].pose.position.y,
           x1 = path[i].pose.position.x, y1 = path[i].pose.position.y;
    double angle = atan2(y1 - y0, x1 - x0);
    set_angle(path[i - 1], angle);
  }

  set_angle(path[path.size() - 1], angle);
}

double RPPPlanner::sign(const double& value) const {
  return (value >= 0.0) ? 1.0 : -1.0;
}

// geometry_msgs::msg::Quaternion LatticeController::getQuaternionFromYaw(const
// double& yaw) {
//   tf2::Quaternion q;
//   q.setRPY(0.0, 0.0, yaw);
//   return tf2::toMsg(q);
// }

double RPPPlanner::calcDistance(const geometry_msgs::PoseStamped& p1,
                                const geometry_msgs::PoseStamped& p2) const {
  return hypot(p2.pose.position.x - p1.pose.position.x,
               p2.pose.position.y - p1.pose.position.y);
}

unsigned int RPPPlanner::findNearstIndex(
    const std::vector<geometry_msgs::PoseStamped>& path) const {
  double min_dist = std::numeric_limits<double>::max();
  unsigned int min_idx = 0;

  for (unsigned int i = 0; i < path.size(); ++i) {
    const auto dist = hypot(path[i].pose.position.x, path[i].pose.position.y);
    if (dist < min_dist) {
      min_dist = dist;
      min_idx = i;
    }
  }

  return min_idx;
}

double RPPPlanner::getLookAheadDistance() {
  geometry_msgs::PoseStamped robot_vel;
  odom_helper_.getRobotVel(robot_vel);
  // If using velocity-scaled look ahead distances, find and clamp the dist
  // Else, use the static look ahead distance
  double lookahead_dist = lookahead_dist_;
  if (1 /*use_velocity_scaled_lookahead_dist_*/) {
    lookahead_dist = fabs(robot_vel.pose.position.x) * lookahead_time_;
    lookahead_dist = std::min(lookahead_dist, max_lookahead_dist_);
    lookahead_dist = std::max(lookahead_dist, min_lookahead_dist_);
  }

  return lookahead_dist;
}

// geometry_msgs::PoseStamped
// RPPPlanner::getLookAheadPoint(const double &lookahead_dist) {
//   // Find the first pose which is at a distance greater than the lookahead
//   // distance
//   auto goal_pose_it = std::find_if(
//       base_plan_.begin(), base_plan_.end(),
//       [&](const auto &ps) {
//         return hypot(ps.pose.position.x, ps.pose.position.y) >=
//         lookahead_dist;
//       });

//   // If the no pose is not far enough, take the last pose
//   if (goal_pose_it == base_plan_.end()) {
//     goal_pose_it = std::prev(base_plan_.end());
//   } else if (use_interpolation_ &&
//              goal_pose_it != base_plan_.begin()) {
//     // Find the point on the line segment between the two poses
//     // that is exactly the lookahead distance away from the robot pose (the
//     // origin) This can be found with a closed form for the intersection of a
//     // segment and a circle Because of the way we did the std::find_if,
//     // prev_pose is guaranteed to be inside the circle, and goal_pose is
//     // guaranteed to be outside the circle.
//     auto prev_pose_it = std::prev(goal_pose_it);
//     auto point =
//         circleSegmentIntersection(prev_pose_it->pose.position,
//                                   goal_pose_it->pose.position,
//                                   lookahead_dist);
//     geometry_msgs::PoseStamped pose;
//     pose.header.frame_id = prev_pose_it->header.frame_id;
//     pose.header.stamp = goal_pose_it->header.stamp;
//     pose.pose.position = point;
//     return pose;
//   }

//   return *goal_pose_it;
// }
geometry_msgs::PoseStamped RPPPlanner::getLookAheadPoint(
    const double& lookahead_dist) {
  unsigned int index = findNearstIndex(base_plan_);
  geometry_msgs::PoseStamped pose;
  for (unsigned int i = index; i < base_plan_.size(); i++) {
    pose = base_plan_[i];
    if (hypot(base_plan_[i].pose.position.x, base_plan_[i].pose.position.y) >=
        lookahead_dist)
      break;
  }

  return std::move(pose);
}

geometry_msgs::Point RPPPlanner::circleSegmentIntersection(
    const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, double r) {
  // Formula for intersection of a line with a circle centered at the origin,
  // modified to always return the point that is on the segment between the two
  // points. https://mathworld.wolfram.com/Circle-LineIntersection.html This
  // works because the poses are transformed into the robot frame. This can be
  // derived from solving the system of equations of a line and a circle which
  // results in something that is just a reformulation of the quadratic formula.
  // Interactive illustration in doc/circle-segment-intersection.ipynb as well
  // as at https://www.desmos.com/calculator/td5cwbuocd
  double x1 = p1.x;
  double x2 = p2.x;
  double y1 = p1.y;
  double y2 = p2.y;

  double dx = x2 - x1;
  double dy = y2 - y1;
  double dr2 = dx * dx + dy * dy;
  double D = x1 * y2 - x2 * y1;

  // Augmentation to only return point within segment
  double d1 = x1 * x1 + y1 * y1;
  double d2 = x2 * x2 + y2 * y2;
  double dd = d2 - d1;

  geometry_msgs::Point p;
  double sqrt_term = std::sqrt(r * r * dr2 - D * D);
  p.x = (D * dy + std::copysign(1.0, dd) * dx * sqrt_term) / dr2;
  p.y = (-D * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2;
  return p;
}

};  // namespace rpp_local_planner
