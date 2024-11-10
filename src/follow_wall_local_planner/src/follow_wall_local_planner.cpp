#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>
#include <follow_wall_local_planner/follow_wall_local_planner.h>

PLUGINLIB_EXPORT_CLASS(follow_wall_local_planner::FollowWallLocalPlanner,
                       nav_core::BaseLocalPlanner)

namespace follow_wall_local_planner {

FollowWallLocalPlanner::FollowWallLocalPlanner()
    : initialized_(false), odom_helper_("odom") /*, setup_(false)*/ {}

FollowWallLocalPlanner::~FollowWallLocalPlanner() {}

void FollowWallLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf2,
                                        costmap_2d::Costmap2DROS* costmap_ros) {
  if (!initialized_) {
    initialized_ = true;
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
    
    data_manager_ptr_ = std::make_shared<DataManager>();
    double speed;
    nh.param("desired_speed", speed, 0.20);
    data_manager_ptr_->desiredSpeedCallback(speed);
    data_manager_ptr_->robotRadiusCallback(costmap_ros_->getLayeredCostmap()->getInscribedRadius()); //TODO:注意padding3cm
    double follow_wall_distance;
    nh.param("follow_wall_distance", follow_wall_distance, 0.05);
    data_manager_ptr_->followWallDistanceCallback(follow_wall_distance);
    laser_sub_ = nh.subscribe("/base_scan_0", 10, &DataManager::scanCallback, &(*data_manager_ptr_));
    linear_laser_sub_ = nh.subscribe("/base_scan_1", 10, &DataManager::linearScanCallback, &(*data_manager_ptr_));

    state_machine_ptr_ = std::make_shared<StateMachine>(data_manager_ptr_/*, route_handler_ptr_*/);
    state_machine_ptr_->init();
    ROS_INFO("Initialized follow wall local planner.");
  }
}

bool FollowWallLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
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
  planner_util_.setPlan(orig_global_plan);
  base_local_planner::publishPlan(global_plan_, global_plan_pub_);
  ROS_INFO("Got new plan");
  return true;
}

bool FollowWallLocalPlanner::isGoalReached() {
  if (!initialized_) {
    ROS_ERROR(
        "This planner has not been initialized, please call initialize() "
        "before using this planner");
    return false;
  }
  return false;
}

bool FollowWallLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {

  if (!initialized_) {
    ROS_ERROR(
        "This planner has not been initialized, please call initialize() "
        "before using this planner");
    return false;
  }

  geometry_msgs::PoseStamped local_pose;
  if (!costmap_ros_->getRobotPose(local_pose)) {
    ROS_ERROR("Could not get robot pose");
    return false;
  }

  // if (!planner_util_.getLocalPlan(local_pose, local_plan_)) {
  //   ROS_ERROR("Could not get local plan");
  //   return false;
  // } 
  // base_local_planner::publishPlan(local_plan_, local_plan_pub_);


  // if (!tf2_->canTransform(costmap_ros_->getBaseFrameID(), costmap_ros_->getGlobalFrameID(), ros::Time::now())) {
  //   ROS_ERROR("Could not get base plan");
  //   return false;
  // } else {
  //   geometry_msgs::TransformStamped transform = tf2_->lookupTransform(costmap_ros_->getBaseFrameID(), costmap_ros_->getGlobalFrameID(), ros::Time::now());
  //   base_plan_.resize(local_plan_.size());
  //   for(unsigned int i = 0; i < local_plan_.size(); i++) 
  //     tf2::doTransform(local_plan_[i], base_plan_[i], transform);
  // }

  data_manager_ptr_->globalPoseCallback(local_pose); //TODO:check this
  state_machine_ptr_->updateState(cmd_vel);
  ROS_DEBUG("Computed velocity command: (%f, %f, %f)", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
  return true;
}

};  // namespace follow_wall_local_planner
