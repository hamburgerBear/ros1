#include "control_component/control_component.h"

#include <functional>
namespace control {

ControlComponent::ControlComponent() {
  dependency_injector_ = std::make_shared<DependencyInjector>();

  //注册插件，可以根据.yaml文件
  // plugin_map_["base_controller"] = std::make_shared<BaseController>();
  plugin_map_["follow_wall_controller"] =
      std::make_shared<FollowWallController>();

  //启动控制任务action server
  control_task_ = std::make_unique<ControlTaskServer>(
      ros::NodeHandle(), "control_task",
      boost::bind(&ControlComponent::controlTask, this, _1), false);
  control_task_->start();

  // ros::NodeHandle private_nh("~");

  sub_scan_ = nh_.subscribe("/scan", 1, &ControlComponent::scanCB, this);
  sub_stage_scan_ =
      nh_.subscribe("/base_scan", 1, &ControlComponent::stageScanCB, this);
  sub_pose_ =
      nh_.subscribe("/current_pose", 1, &ControlComponent::poseCB, this);
  sub_stage_pose_ = nh_.subscribe("/base_pose_ground_truth", 1,
                                  &ControlComponent::stagePoseCB, this);
  sub_odom_ = nh_.subscribe("/odom", 1, &ControlComponent::odomCB, this);
  sub_stage_bumper_ =
      nh_.subscribe("/bump_0", 1, &ControlComponent::stageBumperCB, this);
  sub_static_tf_ =
      nh_.subscribe("/tf_static", 1, &ControlComponent::staticTfCB, this);
  pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

ControlComponent::~ControlComponent() {}

void ControlComponent::controlTask(
    const control_interface::ControlTaskGoalConstPtr& goal) {
  ROS_INFO("Task start that plugin_name[%s], algorithm_name[%s].",
           goal->plugin_name.c_str(), goal->algorithm_name.c_str());

  if (plugin_map_.find(goal->plugin_name) == plugin_map_.end()) {
    ROS_WARN("Task aborted, because invaild plugin name[%s].",
             goal->plugin_name.c_str());
    control_task_->setAborted(control_interface::ControlTaskResult(),
                              "Aborted task.");
    return;
  }

  dependency_injector_->plugin_name_ = goal->plugin_name;
  dependency_injector_->algorithm_name_ = goal->algorithm_name;

  auto controller = plugin_map_[goal->plugin_name];
  // 初始化
  // 运行中
  controller->init(goal->plugin_name, dependency_injector_);
  PluginStage stage;
  while (stage.working() && nh_.ok()) {
    stage = controller->run();
    pub_cmd_vel_.publish(injector()->cmd_vel_);
  }

  if (stage.succeeded()) {
    control_task_->setSucceeded(control_interface::ControlTaskResult(),
                                "Succeeded task.");
    ROS_INFO("Task succeeded.");
  } else {
    control_task_->setAborted(control_interface::ControlTaskResult(),
                              "Aborted task.");
    ROS_INFO("Task aborted.");
  }

  return;
}

void ControlComponent::scanCB(const sensor_msgs::LaserScanPtr& msg) {
  // ROS_INFO("Receive scan data.");
  dependency_injector_->scan_ = msg;
}

void ControlComponent::stageScanCB(const sensor_msgs::LaserScanPtr& msg) {
  // ROS_INFO("Receive stage scan data.");
  dependency_injector_->scan_ = msg;
}

void ControlComponent::poseCB(
    const geometry_msgs::PoseWithCovarianceStampedPtr& msg) {
  // ROS_INFO("Receive pose data.");
  dependency_injector_->current_pose_ = msg;
}

void ControlComponent::stagePoseCB(const nav_msgs::OdometryPtr& msg) {
  // ROS_INFO("Receive stage pose data");
  dependency_injector_->current_pose_->header = msg->header;
  dependency_injector_->current_pose_->pose = msg->pose;
}

void ControlComponent::odomCB(const nav_msgs::OdometryPtr& msg) {
  // ROS_INFO("Receive odom data.");
  dependency_injector_->odom_ = msg;
  dependency_injector_->odom_deque_.push_back(msg);
  if (dependency_injector_->odom_deque_.size() >= 10)
    dependency_injector_->odom_deque_.pop_front();
}

void ControlComponent::stageBumperCB(const std_msgs::ByteMultiArrayPtr& msg) {
  // ROS_INFO("Receive stage bumper data.");
  dependency_injector_->bumper_ = msg;
}

void ControlComponent::staticTfCB(const tf2_msgs::TFMessage::ConstPtr& msg) {
  // ROS_INFO("Receive static tf data.");
  dependency_injector_->static_tf_ = msg;
}

std::shared_ptr<DependencyInjector> ControlComponent::injector() const {
  return dependency_injector_;
}

}  // namespace control
