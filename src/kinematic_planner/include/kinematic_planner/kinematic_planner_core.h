#ifndef KINEMATIC_PLANNER_CORE_H
#define KINEMATIC_PLANNER_CORE_H

#include <queue>
#include <unordered_map>

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Eigen>
#include "kinematic_planner/collision_checker.h"
#include "kinematic_planner/dubins.h"

#include "nlohmann/json.hpp"

#include <angles/angles.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace kinematic_planner {

//-----------------------------------------------------------运动学基元的方法-----------
enum MotionDir {
  NONE = 0,
  FRONT_LEFT = 1,
  FRONT = 2,
  FRONT_RIGHT = 3,
  BACK_LEFT = 4,
  BACK = 5,
  BACK_RIGHT = 6
};

struct MotionPose {
  float mx;  //格栅坐标系(目前是世界坐标系) - 为了加速搜索邻域节点计算
  float my;  //格栅坐标系(目前是世界坐标系) - 为了加速搜索邻域节点计算
  float mtheta;  //(目前是世界坐标系)，可以不考虑
};

struct MotionPrimitiveMeta {
  float turning_radius;             //最小转弯半径
  float grid_resolution;            //地图的分辨率
  unsigned int angle_quantization;  //角度的量化，hybrid=72 lattice=16
};

// 运动基元表格是不用保存在运动学节点(KinematicNode)属性中的。
struct MotionPrimitive {     //运动基元属性
  unsigned int start_angle;  //开始角度 - 为了快速搜索邻域
  unsigned int end_angle;    //结束角度 - 为了快速搜索邻域
  MotionDir motion_dir;      //运动方向 - 为了辅助计算旅行代价g值
  float motion_length;       //运动距离 - 为了辅助计算旅行代价g值
  std::vector<MotionPose> motion_poses;  //轨迹位置点 - 为了回溯路径
};

struct MotionPrimitiveTable {
  MotionPrimitiveMeta meta;
  std::unordered_map<unsigned int, std::vector<MotionPrimitive>> primitives;
};

struct KinematicGrid {
  float origin_x;
  float origin_y;
  float resolution;
  unsigned int weight;
  unsigned int hight;

  KinematicGrid(float ox, float oy, float res)
      : origin_x(ox), origin_y(oy), resolution(res) {
    weight = fabs(ox) * 2.0 / resolution;
    hight = fabs(oy) * 2.0 / resolution;
  }

  bool worldToMap(float wx, float wy, float& mx, float& my) {
    if (wx < origin_x || wy < origin_y) {
      return false;
    }

    mx = (wx - origin_x) / resolution;
    my = (wy - origin_y) / resolution;

    if (static_cast<unsigned int>(mx) < weight &&
        static_cast<unsigned int>(my) < hight) {
      return true;
    }

    return false;
  }

  void mapToWorld(float mx, float my, float& wx, float& wy) const {
    wx = origin_x + mx * resolution;
    wy = origin_y + my * resolution;
  }

  inline bool outGrid(int mx, int my) {
    if ((mx < 0) || (mx >= weight) || (my < 0) || (my >= hight))
      return true;
    else
      return false;
  }
};

//----------------------------------------------------------运动学节点的属性---------------
struct KinematicNode {
  bool visited;
  float g;
  float mx;
  float my;  //一个高精度的格栅位置
  unsigned int mtheta;
  //节点的格栅朝向,一个高精度的格栅朝向,
  //角度刻度。可快速通过查表的方式进行转换，
  MotionDir motion_dir;
  KinematicNode* parent_node;
  // MotionPrimitive :
  // 1.运动方向(用于计算旅行代价g)，2.运动轨迹(用于回溯轨迹)，3.轨迹长度(用于计算旅行代价g)，开始点角度和结束角度。
  //上一个节点
  //对于lattice，保存运动基元。<会不会很大？，为了可以回溯>，运动方向是否可以保存在运动基元内部

  KinematicNode()
      : visited(false),
        g(std::numeric_limits<float>::max()),
        mx(-1.0),
        my(-1.0),
        mtheta(-1.0),
        motion_dir(MotionDir::NONE),
        parent_node(nullptr) {}

  KinematicNode(bool in_visited, float in_g, float in_mx, float in_my,
                float in_mtheta, MotionDir in_motion_dir,
                KinematicNode* in_parent_node)
      : visited(in_visited),
        g(in_g),
        mx(in_mx),
        my(in_my),
        mtheta(in_mtheta),
        motion_dir(in_motion_dir),
        parent_node(in_parent_node) {}
};

//--------------------------------------------------------开放队列的定义----------------------
class QueueElement {
 public:
  QueueElement(int a, float b) {
    index = a;
    f = b;
  }
  unsigned int index;
  float f;
};

struct greater1 {
  bool operator()(const QueueElement& a, const QueueElement& b) const {
    return a.f > b.f;
  }
};

//------------------------------------------------------图搜索算法框架--------------------------
class kinematicPlannerCore {
 public:
  kinematicPlannerCore(costmap_2d::Costmap2DROS* costmap_ros);
  ~kinematicPlannerCore();

  bool initialize();
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan,
                std::vector<Eigen::Vector3f>& expands);

 private:
  bool initMotionPrimitivesTable(const std::string& file);

  void expand(KinematicNode* current_node);
  bool analyticExpansion(KinematicNode* current_node);

  float toAngle(const unsigned int& angle_bin);
  unsigned int toAngleBin(const float& angle);
  geometry_msgs::Quaternion toQuaternion(const float& theta);

  float tolerance_xy_, tolerance_yaw_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  costmap_2d::Costmap2D* costmap_;
  std::unordered_map<unsigned int, KinematicNode> graph_;
  std::vector<QueueElement> queue_;
  KinematicNode start_node_, goal_node_;
  geometry_msgs::PoseStamped goal_;
  unsigned int angle_quantizations_;
  float min_turning_radius_;
  float resolution_;
  std::string motion_primitives_filepath_;
  std::shared_ptr<MotionPrimitiveTable> motion_primitives_table_;
  std::shared_ptr<GridCollisionChecker> collision_checker_;
  std::vector<geometry_msgs::PoseStamped> analytic_plan_;
  //算法改造:
  //碰撞检测使用点云，避免高精度代价地图更新耗时。第一阶段可以考虑使用2cm精度的代价地图。
  // float kinematic_grid_resolution_;  // 运动格栅精度，可以是2cm
};
}  // namespace kinematic_local_planner
#endif
