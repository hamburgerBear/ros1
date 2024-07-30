$ catkin_make -j4
$ source devel/setup.bash
$ roslaunch navigation_stage test_move_base_fake_localization_5cm.launch
$ rosrun rqt_reconfigure rqt_reconfigure
rqt_reconfigure : base_global_planner -> spline_planner/SplinePlanner
[.]spline_planner/SplinePlanner
[.]search_planner/SearchPlanner
rviz : 2D Pose Estimate 、2D Nav Goal 、Publish Point 

# 创建global_planner的方式
拷贝navigation/carrot_planner后修改。或参考navigation官方教程-如何创建一个全局规划器