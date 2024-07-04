$ catkin_make -j4
$ source devel/setup.bash
$ roslaunch navigation_stage test_move_base_fake_localization_5cm.launch
$ rosrun rqt_reconfigure rqt_reconfigure
rqt_reconfigure : base_global_planner -> spline_planner/SplinePlanner
rviz : 2D Pose Estimate 、2D Nav Goal 、Publish Point 