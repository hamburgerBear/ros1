$ catkin_make -j4
$ source devel/setup.bash
$ roslaunch navigation_stage test_move_base_fake_localization_5cm.launch
$ rosrun rqt_reconfigure rqt_reconfigure
rqt_reconfigure : base_global_planner -> spline_planner/SplinePlanner
rviz : 2D Pose Estimate 、2D Nav Goal 、Publish Point 


1.完善三次多项式、五次多项式、三次贝塞尔曲线、五次贝塞尔曲线等。圆弧、直角圆弧、
2.dubins、RS、三次样条终点处的插值逻辑。