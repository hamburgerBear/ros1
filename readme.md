$ catkin_make -j4
$ source devel/setup.bash
$ roslaunch navigation_stage test_move_base_fake_localization_5cm.launch
$ rosrun rqt_reconfigure rqt_reconfigure
rqt_reconfigure : base_global_planner -> spline_planner/SplinePlanner
[.]spline_planner/SplinePlanner
[.]search_planner/SearchPlanner
rviz : 2D Pose Estimate 、2D Nav Goal 、Publish Point 

# 创建global_planner的方式
全局规划器理论上应包含如下5种方法：
1. 示教路径，可保存至本地，通过读取文件的方式获得
2. 几何路径，或样条插值，通过给定始止点和一系列路途点(waypoint)，通过样条插值(如dubins、bezier、cubic spline等)的方法生成一条轨迹
3. 基于搜索，通过A*、Dijkstra、Hybrid A*、Lattice Planner等方法，结合代价地图和给定始止点，搜索出一条轨迹
4. 基于采样，本质上和基于搜索的方式一致，区别在于基于搜索的方法通常使用的是格栅地图，基于搜索的方法通常采用的是[临时随机采样生成的topo地图]。
   提前处理好的topo图，如autoware中的vector_map里的车道线。使用topo规划应该属于基于搜索的规划方法。
5. 基于优化，基于优化的方法需要在已有路径的基础上，路径可由1~4的方法生成，场景的优化方法有：基于共厄梯度下降的EBand、基于g2o优化器的TEB、基于ceres的优化器、基于osqp的优化器等。
    
拷贝navigation/carrot_planner后修改。或参考navigation官方教程-如何创建一个全局规划器
