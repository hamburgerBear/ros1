cmake_minimum_required(VERSION 2.8.3)
project(spline_planner)

find_package(catkin REQUIRED
  COMPONENTS
    base_local_planner
    costmap_2d
    nav_core
    pluginlib
    roscpp
    tf2
    tf2_geometry_msgs
    tf2_ros
)
find_package(ompl REQUIRED)

include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    )
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES spline_planner
  CATKIN_DEPENDS
    base_local_planner
    costmap_2d
    nav_core
    pluginlib
    roscpp
    tf2
    tf2_ros
)

add_library(spline_planner src/spline_planner.cpp src/dubins.cpp)
add_dependencies(spline_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(spline_planner
    ${catkin_LIBRARIES}
    ${OMPL_LIBRARIES}
    )

install(TARGETS spline_planner
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(FILES bgp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


