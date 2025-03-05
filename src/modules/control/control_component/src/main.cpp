#include <ros/ros.h>

#include "control_component/control_component.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "control_component");
  control::ControlComponent component;
  ros::spin();
  return (0);
}
