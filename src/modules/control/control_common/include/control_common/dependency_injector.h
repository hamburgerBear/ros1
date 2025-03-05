#pragma once

#include <ros/ros.h>

#include <memory>
#include <string>

namespace control {
class DependencyInjector {
 public:
  using Ptr = std::shared_ptr<DependencyInjector>;

  DependencyInjector() = default;
  ~DependencyInjector() = default;

  std::string plugin_name_;
  std::string algorithm_name_;
};

}  // namespace control