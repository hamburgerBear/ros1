#include "control_algorithm/translation.h"

namespace control {

Translation::Translation(const std::string& name,
                         const DependencyInjector::Ptr& injector)
    : AlgorithmBase(name, injector) {}

Translation::~Translation() {}

void Translation::enter(std::shared_ptr<Args> args) {}

void Translation::execute() {
  // is finish

  // is fail

  // running
}

void Translation::exit() {}

// void Translation::setGoal(std::shared_ptr<Args> goal) {}

}  // namespace control