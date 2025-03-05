#include "control_algorithm/arc.h"

namespace control {

Arc::Arc(const std::string& name, const DependencyInjector::Ptr& injector)
    : AlgorithmBase(name, injector) {}

Arc::~Arc() {}

void Arc::enter(std::shared_ptr<Args> args) {}

void Arc::execute() {}

void Arc::exit() {}

}  // namespace control