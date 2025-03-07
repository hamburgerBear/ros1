#include "control_algorithm/arc.h"

namespace control {

Arc::Arc(const std::string& name, const DependencyInjector::Ptr& injector)
    : ControlBase(name, injector) {}

Arc::~Arc() {}

void Arc::init(std::shared_ptr<Args> args) {}

void Arc::update() {}

bool Arc::isFinish() { return true; }

bool Arc::isFail() { return true; }

}  // namespace control