#include "control_algorithm/translation.h"

namespace control {

Translation::Translation(const std::string& name,
                         const DependencyInjector::Ptr& injector)
    : ControlBase(name, injector) {}

Translation::~Translation() {}

void Translation::init(std::shared_ptr<Args> args) {}

void Translation::update() {}

bool Translation::isFinish() { return true; }

bool Translation::isFail() { return true; }

}  // namespace control