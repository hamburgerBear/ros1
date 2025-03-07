#include "control_algorithm/rotation.h"

namespace control {

Rotation::Rotation(const std::string& name,
                   const DependencyInjector::Ptr& injector)
    : ControlBase(name, injector) {}

Rotation::~Rotation() {}

void Rotation::init(std::shared_ptr<Args> args) {}

void Rotation::update() {}

bool Rotation::isFinish() { return true; }

bool Rotation::isFail() { return true; }

}  // namespace control