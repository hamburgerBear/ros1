#include "control_algorithm/rotation.h"

namespace control {

Rotation::Rotation(const std::string& name,
                   const DependencyInjector::Ptr& injector)
    : AlgorithmBase(name, injector) {}

Rotation::~Rotation() {}

void Rotation::enter(std::shared_ptr<Args> args) {}

void Rotation::execute() {}

void Rotation::exit() {}

}  // namespace control