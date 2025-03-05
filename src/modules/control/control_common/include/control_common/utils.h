#pragma once

namespace control {

double sign(double value) {
  if (value > 0.0)
    return 1.0;
  else
    return -1.0;
}

}  // namespace control