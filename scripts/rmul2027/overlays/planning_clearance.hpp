// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cmath>
#include <stdexcept>
namespace pb_omni_pid_pursuit_controller {
inline constexpr double kPhysicalSafetyRadius = 0.382883794 + 0.05;
inline double checkedPlanningRadius(double radius) {
  if (!std::isfinite(radius) || radius < kPhysicalSafetyRadius) {
    throw std::invalid_argument("planning clearance must retain the physical safety circle");
  }
  return radius;
}
}
