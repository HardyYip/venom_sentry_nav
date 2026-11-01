// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
namespace pb_omni_pid_pursuit_controller {
inline std::pair<double,double> planningPoseShift(
  double vx, double vy, double yaw, double age, double horizon) {
  if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(yaw) ||
      !std::isfinite(age) || !std::isfinite(horizon) || horizon < 0.0 || horizon > 0.1) {
    throw std::invalid_argument("invalid planning pose projection");
  }
  const double dt=std::clamp(age,0.0,horizon);
  return {dt*(std::cos(yaw)*vx-std::sin(yaw)*vy),
          dt*(std::sin(yaw)*vx+std::cos(yaw)*vy)};
}
}
