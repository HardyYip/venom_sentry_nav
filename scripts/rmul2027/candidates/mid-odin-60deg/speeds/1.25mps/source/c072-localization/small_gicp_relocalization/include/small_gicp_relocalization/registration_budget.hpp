// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <stdexcept>
namespace small_gicp_relocalization {
inline int checkedRegistrationBudget(int value) {
  if (value<1 || value>64) {throw std::invalid_argument("registration iteration budget must be 1..64");}
  return value;
}
}
