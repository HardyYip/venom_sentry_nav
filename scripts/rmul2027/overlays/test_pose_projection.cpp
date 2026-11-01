// SPDX-License-Identifier: Apache-2.0
#include "pb_omni_pid_pursuit_controller/pose_projection.hpp"
#include <gtest/gtest.h>
#include <limits>
using pb_omni_pid_pursuit_controller::planningPoseShift;
TEST(PlanningProjection, SourceAgeCompensatesOnlyPlanningSeed) {
  const auto delta=planningPoseShift(.3,-.4,0.,.06,.1);
  EXPECT_DOUBLE_EQ(delta.first,.018); EXPECT_DOUBLE_EQ(delta.second,-.024);
}
TEST(PlanningProjection, BodyVelocityRotatesToPoseFrame) {
  const auto delta=planningPoseShift(.5,0.,std::acos(-1.)/2.,.08,.1);
  EXPECT_NEAR(delta.first,0.,1e-12);EXPECT_NEAR(delta.second,.04,1e-12);
}
TEST(PlanningProjection, FutureDisabledAndLongAgeAreBounded) {
  EXPECT_DOUBLE_EQ(planningPoseShift(.5,0.,0.,-.05,.1).first,0.);
  EXPECT_DOUBLE_EQ(planningPoseShift(.5,0.,0.,.06,0.).first,0.);
  EXPECT_DOUBLE_EQ(planningPoseShift(.5,0.,0.,1.,.1).first,.05);
  EXPECT_DOUBLE_EQ(planningPoseShift(0.,0.,1.,.06,.1).first,0.);
}
TEST(PlanningProjection, InvalidInputsFailClosed) {
  const double nan=std::numeric_limits<double>::quiet_NaN(),inf=std::numeric_limits<double>::infinity();
  EXPECT_THROW(planningPoseShift(nan,0.,0.,.1,.1),std::invalid_argument);
  EXPECT_THROW(planningPoseShift(0.,0.,0.,inf,.1),std::invalid_argument);
  EXPECT_THROW(planningPoseShift(0.,0.,0.,.1,.11),std::invalid_argument);
  EXPECT_THROW(planningPoseShift(0.,0.,0.,.1,-.1),std::invalid_argument);
}
