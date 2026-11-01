// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <limits>
#include <vector>
#include "pb_omni_pid_pursuit_controller/planning_clearance.hpp"
#include "pb_omni_pid_pursuit_controller/geodesic_goal_field.hpp"
using namespace pb_omni_pid_pursuit_controller;
TEST(PlanningClearance, DefaultRetainsHardFloorAndRejectsInvalidValues) {
  EXPECT_DOUBLE_EQ(checkedPlanningRadius(kPhysicalSafetyRadius), kPhysicalSafetyRadius);
  EXPECT_DOUBLE_EQ(checkedPlanningRadius(0.5), 0.5);
  for (double value : {kPhysicalSafetyRadius-1e-9, -1.0,
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
    EXPECT_THROW(checkedPlanningRadius(value), std::invalid_argument);
  }
}
TEST(PlanningClearance, RecordedPrepositionWallPoseCannotEnterPlanningMargin) {
  std::vector<unsigned char> cells(100*100,0);
  cells[49*100+59]=254;
  c041_spin::GridView grid{-1.7,-3.55,.05,100,100,cells.data()};
  const double x=.8359164303598413,y=-1.0883206597387445;
  EXPECT_TRUE(c041_spin::circleClear(grid,x,y,kPhysicalSafetyRadius));
  EXPECT_FALSE(c041_spin::circleClear(grid,x,y,checkedPlanningRadius(.5)));
}
TEST(PlanningClearance, GoalFieldAndTrajectoryKernelUseIdenticalConfiguredRadius) {
  std::vector<unsigned char> cells(60*60,0);
  for (int y=10;y<50;++y) {cells[y*60+30]=254;}
  cells[45*60+15]=255;
  c041_spin::GridView grid{0.,0.,.05,60,60,cells.data()};
  const double radius=checkedPlanningRadius(.5);
  goal_field::Potential field;
  ASSERT_TRUE(c041_spin::circleClear(grid,.825,.825,radius));
  ASSERT_TRUE(field.prepare(grid,{.825,.825},{{.825,.825},{2.225,.825}},radius));
  for (int y=0;y<60;++y) {
    for (int x=0;x<60;++x) {
      EXPECT_EQ(field.safeCell(x,y),c041_spin::circleClear(
        grid,(x+.5)*.05,(y+.5)*.05,radius)) << x << "," << y;
    }
  }
}
TEST(PlanningClearance, InvalidProjectedStartDoesNotForceAnUnsafeEscape) {
  std::vector<unsigned char> cells(100*100,0);cells[49*100+59]=254;
  c041_spin::GridView grid{-1.7,-3.55,.05,100,100,cells.data()};
  goal_field::Potential field;
  EXPECT_FALSE(field.prepare(grid,{.8359164303598413,-1.0883206597387445},
    {{.1,-1.1},{.1,-1.8}},checkedPlanningRadius(.5)));
}
