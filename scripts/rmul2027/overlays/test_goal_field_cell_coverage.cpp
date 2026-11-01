#include <gtest/gtest.h>
#include "pb_omni_pid_pursuit_controller/geodesic_goal_field.hpp"
using pb_omni_pid_pursuit_controller::goal_field::Potential;
using pb_omni_pid_pursuit_controller::goal_field::Point;
struct CellCoverage : ::testing::Test {
 std::vector<unsigned char> data=std::vector<unsigned char>(80*80,0);
 c041_spin::GridView grid{-2.,-2.,.05,80,80,data.data()};
 void obstacle(){data[40*80+40]=254;}
};
TEST_F(CellCoverage, CenterSafeCellCanContainContinuousCollision) {
 obstacle();Potential legacy,covered;
 ASSERT_TRUE(legacy.prepare(grid,{1.,1.},{{1.1,1.1}},.5));
 ASSERT_TRUE(covered.prepare(grid,{1.,1.},{{1.1,1.1}},.5,true));
 EXPECT_TRUE(legacy.safeCell(50,41));
 EXPECT_FALSE(c041_spin::circleClear(grid,.5001,.0501,.5));
 EXPECT_FALSE(covered.safeCell(50,41));
}
TEST_F(CellCoverage, PhysicalSafePoseInsideCellMarginCanAnchorOutward) {
 obstacle();Potential field;
 ASSERT_TRUE(c041_spin::circleClear(grid,.545,.025,.5));
 EXPECT_TRUE(field.prepare(grid,{.545,.025},{{1.1,.025}},.5,true));
 EXPECT_LT(field.score(.65,.025),field.score(.545,.025));
}
TEST_F(CellCoverage, PhysicallyBlockedStartIsNeverReleased) {
 obstacle();Potential field;
 EXPECT_FALSE(c041_spin::circleClear(grid,.515,.025,.5));
 EXPECT_FALSE(field.prepare(grid,{.515,.025},{{1.1,.025}},.5,true));
}
TEST_F(CellCoverage, AllPointsAndEdgesOfAcceptedCellsKeepOriginalCircle) {
 obstacle();data[55*80+20]=255;Potential field;
 ASSERT_TRUE(field.prepare(grid,{1.,1.},{{1.1,1.1}},.5,true));
 int accepted=0;
 for(int y=0;y<80;++y)for(int x=0;x<80;++x)if(field.safeCell(x,y)) {
  ++accepted;
  for(double dy:{0.,.5,1.})for(double dx:{0.,.5,1.})
   ASSERT_TRUE(c041_spin::circleClear(grid,grid.origin_x+(x+dx)*grid.resolution,grid.origin_y+(y+dy)*grid.resolution,.5))<<x<<","<<y;
 }
 EXPECT_GT(accepted,100);
}
TEST_F(CellCoverage, UnknownBarrierCannotBecomeAConnectedRoute) {
 for(int y=0;y<80;++y)data[y*80+40]=255;Potential field;
 EXPECT_FALSE(field.prepare(grid,{-1.,0.},{{1.,0.}},.5,true));
}
TEST_F(CellCoverage, AcceptedCellRequiresConservativeDiagonalReserve) {
 obstacle();Potential field;
 ASSERT_TRUE(field.prepare(grid,{1.,1.},{{1.1,1.1}},.5,true));
 for(int y=0;y<80;++y)for(int x=0;x<80;++x)if(field.safeCell(x,y)) {
  double px=grid.origin_x+(x+.5)*grid.resolution,py=grid.origin_y+(y+.5)*grid.resolution;
  EXPECT_GT(c041_spin::nearestHardDistance(grid,px,py,2.),.5+std::hypot(.05,.05)/2.);
 }
}

TEST_F(CellCoverage, WholeStraightTrajectoryReservesItsMiddleNotOnlyEndpoint) {
 obstacle();Potential field;
 ASSERT_TRUE(field.prepare(grid,{-.30,-.50},{{.45,-.50}},.5,true));
 EXPECT_LT(field.score(.45,-.50),field.unreachableScore());
 for(int i=0;i<=30;++i)ASSERT_TRUE(c041_spin::circleClear(grid,-.30+.75*i/30.,-.50,.5));
 EXPECT_FALSE(pb_omni_pid_pursuit_controller::goal_field::trajectoryReserveClear(grid,{{-.30,-.50},{.45,-.50}},.5));
}
TEST_F(CellCoverage, ReserveStartCanEscapeOutwardWithoutShrinkingPhysicalCircle) {
 obstacle();
 ASSERT_TRUE(c041_spin::circleClear(grid,.545,.025,.5));
 EXPECT_TRUE(pb_omni_pid_pursuit_controller::goal_field::trajectoryReserveClear(grid,{{.545,.025},{.65,.025}},.5));
}
TEST_F(CellCoverage, ReserveStartCannotMoveInwardEvenAbovePhysicalRadius) {
 obstacle();
 ASSERT_TRUE(c041_spin::circleClear(grid,.535,.025,.5));
 EXPECT_FALSE(pb_omni_pid_pursuit_controller::goal_field::trajectoryReserveClear(grid,{{.545,.025},{.535,.025}},.5));
}
TEST_F(CellCoverage, EstablishedReserveCannotBeSpentAfterMovingOutward) {
 obstacle();
 EXPECT_FALSE(pb_omni_pid_pursuit_controller::goal_field::trajectoryReserveClear(grid,{{.545,.025},{.65,.025},{.55,.025}},.5));
}
TEST_F(CellCoverage, UnknownObstacleReceivesSameContinuousReserve) {
 data[40*80+40]=255;
 EXPECT_FALSE(pb_omni_pid_pursuit_controller::goal_field::trajectoryReserveClear(grid,{{-.30,-.50},{.45,-.50}},.5));
 EXPECT_TRUE(pb_omni_pid_pursuit_controller::goal_field::trajectoryReserveClear(grid,{{-.30,-.60},{.45,-.60}},.5));
}
TEST_F(CellCoverage, CollisionBetweenSparseSamplesNeverEscapesPhysicalCheck) {
 obstacle();
 EXPECT_FALSE(pb_omni_pid_pursuit_controller::goal_field::trajectoryReserveClear(grid,{{-.7,.025},{.7,.025}},.5));
 EXPECT_FALSE(pb_omni_pid_pursuit_controller::goal_field::trajectoryReserveClear(grid,{{.515,.025},{.7,.025}},.5));
}
TEST_F(CellCoverage, InvalidTrajectoryAndMapFailClosed) {
 using pb_omni_pid_pursuit_controller::goal_field::trajectoryReserveClear;
 EXPECT_FALSE(trajectoryReserveClear(grid,{},.5));
 EXPECT_FALSE(trajectoryReserveClear(grid,{{std::numeric_limits<double>::quiet_NaN(),0}},.5));
 EXPECT_FALSE(trajectoryReserveClear(grid,{{0,0}},-.5));
 auto invalid=grid;invalid.data=nullptr;EXPECT_FALSE(trajectoryReserveClear(invalid,{{0,0}},.5));
 invalid=grid;invalid.resolution=0;EXPECT_FALSE(trajectoryReserveClear(invalid,{{0,0}},.5));
}
