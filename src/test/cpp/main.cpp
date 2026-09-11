#include <hal/HAL.h>
#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>

#include "gtest/gtest.h"
#include "TurretConstants.h"

// Test Dynamic Tolerance calculation (OnyxTronix #2231 model)
TEST(TurretTargetingTest, DynamicToleranceScaling) {
  auto calcDynamicTol = [](double dist) {
    double apparentRadiusDeg = std::atan2(TurretConstants::kHubRadiusMeters, std::max(dist, 0.5)) * (180.0 / M_PI);
    return std::clamp(
        apparentRadiusDeg * TurretConstants::kToleranceMarginRatio,
        TurretConstants::kMinToleranceDeg,
        TurretConstants::kMaxToleranceDeg);
  };

  double closeTol = calcDynamicTol(1.5);
  double midTol = calcDynamicTol(3.5);
  double farTol = calcDynamicTol(6.5);

  // Close tolerance should be larger than far tolerance to reduce lock-on latency
  EXPECT_GT(closeTol, midTol);
  EXPECT_GT(midTol, farTol);

  // Ensure bounded by min and max
  EXPECT_LE(closeTol, TurretConstants::kMaxToleranceDeg);
  EXPECT_GE(farTol, TurretConstants::kMinToleranceDeg);
}

// Test Ballistic Table Interpolation
TEST(TurretTargetingTest, BallisticTableInterpolation) {
  const auto& table = TurretConstants::kBallisticTable;

  auto interpRPM = [&](double dist) {
    if (dist <= table.front().distanceMeters) return table.front().shooterRPM;
    if (dist >= table.back().distanceMeters) return table.back().shooterRPM;
    for (size_t i = 0; i < table.size() - 1; ++i) {
      if (dist >= table[i].distanceMeters && dist <= table[i + 1].distanceMeters) {
        double t = (dist - table[i].distanceMeters) / (table[i + 1].distanceMeters - table[i].distanceMeters);
        return table[i].shooterRPM + t * (table[i + 1].shooterRPM - table[i].shooterRPM);
      }
    }
    return table.front().shooterRPM;
  };

  auto interpHood = [&](double dist) {
    if (dist <= table.front().distanceMeters) return table.front().hoodAngleDeg;
    if (dist >= table.back().distanceMeters) return table.back().hoodAngleDeg;
    for (size_t i = 0; i < table.size() - 1; ++i) {
      if (dist >= table[i].distanceMeters && dist <= table[i + 1].distanceMeters) {
        double t = (dist - table[i].distanceMeters) / (table[i + 1].distanceMeters - table[i].distanceMeters);
        return table[i].hoodAngleDeg + t * (table[i + 1].hoodAngleDeg - table[i].hoodAngleDeg);
      }
    }
    return table.front().hoodAngleDeg;
  };

  // Exact boundary points
  EXPECT_DOUBLE_EQ(interpRPM(1.5), 2200.0);
  EXPECT_DOUBLE_EQ(interpHood(1.5), 20.0);

  // Midpoint between 1.5m (2200) and 2.5m (2600) -> 2.0m: 2400 RPM, 24.0 deg hood
  EXPECT_DOUBLE_EQ(interpRPM(2.0), 2400.0);
  EXPECT_DOUBLE_EQ(interpHood(2.0), 24.0);

  // Beyond boundary clamps
  EXPECT_DOUBLE_EQ(interpRPM(1.0), 2200.0);
  EXPECT_DOUBLE_EQ(interpRPM(10.0), 4700.0);
}

// Test Field-to-Robot Relative Angle Math (as in ChiefDelphi & OnyxTronix)
TEST(TurretTargetingTest, OdometryAngleCalculation) {
  double hubX = 10.0;
  double hubY = 5.0;

  // Case 1: Turret at (5.0, 5.0), Robot heading 0 -> Target directly ahead (0 deg)
  double dx1 = hubX - 5.0;
  double dy1 = hubY - 5.0;
  double fieldAngle1 = std::atan2(dy1, dx1) * (180.0 / M_PI);
  double robotRelAngle1 = std::remainder(fieldAngle1 - 0.0, 360.0);
  if (robotRelAngle1 < 0.0) robotRelAngle1 += 360.0;
  EXPECT_NEAR(robotRelAngle1, 0.0, 1e-4);

  // Case 2: Turret at (10.0, 0.0), Robot heading 0 -> Target at 90 deg (left)
  double dx2 = hubX - 10.0;
  double dy2 = hubY - 0.0;
  double fieldAngle2 = std::atan2(dy2, dx2) * (180.0 / M_PI);
  double robotRelAngle2 = std::remainder(fieldAngle2 - 0.0, 360.0);
  if (robotRelAngle2 < 0.0) robotRelAngle2 += 360.0;
  EXPECT_NEAR(robotRelAngle2, 90.0, 1e-4);

  // Case 3: Same position, but robot chassis turned 30 deg -> Target should be 60 deg relative
  double robotRelAngle3 = std::remainder(fieldAngle2 - 30.0, 360.0);
  if (robotRelAngle3 < 0.0) robotRelAngle3 += 360.0;
  EXPECT_NEAR(robotRelAngle3, 60.0, 1e-4);
}

int main(int argc, char** argv) {
  HAL_Initialize(500, 0);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

