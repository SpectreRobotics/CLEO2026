#include <hal/HAL.h>
#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>

#include "gtest/gtest.h"
#include "DrivetrainConstants.h"

// Test Swerve Angle Wrapping to [-0.5, 0.5) turns
TEST(SwerveMathTest, AngleWrapping) {
  auto wrapTurns = [](double t) {
    return t - std::floor(t + 0.5);
  };

  EXPECT_NEAR(wrapTurns(0.0), 0.0, 1e-6);
  EXPECT_NEAR(wrapTurns(0.25), 0.25, 1e-6);
  EXPECT_NEAR(wrapTurns(0.5), -0.5, 1e-6);
  EXPECT_NEAR(wrapTurns(0.75), -0.25, 1e-6);
  EXPECT_NEAR(wrapTurns(1.0), 0.0, 1e-6);
  EXPECT_NEAR(wrapTurns(-0.25), -0.25, 1e-6);
  EXPECT_NEAR(wrapTurns(-0.75), 0.25, 1e-6);
  EXPECT_NEAR(wrapTurns(1.25), 0.25, 1e-6);
}

// Test Swerve Module Direction Optimization (never turn > 90 deg)
TEST(SwerveMathTest, ModuleOptimization) {
  auto wrapTurns = [](double t) {
    return t - std::floor(t + 0.5);
  };

  auto optimize = [&](double targetTurns, double currentTurns, double& outSpeed) {
    double delta = wrapTurns(targetTurns - currentTurns);
    if (std::abs(delta) > 0.25) {
      targetTurns = wrapTurns(targetTurns + 0.5);
      outSpeed = -outSpeed;
    }
    return targetTurns;
  };

  // Case 1: Wheel at 0, target 30 deg (0.0833 turns) -> no reversal needed
  double speed1 = 1.0;
  double optTarget1 = optimize(30.0 / 360.0, 0.0, speed1);
  EXPECT_NEAR(optTarget1, 30.0 / 360.0, 1e-4);
  EXPECT_DOUBLE_EQ(speed1, 1.0);

  // Case 2: Wheel at 0, target 170 deg (0.4722 turns) -> should flip target by 180 and reverse speed
  double speed2 = 1.0;
  double optTarget2 = optimize(170.0 / 360.0, 0.0, speed2);
  EXPECT_NEAR(optTarget2, -10.0 / 360.0, 1e-4);
  EXPECT_DOUBLE_EQ(speed2, -1.0);

  // Case 3: Wheel at 90 deg (0.25 turns), target -80 deg (-0.222 turns) -> delta is 170 deg -> reverse
  double speed3 = 0.8;
  double optTarget3 = optimize(-80.0 / 360.0, 0.25, speed3);
  EXPECT_NEAR(optTarget3, 100.0 / 360.0, 1e-4);
  EXPECT_DOUBLE_EQ(speed3, -0.8);
}

// Test Rescaled Deadband Function
TEST(SwerveMathTest, DeadbandRescaling) {
  auto deadband = [](double v) {
    double d = DrivetrainConstants::kDeadband; // 0.1
    if (std::abs(v) < d) return 0.0;
    return std::copysign((std::abs(v) - d) / (1.0 - d), v);
  };

  EXPECT_DOUBLE_EQ(deadband(0.0), 0.0);
  EXPECT_DOUBLE_EQ(deadband(0.05), 0.0);
  EXPECT_DOUBLE_EQ(deadband(-0.09), 0.0);
  EXPECT_NEAR(deadband(0.1), 0.0, 1e-6);
  EXPECT_NEAR(deadband(1.0), 1.0, 1e-6);
  EXPECT_NEAR(deadband(-1.0), -1.0, 1e-6);
  EXPECT_NEAR(deadband(0.55), 0.5, 1e-6);
}

// Test Swerve Odometry Integration Math (WPILib NWU Blue Alliance coordinates)
TEST(DrivetrainOdometryTest, WheelVelocityToFieldPositionIntegration) {
  double positionX = 2.0;
  double positionY = 4.0;
  double dt = 0.02; // 50 Hz loop

  auto integrateOdometry = [&](double odoFWD, double odoSTR, double gyroDeg) {
    double gyroRad = gyroDeg * M_PI / 180.0;
    double fieldFWD = odoFWD * std::cos(gyroRad) + odoSTR * std::sin(gyroRad);
    double fieldSTR = -odoFWD * std::sin(gyroRad) + odoSTR * std::cos(gyroRad);
    positionX += fieldFWD * dt;
    positionY -= fieldSTR * dt;
  };

  // Case 1: Drive forward at 2.0 m/s for 1 second (50 steps) at heading 0
  for (int i = 0; i < 50; ++i) {
    integrateOdometry(2.0, 0.0, 0.0);
  }
  EXPECT_NEAR(positionX, 4.0, 1e-4);
  EXPECT_NEAR(positionY, 4.0, 1e-4);

  // Case 2: Strafe right at 1.0 m/s for 1 second (50 steps) at heading 0 (odoSTR > 0 is Right)
  // In WPILib, Right is -Y, so positionY should decrease from 4.0 to 3.0
  for (int i = 0; i < 50; ++i) {
    integrateOdometry(0.0, 1.0, 0.0);
  }
  EXPECT_NEAR(positionX, 4.0, 1e-4);
  EXPECT_NEAR(positionY, 3.0, 1e-4);

  // Case 3: Strafe left at 1.0 m/s for 1 second (50 steps) at heading 0 (odoSTR < 0 is Left)
  for (int i = 0; i < 50; ++i) {
    integrateOdometry(0.0, -1.0, 0.0);
  }
  EXPECT_NEAR(positionX, 4.0, 1e-4);
  EXPECT_NEAR(positionY, 4.0, 1e-4);

  // Case 4: Turned 90 deg CCW (facing Left/+Y) and driving forward at 1.5 m/s for 1 second
  // Robot forward should now translate directly to field +Y (increasing Y)
  for (int i = 0; i < 50; ++i) {
    integrateOdometry(1.5, 0.0, 90.0);
  }
  EXPECT_NEAR(positionX, 4.0, 1e-4);
  EXPECT_NEAR(positionY, 5.5, 1e-4);
}

// Test Swerve Inverse Kinematics Wheel Velocities
TEST(DrivetrainKinematicsTest, PureTranslationAndRotation) {
  namespace DC = DrivetrainConstants;
  double radius = 0.0;
  for (const auto& m : DC::kModules) {
    radius = std::max(radius, std::hypot(m.xMeters, m.yMeters));
  }

  // 1. Pure forward (vx = 1, vy = 0, rotCCW = 0)
  for (int i = 0; i < 4; ++i) {
    const auto& mc = DC::kModules[i];
    double wx = 1.0 - 0.0 * (mc.yMeters / radius);
    double wy = 0.0 + 0.0 * (mc.xMeters / radius);
    double speed = std::hypot(wx, wy);
    double angleTurns = std::atan2(wy, wx) / (2.0 * M_PI);
    EXPECT_NEAR(speed, 1.0, 1e-5);
    EXPECT_NEAR(angleTurns, 0.0, 1e-5); // 0 turns = straight forward
  }

  // 2. Pure CCW rotation (vx = 0, vy = 0, rotCCW = 1)
  // Every wheel speed should be 1.0
  for (int i = 0; i < 4; ++i) {
    const auto& mc = DC::kModules[i];
    double wx = 0.0 - 1.0 * (mc.yMeters / radius);
    double wy = 0.0 + 1.0 * (mc.xMeters / radius);
    double speed = std::hypot(wx, wy);
    EXPECT_NEAR(speed, 1.0, 1e-5);
  }
}

int main(int argc, char** argv) {
  HAL_Initialize(500, 0);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
