#include <hal/HAL.h>
#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>

#include "gtest/gtest.h"
#include "Drivetrain.h"
#include "DrivetrainConstants.h"

// Test ChimiOptimizeAzimuth (ChimiSwerve Page 16: 90° flip detection and cosine scaling)
TEST(ChimiSwerveMathTest, OptimizeAzimuthShortestPath) {
  // Case 1: Wheel at 0 turns, target angle is 0 rad (forward)
  double speed1 = 1.0;
  double targetTurns1 = Drivetrain::ChimiOptimizeAzimuth(0.0, 0.0, speed1);
  EXPECT_NEAR(targetTurns1, 0.0, 1e-5);
  EXPECT_NEAR(speed1, 1.0, 1e-5);

  // Case 2: Target is 30° (+pi/6 rad) -> +1/12 turns
  double speed2 = 1.0;
  double targetTurns2 = Drivetrain::ChimiOptimizeAzimuth(M_PI / 6.0, 0.0, speed2);
  EXPECT_NEAR(targetTurns2, 1.0 / 12.0, 1e-4);
  EXPECT_GT(speed2, 0.8); // Cosine of small angle remains high

  // Case 3: Target is 170° away (>90°). Wheel should reverse and flip setpoint.
  double speed3 = 1.0;
  double targetRad3 = 170.0 * M_PI / 180.0;
  double targetTurns3 = Drivetrain::ChimiOptimizeAzimuth(targetRad3, 0.0, speed3);
  // Target was +0.472 turns -> flipped by 0.5 turns gives -0.0278 turns
  EXPECT_LT(std::abs(targetTurns3), 0.25);
  EXPECT_LT(speed3, 0.0); // Negated drive speed
}

// Test Rescaled Deadband & Radial Deadzone Interpolation (ChimiSwerve Section 3.5)
TEST(ChimiSwerveMathTest, DeadbandRescaling) {
  auto deadband = [](double val, double d) {
    if (std::abs(val) < d) return 0.0;
    return std::copysign((std::abs(val) - d) / (1.0 - d), val);
  };

  double d = DrivetrainConstants::xdeadz; // 0.1
  EXPECT_DOUBLE_EQ(deadband(0.0, d), 0.0);
  EXPECT_DOUBLE_EQ(deadband(0.05, d), 0.0);
  EXPECT_DOUBLE_EQ(deadband(-0.09, d), 0.0);
  EXPECT_NEAR(deadband(0.1, d), 0.0, 1e-6);
  EXPECT_NEAR(deadband(1.0, d), 1.0, 1e-6);
  EXPECT_NEAR(deadband(-1.0, d), -1.0, 1e-6);
  EXPECT_NEAR(deadband(0.55, d), 0.5, 1e-6);
}

// Test ChimiSwerve Heading Lock (Section 2.3d, Page 34)
TEST(ChimiSwerveMathTest, CalcYawStraight) {
  EXPECT_NEAR(Drivetrain::calcYawStraight(0.0, 0.0, 0.004), 0.0, 1e-6);
  EXPECT_NEAR(Drivetrain::calcYawStraight(10.0, 0.0, 0.004), 0.04, 1e-6);
  // Wraparound across +-180 deg
  EXPECT_NEAR(Drivetrain::calcYawStraight(-175.0, 175.0, 0.004), 0.04, 1e-6);
}

// Test ChimiSwerve Inverse Kinematics (Section 2.1b, Pages 13-15)
TEST(ChimiSwerveKinematicsTest, PureTranslationAndRotation) {
  // 1. Pure forward (FWD = 1, STR = 0, ROT = 0)
  ModuleStates fwd = Drivetrain::InverseKinematics(1.0, 0.0, 0.0);
  EXPECT_NEAR(fwd.wsFL, 1.0, 1e-5);
  EXPECT_NEAR(fwd.wsFR, 1.0, 1e-5);
  EXPECT_NEAR(fwd.wsBL, 1.0, 1e-5);
  EXPECT_NEAR(fwd.wsBR, 1.0, 1e-5);
  EXPECT_NEAR(fwd.waFL, 0.0, 1e-5);
  EXPECT_NEAR(fwd.waFR, 0.0, 1e-5);
  EXPECT_NEAR(fwd.waBL, 0.0, 1e-5);
  EXPECT_NEAR(fwd.waBR, 0.0, 1e-5);

  // 2. Pure strafe right (FWD = 0, STR = 1, ROT = 0)
  ModuleStates str = Drivetrain::InverseKinematics(0.0, 1.0, 0.0);
  EXPECT_NEAR(str.wsFL, 1.0, 1e-5);
  EXPECT_NEAR(str.wsFR, 1.0, 1e-5);
  EXPECT_NEAR(str.wsBL, 1.0, 1e-5);
  EXPECT_NEAR(str.wsBR, 1.0, 1e-5);
  EXPECT_NEAR(str.waFL, M_PI / 2.0, 1e-5); // 90° CW
  EXPECT_NEAR(str.waFR, M_PI / 2.0, 1e-5);
  EXPECT_NEAR(str.waBL, M_PI / 2.0, 1e-5);
  EXPECT_NEAR(str.waBR, M_PI / 2.0, 1e-5);

  // 3. Pure clockwise rotation (FWD = 0, STR = 0, ROT = 1)
  ModuleStates rot = Drivetrain::InverseKinematics(0.0, 0.0, 1.0);
  EXPECT_NEAR(rot.wsFL, 1.0, 1e-5);
  EXPECT_NEAR(rot.wsFR, 1.0, 1e-5);
  EXPECT_NEAR(rot.wsBL, 1.0, 1e-5);
  EXPECT_NEAR(rot.wsBR, 1.0, 1e-5);

  // 4. Combined translation + rotation desaturation (speeds exceed 1.0)
  ModuleStates combo = Drivetrain::InverseKinematics(1.0, 1.0, 1.0);
  EXPECT_LE(combo.wsFL, 1.0 + 1e-5);
  EXPECT_LE(combo.wsFR, 1.0 + 1e-5);
  EXPECT_LE(combo.wsBL, 1.0 + 1e-5);
  EXPECT_LE(combo.wsBR, 1.0 + 1e-5);
  double maxSpeed = std::max({combo.wsFL, combo.wsFR, combo.wsBL, combo.wsBR});
  EXPECT_NEAR(maxSpeed, 1.0, 1e-5); // Normalized to 1.0
}

// Test Odometry Integration Math (ChimiSwerve forward kinematics)
TEST(ChimiSwerveOdometryTest, FieldTransform) {
  double posFWD = 0.0;
  double posSTR = 0.0;
  double dt = 0.02;

  auto stepOdo = [&](double odoFWD, double odoSTR, double gyroDeg) {
    double g = gyroDeg * M_PI / 180.0;
    double fieldFWD = odoFWD * std::cos(g) - odoSTR * std::sin(g);
    double fieldSTR = odoFWD * std::sin(g) + odoSTR * std::cos(g);
    posFWD += fieldFWD * dt;
    posSTR += fieldSTR * dt;
  };

  // Case 1: Drive forward at 2 m/s for 1 second (50 steps) facing 0°
  for (int i = 0; i < 50; ++i) {
    stepOdo(2.0, 0.0, 0.0);
  }
  EXPECT_NEAR(posFWD, 2.0, 1e-4);
  EXPECT_NEAR(posSTR, 0.0, 1e-4);

  // Case 2: Strafe right at 1 m/s for 1 second facing 0°
  for (int i = 0; i < 50; ++i) {
    stepOdo(0.0, 1.0, 0.0);
  }
  EXPECT_NEAR(posFWD, 2.0, 1e-4);
  EXPECT_NEAR(posSTR, 1.0, 1e-4);

  // Case 3: Robot turned 90° CW (facing field-right). Driving forward at 1 m/s should increase field STR.
  for (int i = 0; i < 50; ++i) {
    stepOdo(1.0, 0.0, 90.0);
  }
  EXPECT_NEAR(posFWD, 2.0, 1e-4);
  EXPECT_NEAR(posSTR, 2.0, 1e-4);
}

int main(int argc, char** argv) {
  HAL_Initialize(500, 0);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
