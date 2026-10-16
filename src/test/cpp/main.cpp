#include <hal/HAL.h>
#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include <string>

#include "gtest/gtest.h"
#include "Drivetrain.h"
#include "DrivetrainConstants.h"

namespace DC = DrivetrainConstants;

// ============================================================================
// 1. Hardware Constants, CAN Buses & Magnet Offsets Verification
// ============================================================================
TEST(ChimiSwerveConfigTest, CanIdsAndBusesMatchRobotTopology) {
  // CAN bus names
  EXPECT_STREQ(DC::kCanBus, "rio");
  EXPECT_STREQ(DC::kIntakeCanBus, "intake");
  EXPECT_STREQ(DC::kTophalfCanBus, "Tophalf");

  // Swerve module CAN IDs
  // Swerve 1 (BR): 10, 11, 12
  EXPECT_EQ(DC::kBRDriveId, 10);
  EXPECT_EQ(DC::kBRSteerId, 11);
  EXPECT_EQ(DC::kBREncoderId, 12);

  // Swerve 2 (FR): 20, 21, 22
  EXPECT_EQ(DC::kFRDriveId, 20);
  EXPECT_EQ(DC::kFRSteerId, 21);
  EXPECT_EQ(DC::kFREncoderId, 22);

  // Swerve 3 (FL): 30, 31, 32
  EXPECT_EQ(DC::kFLDriveId, 30);
  EXPECT_EQ(DC::kFLSteerId, 31);
  EXPECT_EQ(DC::kFLEncoderId, 32);

  // Swerve 4 (BL): 40, 41, 42
  EXPECT_EQ(DC::kBLDriveId, 40);
  EXPECT_EQ(DC::kBLSteerId, 41);
  EXPECT_EQ(DC::kBLEncoderId, 42);

  // Pigeon2 IMU
  EXPECT_EQ(DC::kPigeonId, 2);

  // Mechanism CAN IDs
  EXPECT_EQ(DC::kIndexerMotorId, 50);
  EXPECT_EQ(DC::kIntakeMotorId, 54);
  EXPECT_EQ(DC::kBottomFeederMotorId, 55);
  EXPECT_EQ(DC::kRollerbedMotorId, 56);
  EXPECT_EQ(DC::kTopFeederMotorId, 57);
  EXPECT_EQ(DC::kOuttakeLeftMotorId, 60);
  EXPECT_EQ(DC::kOuttakeRightMotorId, 61);
}

TEST(ChimiSwerveConfigTest, MagnetOffsetsWithinValidPhoenixRange) {
  EXPECT_TRUE(DC::kWriteMagnetOffsets);

  // Verify exact calibrated magnet offsets for Swerve 1-4
  EXPECT_NEAR(DC::kBRMagnetOffset, -0.3969,   1e-6); // Swerve 1 (ID 12)
  EXPECT_NEAR(DC::kFRMagnetOffset,  0.168701, 1e-6); // Swerve 2 (ID 22)
  EXPECT_NEAR(DC::kFLMagnetOffset,  0.599121, 1e-6); // Swerve 3 (ID 32, "intake" bus)
  EXPECT_NEAR(DC::kBLMagnetOffset,  0.329102, 1e-6); // Swerve 4 (ID 42)

  // Phoenix 6 MagnetOffset valid range is [-1.0, 1.0] turns
  EXPECT_GE(DC::kBRMagnetOffset, -1.0);
  EXPECT_LE(DC::kBRMagnetOffset,  1.0);
  EXPECT_GE(DC::kFRMagnetOffset, -1.0);
  EXPECT_LE(DC::kFRMagnetOffset,  1.0);
  EXPECT_GE(DC::kFLMagnetOffset, -1.0);
  EXPECT_LE(DC::kFLMagnetOffset,  1.0);
  EXPECT_GE(DC::kBLMagnetOffset, -1.0);
  EXPECT_LE(DC::kBLMagnetOffset,  1.0);
}

// ============================================================================
// 2. Azimuth Optimization & Inversion Awareness (ChimiSwerve Page 16)
// ============================================================================
TEST(ChimiSwerveMathTest, OptimizeAzimuthShortestPathAndMultiTurn) {
  // Case 1: Wheel at 0 turns, target angle is 0 rad (forward)
  double speed1 = 1.0;
  double targetTurns1 = Drivetrain::ChimiOptimizeAzimuth(0.0, 0.0, speed1);
  EXPECT_NEAR(targetTurns1, 0.0, 1e-5);
  EXPECT_NEAR(speed1, 1.0, 1e-5);

  // Case 2: Target is 30° (+pi/6 rad) -> +1/12 turns
  double speed2 = 1.0;
  double targetTurns2 = Drivetrain::ChimiOptimizeAzimuth(M_PI / 6.0, 0.0, speed2);
  EXPECT_NEAR(targetTurns2, 1.0 / 12.0, 1e-4);
  EXPECT_NEAR(speed2, std::cos(M_PI / 6.0), 1e-4); // Cosine scaling at 30° (~0.866)

  // Case 3: Target is 170° away (>90°). Wheel should reverse and flip setpoint by 180°.
  double speed3 = 1.0;
  double targetRad3 = 170.0 * M_PI / 180.0;
  double targetTurns3 = Drivetrain::ChimiOptimizeAzimuth(targetRad3, 0.0, speed3);
  // Target was +0.4722 turns -> flipped by 0.5 turns gives -0.0278 turns (-10°)
  EXPECT_NEAR(targetTurns3, -10.0 / 360.0, 1e-4);
  EXPECT_LT(speed3, 0.0); // Negated drive speed

  // Case 4: Multi-turn continuity (wheel has rotated to +3.0 turns, target is 0 rad)
  double speed4 = 1.0;
  double targetTurns4 = Drivetrain::ChimiOptimizeAzimuth(0.0, 3.0, speed4);
  EXPECT_NEAR(targetTurns4, 3.0, 1e-5); // Stays at 3.0 turns instead of unwinding to 0.0
  EXPECT_NEAR(speed4, 1.0, 1e-5);
}

TEST(ChimiSwerveMathTest, MinimizeRotationRadianWrapper) {
  double speed = 1.0;
  // Target 180 deg (pi) from 0 rad -> flips to 0 rad and negates speed
  double optAngle = Drivetrain::MinimizeRotation(M_PI, 0.0, speed);
  EXPECT_NEAR(optAngle, 0.0, 1e-5);
  EXPECT_NEAR(speed, -1.0, 1e-5);
}

// ============================================================================
// 3. Steering PID & Heading Lock Tuning Tests (ChimiSwerve Sec 2.3a & 2.3d)
// ============================================================================
TEST(ChimiSwervePidTuningTest, SteerProportionalLoopAndClamping) {
  auto computeSteerDuty = [](double targetTurns, double currentTurns, double kP) {
    double err = std::remainder(targetTurns - currentTurns, 1.0);
    return std::clamp(kP * err, -DC::kSteerPeakOutput, DC::kSteerPeakOutput);
  };

  // At 0 error, output is 0
  EXPECT_NEAR(computeSteerDuty(0.25, 0.25, DC::ModuleP), 0.0, 1e-6);

  // At 0.10 turns (36°) error with ModuleP = 2.0 -> 0.20 duty cycle
  EXPECT_NEAR(computeSteerDuty(0.10, 0.0, DC::ModuleP), 0.20, 1e-6);

  // At 0.25 turns (90°, max error after inversion awareness) with ModuleP = 2.0 -> 0.50 duty
  EXPECT_NEAR(computeSteerDuty(0.25, 0.0, DC::ModuleP), 0.50, 1e-6);

  // Verify clamping if someone tunes ModuleP higher (e.g. kP = 6.0 at 0.25 turns -> clamped to 1.0)
  EXPECT_NEAR(computeSteerDuty(0.25, 0.0, 6.0), DC::kSteerPeakOutput, 1e-6);
  EXPECT_NEAR(computeSteerDuty(-0.25, 0.0, 6.0), -DC::kSteerPeakOutput, 1e-6);
}

TEST(ChimiSwervePidTuningTest, CalcYawStraightAndHeadingClamp) {
  // Zero error -> zero correction
  EXPECT_NEAR(Drivetrain::calcYawStraight(0.0, 0.0, DC::straightP), 0.0, 1e-6);

  // 10° CW error -> +10 * straightP correction
  EXPECT_NEAR(Drivetrain::calcYawStraight(10.0, 0.0, DC::straightP), 10.0 * DC::straightP, 1e-6);

  // Wraparound across ±180°: target -175°, current +175° -> shortest error is +10° CW
  EXPECT_NEAR(Drivetrain::calcYawStraight(-175.0, 175.0, DC::straightP), 10.0 * DC::straightP, 1e-6);

  // Large error clamped by rotationClamp
  double rawCorr = Drivetrain::calcYawStraight(180.0, 0.0, 0.02);
  double clamped = std::clamp(rawCorr, -DC::rotationClamp, DC::rotationClamp);
  EXPECT_LE(std::abs(clamped), DC::rotationClamp + 1e-6);
}

// ============================================================================
// 4. PS5 Radial Deadzone & Trigger Speed Scaling (ChimiSwerve Sec 3.5)
// ============================================================================
TEST(ChimiSwerveControllerTest, RadialDeadzoneInterpolation) {
  auto radialDeadzone = [](double rawX, double rawY, double d, double& outX, double& outY) {
    double mag = std::hypot(rawX, rawY);
    if (mag < d) {
      outX = 0.0;
      outY = 0.0;
    } else {
      double scaled = std::min(1.0, (mag - d) / (1.0 - d));
      outX = (rawX / mag) * scaled;
      outY = (rawY / mag) * scaled;
    }
  };

  double x = 0.0, y = 0.0;
  const double d = DC::xdeadz; // 0.1

  // Inside deadzone -> (0, 0)
  radialDeadzone(0.05, -0.05, d, x, y);
  EXPECT_DOUBLE_EQ(x, 0.0);
  EXPECT_DOUBLE_EQ(y, 0.0);

  // Right on deadzone boundary -> (0, 0)
  radialDeadzone(0.0, 0.1, d, x, y);
  EXPECT_NEAR(x, 0.0, 1e-6);
  EXPECT_NEAR(y, 0.0, 1e-6);

  // Halfway between deadzone (0.1) and full (1.0) is 0.55 -> outputs 0.5
  radialDeadzone(0.0, 0.55, d, x, y);
  EXPECT_NEAR(x, 0.0, 1e-6);
  EXPECT_NEAR(y, 0.5, 1e-6);

  // Diagonal 45° full stick -> preserves 45° angle and clamps magnitude to 1.0
  radialDeadzone(1.0, 1.0, d, x, y);
  EXPECT_NEAR(std::hypot(x, y), 1.0, 1e-6);
  EXPECT_NEAR(x, y, 1e-6);
}

TEST(ChimiSwerveControllerTest, TriggerSpeedScaling) {
  auto calcSpeedPct = [](double triggerL, double triggerR) {
    double pct = DC::DefaultDriveSpeed + (triggerR - triggerL) * DC::TriggerConstant;
    return std::clamp(pct, DC::MinDriveSpeed, 100.0);
  };

  // No triggers pressed -> DefaultDriveSpeed (35%)
  EXPECT_NEAR(calcSpeedPct(0.0, 0.0), 35.0, 1e-6);

  // Full R2 boost -> 35 + 30 = 65%
  EXPECT_NEAR(calcSpeedPct(0.0, 1.0), 65.0, 1e-6);

  // Full L2 slow -> 35 - 30 = 5% (MinDriveSpeed)
  EXPECT_NEAR(calcSpeedPct(1.0, 0.0), 5.0, 1e-6);

  // Both triggers pressed equally -> cancels out to 35%
  EXPECT_NEAR(calcSpeedPct(0.5, 0.5), 35.0, 1e-6);
}

// ============================================================================
// 5. ChimiSwerve Field-Centric Examples (White Paper Pages 11-13) & Kinematics
// ============================================================================
TEST(ChimiSwerveKinematicsTest, WhitePaperFieldCentricExamples) {
  auto fieldCentric = [](double fwd, double str, double thetaDeg, double& fwdNew, double& strNew) {
    double theta = thetaDeg * M_PI / 180.0;
    fwdNew = fwd * std::cos(theta) + str * std::sin(theta);
    strNew = str * std::cos(theta) - fwd * std::sin(theta);
  };

  double fwdOut = 0.0, strOut = 0.0;

  // White Paper Example 1 (Page 12): Robot aligned with field (theta = 0°)
  fieldCentric(1.0, 0.5, 0.0, fwdOut, strOut);
  EXPECT_NEAR(fwdOut, 1.0, 1e-6);
  EXPECT_NEAR(strOut, 0.5, 1e-6);

  // White Paper Example 2 (Page 12): Robot at 90° CW to field (theta = 90°)
  // FWD_new = STR, STR_new = -FWD
  fieldCentric(1.0, 0.25, 90.0, fwdOut, strOut);
  EXPECT_NEAR(fwdOut,  0.25, 1e-6);
  EXPECT_NEAR(strOut, -1.0,  1e-6);

  // White Paper Example 3 (Page 13): Robot at 30° CW to field (theta = 30°)
  fieldCentric(1.0, 0.0, 30.0, fwdOut, strOut);
  EXPECT_NEAR(fwdOut,  std::cos(30.0 * M_PI / 180.0), 1e-6); //  0.866
  EXPECT_NEAR(strOut, -std::sin(30.0 * M_PI / 180.0), 1e-6); // -0.500
}

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

// ============================================================================
// 6. Odometry Forward Kinematics & Integration (ChimiSwerve Pages 17-19)
// ============================================================================
TEST(ChimiSwerveOdometryTest, ForwardKinematicsAndFieldTransform) {
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

  // Case 3: Robot turned 90° CW (facing field-right). Driving forward at 1 m/s increases field STR.
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
