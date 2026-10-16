// ============================================================================
// FRC Team 8753 - 2026 Swerve Drivetrain Constants (ChimiSwerve White Paper)
// ============================================================================
// COORDINATE CONVENTION (ChimiSwerve / Ether swerve derivation):
//   FWD  = +forward            STR = +right (strafe)
//   ROT  = +clockwise rotation (viewed from ABOVE the robot)
//   Gyro = degrees, CLOCKWISE positive (Robot.cpp negates Pigeon2 yaw,
//          which is CCW positive).
//   Wheel angle (wa) = radians, clockwise positive, 0 = straight ahead.
//
// CANCODER MAGNET OFFSETS:
//   kWriteMagnetOffsets = false -> uses the current magnet offsets already
//   stored in the CANcoders' flash memory on the robot (never overwrites them).
// ============================================================================

#pragma once

#include <cmath>

namespace DrivetrainConstants {
  // ---------- CAN bus names ----------
  //   "rio"     = roboRIO built-in CAN bus (all swerve modules + Pigeon2)
  //   "intake"  = Intake CANivore (IntakeMotor + RollerbedMotor)
  //   "Tophalf" = Tophalf CANivore (Feeders, Indexer, Outtake)
  inline constexpr const char* kCanBus = "rio";
  inline constexpr const char* kIntakeCanBus = "intake";
  inline constexpr const char* kTophalfCanBus = "Tophalf";

  // ---------- Swerve Module CAN IDs ----------
  // Module 3 (Front Left):  Drive 30, Steer 31, CANcoder 32
  // Module 2 (Front Right): Drive 20, Steer 21, CANcoder 22
  // Module 4 (Back Left):   Drive 40, Steer 41, CANcoder 42
  // Module 1 (Back Right):  Drive 10, Steer 11, CANcoder 12
  inline constexpr int kFLDriveId = 30, kFLSteerId = 31, kFLEncoderId = 32;
  inline constexpr int kFRDriveId = 20, kFRSteerId = 21, kFREncoderId = 22;
  inline constexpr int kBLDriveId = 40, kBLSteerId = 41, kBLEncoderId = 42;
  inline constexpr int kBRDriveId = 10, kBRSteerId = 11, kBREncoderId = 12;
  inline constexpr int kPigeonId = 2;

  // ---------- Mechanism Motor CAN IDs ----------
  inline constexpr int kIndexerMotorId = 50;
  inline constexpr int kIntakeMotorId = 54;
  inline constexpr int kBottomFeederMotorId = 55;
  inline constexpr int kRollerbedMotorId = 56;
  inline constexpr int kTopFeederMotorId = 57;
  inline constexpr int kOuttakeLeftMotorId = 60;
  inline constexpr int kOuttakeRightMotorId = 61;

  // ---------- Robot dimensions (wheel center to wheel center, meters) ----------
  // ChimiSwerve Section 2.1b (Page 13): L = wheelbase, W = trackwidth, R = sqrt(L^2 + W^2)
  inline constexpr double L = 0.533;                   ///< wheelbase  (front-to-back)
  inline constexpr double W = 0.533;                   ///< trackwidth (left-to-right)
  inline const double R = std::sqrt(L * L + W * W);    ///< diagonal diameter

  // ---------- Motor Inversions (matched to current CANcoder magnet offsets) ----------
  inline constexpr bool kFLDriveInverted = false;
  inline constexpr bool kFRDriveInverted = false;
  inline constexpr bool kBLDriveInverted = false;
  inline constexpr bool kBRDriveInverted = true;       ///< BR drive is inverted on CLEO chassis
  inline constexpr bool kSteerMotorInverted = false;   ///< CounterClockwise_Positive

  // ---------- CANcoder magnet offsets (turns) ----------
  // false = use the current magnet offsets already stored on the CANcoders.
  inline constexpr bool kWriteMagnetOffsets = false;
  inline constexpr double kFLMagnetOffset = 0.0;
  inline constexpr double kFRMagnetOffset = 0.0;
  inline constexpr double kBLMagnetOffset = 0.0;
  inline constexpr double kBRMagnetOffset = 0.0;

  // ---------- Steering (TalonFX Slot 0 PositionDutyCycle + RemoteCANcoder) ----------
  inline constexpr double ModuleP = 2.0;
  inline constexpr double ModuleI = 0.0;
  inline constexpr double ModuleD = 0.0;
  inline constexpr double kSteerPeakOutput = 1.0;
  inline constexpr double SteeringRampRateSeconds = 0.01;
  inline constexpr double kSteerCurrentLimit = 40.0;      ///< A (ChimiSwerve Sec 3.6 brownout protection)
  inline constexpr double sensorToMechanismRatio = 1.0;   ///< CANcoder turns per wheel turn
  inline constexpr double rotorToSensorRatio = 18.0;      ///< steer motor turns per CANcoder turn (18:1)

  // ---------- Drive (DutyCycle + Ramp + Current Limiting) ----------
  inline constexpr double DriveMotorsHardLimit = 0.75;    ///< max duty cycle cap
  inline constexpr double DriveRampRateSeconds = 1.1;     ///< closed-loop voltage ramp (s)
  inline constexpr double DriveOpenLoopRamp = 0.15;       ///< open-loop duty ramp (s, ChimiSwerve Sec 3.6)
  inline constexpr double kDriveCurrentLimit = 60.0;      ///< A (ChimiSwerve Sec 3.6 brownout protection)
  inline constexpr double DriveGearRatio = 5.0;           ///< motor turns per wheel turn (5:1)
  inline constexpr double WheelDiameter = 0.102;          ///< m (~4 in wheel diameter)
  inline constexpr double WheelCircumference = 0.102;     ///< alias used in odometry formula (multiplied by pi)

  // ---------- Speed control (percent) ----------
  // Output % = DefaultDriveSpeed + (triggerR - triggerL) * TriggerConstant
  //   no trigger = 35%, full R2 = 65%, full L2 = 5%
  inline constexpr double DefaultDriveSpeed = 35.0;
  inline constexpr double TriggerConstant = 30.0;
  inline constexpr double MinDriveSpeed = 5.0;

  // ---------- PS5 Controller Deadzones (ChimiSwerve Sec 3.5) ----------
  inline constexpr double xdeadz = 0.1;
  inline constexpr double ydeadz = 0.1;
  inline constexpr double x2deadz = 0.1;

  // ---------- Straight-drive heading lock (ChimiSwerve Sec 2.3d, Pages 34-35) ----------
  inline constexpr bool kEnableHeadingLock = true;
  inline constexpr double straightP = 0.004;              ///< ChimiSwerve calcYawStraight kP (0.004 / deg)
  inline constexpr double straightI = 0.0;
  inline constexpr double straightD = 0.0;
  inline constexpr double rotationClamp = 0.3;            ///< max heading-lock correction
}
