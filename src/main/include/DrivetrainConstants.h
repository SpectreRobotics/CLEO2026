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
//   Swerve 1 (BR, ID 12, "rio" bus):    -0.3969
//   Swerve 2 (FR, ID 22, "rio" bus):     0.168701
//   Swerve 3 (FL, ID 32, "intake" bus):  0.599121
//   Swerve 4 (BL, ID 42, "rio" bus):     0.329102
// ============================================================================

#pragma once

#include <cmath>

namespace DrivetrainConstants {
  // ---------- CAN bus names ----------
  //   "rio"       = roboRIO built-in CAN bus (all swerve drive/steer motors, CANcoders 12/22/42, Pigeon2)
  //   "intakebus" = Intake CANivore (Swerve 3 FL CANcoder 32; also IntakeMotor 54, RollerbedMotor 56)
  //   "Tophalf"   = Tophalf CANivore (Feeders 55/57, Indexer 50, Outtake 60/61)
  inline constexpr const char* kCanBus = "rio";
  inline constexpr const char* kIntakeCanBus = "intakebus";
  inline constexpr const char* kTophalfCanBus = "Tophalf";

  // ---------- Swerve Module CAN IDs ----------
  // Swerve 3 (Front Left):  Drive 30 ("rio"), Steer 31 ("rio"), CANcoder 32 ("intakebus")
  // Swerve 2 (Front Right): Drive 20 ("rio"), Steer 21 ("rio"), CANcoder 22 ("rio")
  // Swerve 4 (Back Left):   Drive 40 ("rio"), Steer 41 ("rio"), CANcoder 42 ("rio")
  // Swerve 1 (Back Right):  Drive 10 ("rio"), Steer 11 ("rio"), CANcoder 12 ("rio")
  inline constexpr int kFLDriveId = 30, kFLSteerId = 31, kFLEncoderId = 32;
  inline constexpr int kFRDriveId = 21, kFRSteerId = 20, kFREncoderId = 22;
  inline constexpr int kBLDriveId = 40, kBLSteerId = 41, kBLEncoderId = 42;
  inline constexpr int kBRDriveId = 10, kBRSteerId = 11, kBREncoderId = 12;
  inline constexpr int kPigeonId = 2;

  // ---------- REV Servo Hub (CAN ID 62 on "rio" bus) ----------
  inline constexpr int kServoHubId = 62;
  inline constexpr int kServoChannel2 = 2;
  inline constexpr int kServoChannel3 = 3;

  // ---------- Mechanism Motor CAN IDs ("Tophalf" and "intakebus") ----------
  inline constexpr int kIndexerMotorId = 50;
  inline constexpr int kIntakeMotorId = 54;
  inline constexpr int kBottomFeederMotorId = 55;
  inline constexpr int kRollerbedMotorId = 56;
  inline constexpr int kTopFeederMotorId = 57;
  inline constexpr int kOuttakeLeftMotorId = 60;
  inline constexpr int kOuttakeRightMotorId = 61;

  // ---------- Top Half Mechanism Speeds ----------
  inline constexpr double kOuttakeSpeed = 0.70;
  inline constexpr double kFeederSpeed = 0.70;

  // ---------- Robot dimensions (wheel center to wheel center, meters) ----------
  // ChimiSwerve Section 2.1b (Page 13): L = wheelbase, W = trackwidth, R = sqrt(L^2 + W^2)
  inline constexpr double L = 0.533;                   ///< wheelbase  (front-to-back)
  inline constexpr double W = 0.533;                   ///< trackwidth (left-to-right)
  inline const double R = std::sqrt(L * L + W * W);    ///< diagonal diameter

  // ---------- Motor Inversions (matched to CANcoder magnet offsets) ----------
  inline constexpr bool kFLDriveInverted = false;
  inline constexpr bool kFRDriveInverted = false;
  inline constexpr bool kBLDriveInverted = false;
  inline constexpr bool kBRDriveInverted = true;       ///< BR drive is inverted on CLEO chassis
  inline constexpr bool kSteerMotorInverted = false;   ///< CounterClockwise_Positive

  // ---------- CANcoder magnet offsets (turns) ----------
  // Swerve 1 (BR, ID 12): -0.3969
  // Swerve 2 (FR, ID 22):  0.168701
  // Swerve 3 (FL, ID 32):  0.599121
  // Swerve 4 (BL, ID 42):  0.329102
  inline constexpr bool kWriteMagnetOffsets = true;
  inline constexpr double kBRMagnetOffset = -0.3969;   ///< Swerve 1 (CANcoder 12)
  inline constexpr double kFRMagnetOffset = 0.168701;  ///< Swerve 2 (CANcoder 22)
  inline constexpr double kFLMagnetOffset = 0.599121;  ///< Swerve 3 (CANcoder 32, on "intake" bus)
  inline constexpr double kBLMagnetOffset = 0.329102;  ///< Swerve 4 (CANcoder 42)

  // ---------- Steering (TalonFX Slot 0 PositionDutyCycle + CANcoder Deadband Buffer) ----------
  inline constexpr double ModuleP = 2.0;
  inline constexpr double ModuleI = 0.0;
  inline constexpr double ModuleD = 0.0;
  inline constexpr double kSteerPeakOutput = 1.0;
  inline constexpr double SteeringRampRateSeconds = 0.01;
  inline constexpr double kSteerCurrentLimit = 40.0;      ///< A (ChimiSwerve Sec 3.6 brownout protection)
  inline constexpr double kSteerDeadbandTurns = 0.012;    ///< 0.012 turns (4.3 deg) deadband to prevent wobble while keeping modules tightly synchronized
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
  inline constexpr bool kEnableHeadingLock = false;       ///< disabled by default so gyro never fights stick translation
  inline constexpr double straightP = 0.004;              ///< ChimiSwerve calcYawStraight kP (0.004 / deg)
  inline constexpr double straightI = 0.0;
  inline constexpr double straightD = 0.0;
  inline constexpr double rotationClamp = 0.3;            ///< max heading-lock correction
}
