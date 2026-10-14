// ============================================================================
// FRC Team 8753 - 2026 Swerve Drivetrain Constants (ChimiSwerve style)
// ============================================================================
// COORDINATE CONVENTION (ChimiSwerve / Ether swerve derivation):
//   FWD  = +forward            STR = +right (strafe)
//   RCW  = +clockwise rotation (viewed from ABOVE the robot)
//   Gyro = degrees, CLOCKWISE positive (Robot.cpp negates the Pigeon2 yaw,
//          which is CCW positive).
//   Wheel angle from kinematics = radians, clockwise positive, 0 = forward.
//
// Steer sensors (CANcoders) are counter-clockwise positive when viewed from
// above, so Drivetrain negates the kinematic angle before commanding steer.
//
// BRING-UP CHECKLIST (robot on blocks):
//   1. Point every wheel straight forward (bevel gears all on the same side),
//      zero the CANcoders in Phoenix Tuner X (or set kXXMagnetOffset below and
//      set kWriteMagnetOffsets = true). Disabled dashboard
//      "Swerve/XX_CANcoderDeg" must read ~0 for every module.
//   2. Enable teleop, push left stick forward: every wheel must roll forward.
//      If one rolls backward, flip that module's kXXDriveInverted.
//   3. Push the right stick right: robot must spin CLOCKWISE.
//   4. If a steer motor spins forever / oscillates, flip kSteerMotorInverted
//      (steer motor must turn the same way the CANcoder counts).
// ============================================================================

#pragma once

#include <cmath>

namespace DrivetrainConstants {
  // ---------- CAN bus ("rio" = roboRIO, otherwise the CANivore name) ----------
  inline constexpr const char* kCanBus = "rio";

  // ---------- CAN IDs ----------
  inline constexpr int kFLDriveId = 30, kFLSteerId = 31, kFLEncoderId = 32;
  inline constexpr int kFRDriveId = 20, kFRSteerId = 21, kFREncoderId = 22;
  inline constexpr int kBLDriveId = 40, kBLSteerId = 41, kBLEncoderId = 42;
  inline constexpr int kBRDriveId = 10, kBRSteerId = 11, kBREncoderId = 12;
  inline constexpr int kPigeonId = 2;

  // ---------- Robot dimensions (wheel center to wheel center, meters) ----------
  inline constexpr double L = 0.533;                   ///< wheelbase  (front-to-back)
  inline constexpr double W = 0.533;                   ///< trackwidth (left-to-right)
  inline const double R = std::sqrt(L * L + W * W);    ///< diagonal

  // ---------- Inversions ----------
  inline constexpr bool kFLDriveInverted = false;
  inline constexpr bool kFRDriveInverted = true;
  inline constexpr bool kBLDriveInverted = false;
  inline constexpr bool kBRDriveInverted = true;
  inline constexpr bool kSteerMotorInverted = false;

  // ---------- CANcoder magnet offsets (turns) ----------
  // false = leave CANcoder config alone (offsets set in Phoenix Tuner X).
  // true  = write the offsets below to the CANcoders on every boot.
  inline constexpr bool kWriteMagnetOffsets = false;
  inline constexpr double kFLMagnetOffset = 0.0;
  inline constexpr double kFRMagnetOffset = 0.0;
  inline constexpr double kBLMagnetOffset = 0.0;
  inline constexpr double kBRMagnetOffset = 0.0;

  // ---------- Steering (TalonFX onboard PositionDutyCycle, RemoteCANcoder) ----------
  inline constexpr double ModuleP = 2.0;
  inline constexpr double ModuleI = 0.0;
  inline constexpr double ModuleD = 0.0;
  inline constexpr double kSteerPeakOutput = 1.0;
  inline constexpr double SteeringRampRateSeconds = 0.01;
  inline constexpr double kSteerCurrentLimit = 40.0;      ///< A
  inline constexpr double sensorToMechanismRatio = 1.0;   ///< CANcoder turns per wheel turn
  inline constexpr double rotorToSensorRatio = 18.0;      ///< steer motor turns per wheel turn (VERIFY for your module)

  // ---------- Drive (open-loop duty cycle) ----------
  inline constexpr double DriveMotorsHardLimit = 0.75;    ///< absolute duty cap
  inline constexpr double DriveOpenLoopRamp = 0.1;        ///< s
  inline constexpr double kDriveCurrentLimit = 60.0;      ///< A
  inline constexpr double DriveGearRatio = 5.0;           ///< motor turns per wheel turn (VERIFY)
  inline constexpr double WheelDiameter = 0.102;          ///< m (~4 in)

  // ---------- Speed control (percent) ----------
  // Output % = DefaultDriveSpeed + (triggerR - triggerL) * TriggerConstant
  //   no trigger = 35%, full right = 65%, full left = 5%
  inline constexpr double DefaultDriveSpeed = 35.0;
  inline constexpr double TriggerConstant = 30.0;
  inline constexpr double MinDriveSpeed = 5.0;

  // ---------- Joystick deadzones ----------
  inline constexpr double xdeadz = 0.1;
  inline constexpr double ydeadz = 0.1;
  inline constexpr double x2deadz = 0.1;

  // ---------- Straight-drive heading lock ----------
  inline constexpr bool kEnableHeadingLock = true;
  inline constexpr double straightP = 0.02;               ///< RCW per degree of heading error
  inline constexpr double rotationClamp = 0.3;            ///< max heading-lock correction
}
