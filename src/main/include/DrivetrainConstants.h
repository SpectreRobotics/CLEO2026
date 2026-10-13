// ============================================================================
// FRC Team 8753 - 2026 Swerve Drivetrain Constants (from-scratch rewrite)
// ============================================================================
// COORDINATE CONVENTION (used everywhere in the drivetrain code):
//   +X = robot forward, +Y = robot left, +rotation = counter-clockwise (CCW)
//   viewed from ABOVE the robot. Wheel angle 0 = wheel pointing forward,
//   positive wheel angle = wheel rotated CCW (toward the left).
//
// BRING-UP CHECKLIST (do these in order, robot on blocks):
//   1. Enable TEST mode. Steer motors get a small +duty. Dashboard
//      "Swerve/<mod>/SteerPhaseOK" must be TRUE for all four modules.
//      If FALSE for a module, flip that module's steerInverted below.
//   2. Disable. Rotate every wheel by hand so it points straight FORWARD
//      (bevel gears all facing the same side). Read
//      "Swerve/<mod>/RawEncoderTurns" and copy into encoderOffsetTurns.
//   3. Redeploy, enable teleop, push left stick forward slightly.
//      Every wheel must spin so the robot would move forward. If one spins
//      backward, flip that module's driveInverted.
// ============================================================================

#pragma once

struct ModuleConstants {
  const char* name;
  int driveId;
  int steerId;
  int encoderId;
  double xMeters;            ///< Module position, +X forward from robot center
  double yMeters;            ///< Module position, +Y left from robot center
  bool driveInverted;        ///< true -> positive duty drives wheel backward without inversion
  bool steerInverted;        ///< Must make +steer duty turn the wheel CCW (encoder increasing)
  double encoderOffsetTurns; ///< Raw CANcoder reading when wheel points forward
};

namespace DrivetrainConstants {
  // ---------- CAN bus ----------
  // "rio" (or "") = roboRIO built-in CAN. Put your CANivore's name here if the
  // swerve is on a CANivore (if this is wrong, NOTHING on the drivetrain moves).
  inline constexpr const char* kCanBusName = "rio";

  // ---------- Robot geometry (wheel center to wheel center, meters) ----------
  inline constexpr double kTrackLength = 0.533;  // front-to-back
  inline constexpr double kTrackWidth  = 0.533;  // left-to-right

  // ---------- Modules: FL, FR, BL, BR ----------
  // TODO: set encoderOffsetTurns per bring-up step 2.
  inline constexpr ModuleConstants kModules[4] = {
      {"FL", 30, 31, 32, +kTrackLength / 2, +kTrackWidth / 2, false, false, 0.0},
      {"FR", 20, 21, 22, +kTrackLength / 2, -kTrackWidth / 2, false, false, 0.0},
      {"BL", 40, 41, 42, -kTrackLength / 2, +kTrackWidth / 2, false, false, 0.0},
      {"BR", 10, 11, 12, -kTrackLength / 2, -kTrackWidth / 2, false, false, 0.0},
  };

  // ---------- Steering controller (hand-written P-D, runs on roboRIO) ----------
  // Units: duty cycle output per TURN of wheel-angle error.
  inline constexpr double kSteerP = 3.0;         // 0.1 turn (36 deg) error -> 0.30 duty
  inline constexpr double kSteerD = 0.03;        // duty per (turn/sec) of error change
  inline constexpr double kSteerStatic = 0.015;  // min duty to overcome friction
  inline constexpr double kSteerMaxOutput = 0.5; // clamp on steer duty
  inline constexpr double kSteerToleranceTurns = 0.004;  // ~1.4 deg -> output 0
  inline constexpr double kSteerCurrentLimitAmps = 30.0;

  // Runaway protection: if steer output is pushing one way but the encoder moves
  // the other way for this long, the steer motor is disabled (wrong phase).
  inline constexpr double kRunawayTimeSec = 0.3;
  inline constexpr double kRunawayMinVelTurnsPerSec = 0.5;

  // Test-mode phase check duty
  inline constexpr double kSteerTestDuty = 0.08;

  // ---------- Drive ----------
  inline constexpr double kDriveMaxOutput = 0.8;      // absolute duty cap
  inline constexpr double kDriveNormalScale = 0.5;    // no trigger pressed
  inline constexpr double kDriveBoostScale = 1.0;     // right trigger fully pressed
  inline constexpr double kDriveSlowScale = 0.2;      // left trigger fully pressed
  inline constexpr double kDriveRampSec = 0.15;       // open-loop ramp
  inline constexpr double kDriveCurrentLimitAmps = 60.0;

  // ---------- Odometry (verify against your module!) ----------
  inline constexpr double kDriveGearRatio = 5.0;        // motor turns per wheel turn
  inline constexpr double kWheelDiameterMeters = 0.102; // ~4 in

  // ---------- Joystick ----------
  inline constexpr double kDeadband = 0.1;
}
