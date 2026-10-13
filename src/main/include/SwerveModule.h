// ============================================================================
// FRC Team 8753 - Swerve Module (from scratch, no WPILib PID / swerve classes)
// ============================================================================
// Steering: hand-written P-D loop on the roboRIO using the CANcoder absolute
// angle directly. Drive: open-loop duty cycle. The motor controllers are used
// ONLY as dumb duty-cycle outputs -> no onboard PID, no remote sensors.
// ============================================================================

#pragma once

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/DutyCycleOut.hpp>

#include "DrivetrainConstants.h"

class SwerveModule {
 public:
  explicit SwerveModule(const ModuleConstants& c);

  /// Apply motor configs. Returns true if every config applied OK.
  bool Configure();

  /// Command the module. speed in [-1, 1] (fraction of max), angleTurns in turns (CCW+).
  void SetDesired(double speed, double angleTurns, double dt);

  /// Keep the wheel at its last angle, drive stopped.
  void Hold(double dt);

  /// Cut all output.
  void Stop();

  /// Test mode: apply a small +steer duty and report whether the encoder agrees.
  void RunPhaseTest();

  /// Wheel angle in turns, wrapped to [-0.5, 0.5), offset applied. CCW+.
  double GetAngleTurns();

  /// Raw CANcoder absolute position (no offset) - use for calibration.
  double GetRawEncoderTurns();

  /// Wheel ground speed in m/s (signed).
  double GetWheelSpeedMps();

  void PublishTelemetry();
  void ResetRunawayLatch() { m_runawayTripped = false; m_runawayTimer = 0.0; }

  /// Simulation: integrate the commanded steer duty into the simulated encoder.
  void UpdateSim(double dt);

  const ModuleConstants& Constants() const { return m_c; }

 private:
  double ComputeSteerOutput(double targetTurns, double dt);
  void ApplySteer(double duty);
  void ApplyDrive(double duty);

  ModuleConstants m_c;
  ctre::phoenix6::hardware::TalonFX m_drive;
  ctre::phoenix6::hardware::TalonFX m_steer;
  ctre::phoenix6::hardware::CANcoder m_encoder;

  ctre::phoenix6::controls::DutyCycleOut m_driveReq{0.0};
  ctre::phoenix6::controls::DutyCycleOut m_steerReq{0.0};

  double m_targetTurns = 0.0;
  bool m_hasTarget = false;
  double m_lastError = 0.0;
  double m_steerDuty = 0.0;
  double m_driveDuty = 0.0;

  double m_runawayTimer = 0.0;
  bool m_runawayTripped = false;
  bool m_phaseOk = false;

  double m_simAngleTurns = 0.0;  // simulation only
};
