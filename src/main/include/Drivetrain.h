// ============================================================================
// FRC Team 8753 - 2026 Swerve Drivetrain (ChimiSwerve style, Team 1684)
// ============================================================================
// Everything swerve lives in this one class: hardware, motor configs,
// inverse kinematics, azimuth optimization, heading lock, and odometry.
// Robot.cpp reads the controller and calls Update() every loop.
// See DrivetrainConstants.h for the coordinate convention and bring-up steps.
// ============================================================================

#pragma once

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/DutyCycleOut.hpp>
#include <ctre/phoenix6/controls/PositionDutyCycle.hpp>
#include <units/time.h>

#include "DrivetrainConstants.h"

/// Speeds [0..1] and angles (radians, CW+, 0 = forward) for all four modules.
struct ModuleStates {
  double wsFL = 0, wsFR = 0, wsBL = 0, wsBR = 0;
  double waFL = 0, waFR = 0, waBL = 0, waBR = 0;
};

class Drivetrain {
 public:
  Drivetrain();

  /// Apply drive/steer TalonFX configs (and optionally CANcoder offsets).
  void ConfigureMotors();

  /// Stop drive motors.
  void DisabledInit();

  /// Track live wheel angles so enabling never snaps the wheels; telemetry.
  void DisabledPeriodic();

  /**
   * Main swerve loop (call from TeleopPeriodic / AutonomousPeriodic).
   * @param x         strafe,   +1 = full right
   * @param y         forward,  +1 = full forward
   * @param x2        rotation, +1 = full clockwise
   * @param GyroValue robot heading in degrees, CLOCKWISE positive
   * @param triggerL  0..1 slow down
   * @param triggerR  0..1 speed up
   * @param FieldCentric true = field-centric, false = robot-centric
   */
  void Update(double x, double y, double x2, double GyroValue,
              double triggerL, double triggerR, bool FieldCentric);

  /// Re-latch the heading-lock target (call after resetting the gyro).
  void ResetHeading(double GyroValue) { storedYaw = GyroValue; }

  /// Zero field odometry.
  void ResetOdometry() { positionFWDField = 0.0; positionSTRField = 0.0; }

  void UpdateSim(units::time::second_t dt);

  // ---------- Pure math (static so it can be unit tested) ----------

  /// ChimiSwerve inverse kinematics + wheel speed normalization.
  static ModuleStates InverseKinematics(double FWD, double STR, double RCW);

  /**
   * ChimiSwerve inversion awareness: returns a continuous steer setpoint
   * (sensor turns, CCW+) never more than 0.25 turns from currentTurns.
   * Negates ws if the wheel is flipped, and cosine-scales ws.
   * @param waRad        target angle, radians, CW+ (from kinematics)
   * @param currentTurns current steer position, turns, CCW+ (from CANcoder)
   */
  static double ChimiOptimizeAzimuth(double waRad, double currentTurns, double& ws);

  // ---------- Odometry (meters, field frame, +FWD away from driver, +STR right) ----------
  double GetFieldFWD() const { return positionFWDField; }
  double GetFieldSTR() const { return positionSTRField; }

 private:
  void odometryUpdate(double waFL, double waFR, double waBL, double waBR,
                      double wsFL, double wsFR, double wsBL, double wsBR,
                      double GyroValue);
  void HoldSteer();
  void SetDrive(double fl, double fr, double bl, double br);
  void ReadSteerTurns();
  void PublishTelemetry();

  // ---------- Hardware ----------
  ctre::phoenix6::hardware::TalonFX m_FL_Drive;
  ctre::phoenix6::hardware::TalonFX m_FR_Drive;
  ctre::phoenix6::hardware::TalonFX m_BL_Drive;
  ctre::phoenix6::hardware::TalonFX m_BR_Drive;

  ctre::phoenix6::hardware::TalonFX m_FL_Steer;
  ctre::phoenix6::hardware::TalonFX m_FR_Steer;
  ctre::phoenix6::hardware::TalonFX m_BL_Steer;
  ctre::phoenix6::hardware::TalonFX m_BR_Steer;

  ctre::phoenix6::hardware::CANcoder CANcoderFL;
  ctre::phoenix6::hardware::CANcoder CANcoderFR;
  ctre::phoenix6::hardware::CANcoder CANcoderBL;
  ctre::phoenix6::hardware::CANcoder CANcoderBR;

  ctre::phoenix6::controls::PositionDutyCycle steerRequest{units::angle::turn_t{0}};
  ctre::phoenix6::controls::DutyCycleOut driveRequest{0.0};

  // ---------- Module state ----------
  double turnsFL = 0, turnsFR = 0, turnsBL = 0, turnsBR = 0;  // live steer position (CCW+)
  double lastTargetTurnsFL = 0, lastTargetTurnsFR = 0;
  double lastTargetTurnsBL = 0, lastTargetTurnsBR = 0;
  double cmdFL = 0, cmdFR = 0, cmdBL = 0, cmdBR = 0;          // last drive duty
  bool hasBootstrapped = false;

  double storedYaw = 0.0;  ///< heading-lock target (deg, CW+)

  // ---------- Odometry ----------
  double positionFWDField = 0.0;
  double positionSTRField = 0.0;
  double odoFWD = 0.0;  ///< robot-relative forward velocity (m/s)
  double odoSTR = 0.0;  ///< robot-relative strafe velocity (m/s)
  double odoLastTime = 0.0;
};
