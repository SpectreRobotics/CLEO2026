// ============================================================================
// FRC Team 8753 - 2026 Swerve Drivetrain (ChimiSwerve White Paper, Team 1684)
// ============================================================================
// Implements:
//   - Section 2.1b: Field-Centric Transformation, Inverse Kinematics (A,B,C,D),
//                   Wheel Speed Normalization, Azimuth & Inversion Awareness
//   - Section 2.2b: Forward Kinematics (FK) & Field Odometry Integration
//   - Section 2.3d: Straight-Drive Heading Lock (calcYawStraight / storedYaw)
//   - Section 3.1:  Zero Azimuth Position & Pre-Match Module Testing
//   - Section 3.6:  Brownout Protection (Ramp Rates & Current Limiting)
// ============================================================================

#pragma once

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/DutyCycleOut.hpp>
#include <ctre/phoenix6/controls/PositionDutyCycle.hpp>
#include <frc/geometry/Rotation2d.h>
#include <units/length.h>
#include <units/time.h>

#include "DrivetrainConstants.h"

/// Speeds [0..1] and azimuth angles (radians, CW+, 0 = straight ahead) for all four modules.
struct ModuleStates {
  double wsFL = 0, wsFR = 0, wsBL = 0, wsBR = 0;
  double waFL = 0, waFR = 0, waBL = 0, waBR = 0;
};

class Drivetrain {
 public:
  Drivetrain();

  /// Apply drive/steer TalonFX configs while preserving current CANcoder magnet offsets.
  void ConfigureMotors();
  bool AreConfigsApplied() const { return m_configsApplied; }

  /// Stop drive motors when entering Disabled.
  void DisabledInit();

  /// Track live wheel angles so enabling never snaps the wheels; retry configs if needed.
  void DisabledPeriodic();

  /**
   * Main ChimiSwerve control loop (call from TeleopPeriodic / AutonomousPeriodic).
   * @param x            STR (strafe),   +1.0 = full right
   * @param y            FWD (forward),  +1.0 = full forward
   * @param x2           ROT (rotation), +1.0 = full clockwise
   * @param GyroValue    robot heading in degrees, CLOCKWISE positive
   * @param triggerL     0..1 slow down (L2 on PS5 controller)
   * @param triggerR     0..1 speed boost (R2 on PS5 controller)
   * @param FieldCentric true = field-centric, false = robot-centric
   */
  void Update(double x, double y, double x2, double GyroValue,
              double triggerL, double triggerR, bool FieldCentric);

  /// ChimiSwerve Section 3.1: Command all wheels to 0 deg (straight forward) in Test Mode.
  void TestZeroAzimuth();

  /// ChimiSwerve Section 3.1: Slowly rotate all wheels in Test Mode to verify smooth steering.
  void TestRotateModules(double dt);

  /// Re-latch the heading-lock target (call after resetting the gyro).
  void ResetHeading(double GyroValue) {
    storedYaw = GyroValue;
    m_headingIError = 0.0;
    m_headingLastError = 0.0;
  }

  /// Zero field odometry (ChimiSwerve Section 2.2b Page 20).
  void ResetOdometry() {
    positionFWDField = 0.0;
    positionSTRField = 0.0;
    odoFWD = 0.0;
    odoSTR = 0.0;
    odoROT = 0.0;
  }

  void UpdateSim(units::time::second_t dt);

  // ---------- Pure ChimiSwerve Math Helpers (static for unit testing) ----------

  /// ChimiSwerve Section 2.1b (Pages 13-15): Inverse kinematics + wheel speed normalization.
  static ModuleStates InverseKinematics(double FWD, double STR, double ROT);

  /**
   * ChimiSwerve Section 2.1b (Page 16): Inversion awareness + cosine scaling.
   * Returns a continuous steer setpoint (turns) never more than 0.25 turns (90 deg)
   * from currentTurns, inverting ws when flipped by 180 deg.
   */
  static double ChimiOptimizeAzimuth(double waRad, double currentTurns, double& ws);

  /// Radian wrapper around ChimiSwerve inversion awareness (compatible with legacy callers).
  static double MinimizeRotation(double targetAngleRad, double currentAngleRad, double& speedPercent);

  /// ChimiSwerve Section 2.3d (Page 34): Proportional heading-lock correction.
  static double calcYawStraight(double targetAngle, double currentAngle, double kP);

  // ---------- Odometry (meters, field frame, +FWD away from driver, +STR right) ----------
  double GetFieldFWD() const { return positionFWDField; }
  double GetFieldSTR() const { return positionSTRField; }

  // ---------- Hardware (Current CAN IDs from DrivetrainConstants) ----------
  // Module 3 (FL): 30/31/32 | Module 2 (FR): 20/21/22 | Module 4 (BL): 40/41/42 | Module 1 (BR): 10/11/12
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

  // Public odometry state
  double positionFWDField = 0.0;
  double positionSTRField = 0.0;
  double odoFWD = 0.0;  ///< robot-relative forward velocity (m/s)
  double odoSTR = 0.0;  ///< robot-relative strafe velocity (m/s)
  double odoROT = 0.0;  ///< rotational velocity (rad/s)

 private:
  void odometryUpdate(double waFL, double waFR, double waBL, double waBR,
                      double wsFL, double wsFR, double wsBL, double wsBR,
                      double GyroValue);
  void HoldSteer();
  void StopSteer();
  void SyncFLSteerToCANcoder();
  void SetDrive(double fl, double fr, double bl, double br);
  void ReadSteerTurns();
  void PublishTelemetry();

  ctre::phoenix6::controls::PositionDutyCycle steerRequest{units::angle::turn_t{0}};
  ctre::phoenix6::controls::DutyCycleOut steerDutyRequest{0.0};
  ctre::phoenix6::controls::DutyCycleOut driveRequest{0.0};

  // ---------- Module state ----------
  double turnsFL = 0, turnsFR = 0, turnsBL = 0, turnsBR = 0;  ///< live fused steer position (turns)
  double lastTargetTurnsFL = 0, lastTargetTurnsFR = 0;
  double lastTargetTurnsBL = 0, lastTargetTurnsBR = 0;
  double cmdFL = 0, cmdFR = 0, cmdBL = 0, cmdBR = 0;          ///< last drive duty cycle
  bool hasBootstrapped = false;
  bool m_configsApplied = false;

  // ---------- Heading lock state (ChimiSwerve Sec 2.3d) ----------
  double storedYaw = 0.0;  ///< target heading (deg, CW+)
  double m_headingIError = 0.0;
  double m_headingLastError = 0.0;
  double m_lastLoopTime = 0.0;

  // ---------- Odometry & Test state ----------
  double odoLastTime = 0.0;
  double m_testRotateTurns = 0.0;
};
