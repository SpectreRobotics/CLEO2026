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

  /// Configure drive and steer motors (sensor fusion, ramps, PID, and CANcoder magnet offsets).
  void ConfigureMotors();
  bool AreConfigsApplied() const { return m_configsApplied; }

  /// Stop drive motors when entering Disabled.
  void DisabledInit();

  /// Track live wheel angles and sync FL steer in Disabled so enabling never snaps the wheels.
  void DisabledPeriodic();

  /**
   * Main swerve drive control loop (call from TeleopPeriodic / AutonomousPeriodic).
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

  /// Test Mode helper: command all modules to 0 deg (straight ahead).
  void TestZeroAzimuth();

  /// Test Mode helper: slowly rotate all modules in a circle.
  void TestRotateModules(double dt);

  /// Re-latch the heading-lock target (call after resetting gyro).
  void ResetHeading(double GyroValue) {
    storedYaw = GyroValue;
    m_headingIError = 0.0;
    m_headingLastError = 0.0;
  }

  /// Zero field odometry.
  void ResetOdometry() {
    positionFWDField = units::length::meter_t(0);
    positionSTRField = units::length::meter_t(0);
    odoFWD = 0.0;
    odoSTR = 0.0;
    odoROT = 0.0;
  }

  void UpdateSim(units::time::second_t dt);

  // ---------- Math Helpers (public & static for unit tests) ----------
  static ModuleStates InverseKinematics(double FWD, double STR, double ROT);
  static double ChimiOptimizeAzimuth(double waRad, double currentTurns, double& ws);
  static double MinimizeRotation(double targetAngleRad, double currentAngleRad, double& speedPercent);
  static double MinimizeRotation(double targetAngleRad, double currentAngleRad);
  static double WrapError(double targetAngle, double currentAngle);
  static double calcYawStraight(double targetAngle, double currentAngle, double kP);

  // ---------- Odometry (meters, field frame) ----------
  double GetFieldFWD() const { return positionFWDField.value(); }
  double GetFieldSTR() const { return positionSTRField.value(); }

  units::length::meter_t positionFWDField = units::length::meter_t(0);
  units::length::meter_t positionSTRField = units::length::meter_t(0);
  frc::Rotation2d ROTField = frc::Rotation2d(units::radian_t(0));
  double odoSTR = 0.0;
  double odoFWD = 0.0;
  double odoROT = 0.0;
  double odoDeltaTime = 0.0;
  units::time::second_t odoLastTime = units::time::second_t(0);

  // ---------- Hardware (Matching current CLEO CAN IDs) ----------
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

 private:
  void odometryUpdate(double angleFL, double angleFR, double angleBL, double angleBR,
                      double wheelSpeedFL, double wheelSpeedFR, double wheelSpeedBL,
                      double wheelSpeedBR, double GyroValue);
  void SyncFLSteerToCANcoder();
  void PublishTelemetry();

  // ---------- Module angles & speeds ----------
  double tempangleFL = 0.0, tempangleFR = 0.0, tempangleBL = 0.0, tempangleBR = 0.0;
  double angleFL = 0.0, angleFR = 0.0, angleBL = 0.0, angleBR = 0.0;
  double speedFL = 0.0, speedFR = 0.0, speedBL = 0.0, speedBR = 0.0;
  double cmdFL = 0.0, cmdFR = 0.0, cmdBL = 0.0, cmdBR = 0.0;

  double turnsFL = 0.0, turnsFR = 0.0, turnsBL = 0.0, turnsBR = 0.0;
  double lastTargetTurnsFL = 0.0, lastTargetTurnsFR = 0.0;
  double lastTargetTurnsBL = 0.0, lastTargetTurnsBR = 0.0;

  bool hasBootstrapped = false;
  bool m_configsApplied = false;

  // Heading lock state
  double storedYaw = 0.0;
  double m_headingIError = 0.0;
  double m_headingLastError = 0.0;

  units::time::second_t lastTime = units::time::second_t(0);
  double m_testRotateTurns = 0.0;
};
