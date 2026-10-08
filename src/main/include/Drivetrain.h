// ============================================================================
// FRC Team 8753 - 2026 Swerve Drivetrain Header
// ============================================================================

#pragma once

#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/controls/PositionDutyCycle.hpp>

#include <cmath>
#include <vector>
#include <algorithm>
#include "DrivetrainConstants.h"

#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

class Drivetrain {
 public:
  Drivetrain();

  /**
   * @brief Configure all Kraken drive and steer TalonFX controllers and CANcoder fusion
   */
  void ConfigureMotors();

  /**
   * @brief Called in DisabledInit to stop drive motors
   */
  void DisabledInit();

  /**
   * @brief Called in DisabledPeriodic to track live wheel positions and publish telemetry
   */
  void DisabledPeriodic();

  /**
   * @brief Main swerve drive update - calculates module states and applies commands
   * @param x Strafe input (left/right, -1.0 to 1.0)
   * @param y Forward input (forward/back, -1.0 to 1.0)
   * @param x2 Rotation input (-1.0 to 1.0)
   * @param GyroValue Current heading in degrees
   * @param triggerL Left trigger value (brake/slow mode)
   * @param triggerR Right trigger value (boost mode)
   * @param FieldCentric true for field-centric, false for robot-centric
   */
  void Update(double x, double y, double x2, double GyroValue,
              double triggerL, double triggerR, bool FieldCentric);

  /**
   * @brief Optimizes swerve module rotation to minimize travel
   * @param targetAngleRad Desired wheel angle (radians)
   * @param currentAngleRad Current wheel angle (radians)
   * @param speedPercent Reference to speed - reversed if wheel flips
   * @return Optimized target angle (radians)
   */
  double MinimizeRotation(double targetAngleRad, double currentAngleRad, double& speedPercent);

  /**
   * @brief Updates robot field position using wheel odometry
   */
  void odometryUpdate(double angleFL, double angleFR, double angleBL, double angleBR,
                      double wheelSpeedFL, double wheelSpeedFR, double wheelSpeedBL,
                      double wheelSpeedBR, double GyroValue);

  /**
   * @brief Simulation physics update for desktop simulation
   */
  void UpdateSim(units::time::second_t dt);

  /**
   * @brief Returns current 2D field pose
   */
  frc::Pose2d GetFieldPose() const {
    return frc::Pose2d(frc::Translation2d(positionFWDField, positionSTRField), ROTField);
  }

  // ========== Odometry State (Accessible by Robot & Turret) ==========
  units::length::meter_t positionFWDField{0};  ///< Forward position on field (meters)
  units::length::meter_t positionSTRField{0};  ///< Strafe position on field (meters)
  frc::Rotation2d ROTField{units::angle::radian_t(0)}; ///< Field heading
  double odoSTR = 0.0;                         ///< Strafe velocity (m/s)
  double odoFWD = 0.0;                         ///< Forward velocity (m/s)

  double odoDeltaTime = 0.0;
  units::time::second_t odoLastTime{0};

  // ========== Hardware - Drive Motors (Native roboRIO CAN Bus) ==========
  // CAN IDs: FL=30, FR=20, BL=40, BR=10
  ctre::phoenix6::hardware::TalonFX m_FL_Drive{30};  // Module 3
  ctre::phoenix6::hardware::TalonFX m_FR_Drive{20};  // Module 2
  ctre::phoenix6::hardware::TalonFX m_BL_Drive{40};  // Module 4
  ctre::phoenix6::hardware::TalonFX m_BR_Drive{10};  // Module 1

  // ========== Hardware - Steering Motors (Native roboRIO CAN Bus) ==========
  // CAN IDs: FL=31, FR=21, BL=41, BR=11
  ctre::phoenix6::hardware::TalonFX m_FL_Steer{31};  // Module 3
  ctre::phoenix6::hardware::TalonFX m_FR_Steer{21};  // Module 2
  ctre::phoenix6::hardware::TalonFX m_BL_Steer{41};  // Module 4
  ctre::phoenix6::hardware::TalonFX m_BR_Steer{11};  // Module 1

 private:
  // ========== Hardware - Absolute Encoders (Native roboRIO CAN Bus) ==========
  // CAN IDs: FL=32, FR=22, BL=42, BR=12
  ctre::phoenix6::hardware::CANcoder CANcoderFL{32};  // Module 3
  ctre::phoenix6::hardware::CANcoder CANcoderFR{22};  // Module 2
  ctre::phoenix6::hardware::CANcoder CANcoderBL{42};  // Module 4
  ctre::phoenix6::hardware::CANcoder CANcoderBR{12};  // Module 1

  // ========== Motor Configurations ==========
  ctre::phoenix6::configs::TalonFXConfiguration driveConfig{};
  ctre::phoenix6::configs::TalonFXConfiguration configFL{};
  ctre::phoenix6::configs::TalonFXConfiguration configFR{};
  ctre::phoenix6::configs::TalonFXConfiguration configBL{};
  ctre::phoenix6::configs::TalonFXConfiguration configBR{};
  ctre::phoenix6::configs::Slot0Configs Posconfig{};

  // ========== Swerve Module State ==========
  double tempangleFL = 0.0, tempangleFR = 0.0, tempangleBL = 0.0, tempangleBR = 0.0;
  double angleFL = 0.0, angleFR = 0.0, angleBL = 0.0, angleBR = 0.0;
  double speedFL = 0.0, speedFR = 0.0, speedBL = 0.0, speedBR = 0.0;

  // ========== Timing ==========
  units::time::second_t lastTime{0};

  // ========== Joystick Inputs & Heading PID State ==========
  double ROT = 0.0;
  double FWD = 0.0;
  double STR = 0.0;
  double targetAngle = 0.0;
  double m_headingIError = 0.0;
  double m_headingLastError = 0.0;

  // ========== Wheel Velocities ==========
  double wheelSpeedFL = 0.0, wheelSpeedFR = 0.0, wheelSpeedBL = 0.0, wheelSpeedBR = 0.0;
};

#endif  // DRIVETRAIN_H
