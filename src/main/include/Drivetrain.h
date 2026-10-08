// ============================================================================
// FRC Team 8753 - 2026 Swerve Drivetrain Header
// ============================================================================
// Complete ground-up implementation based 1:1 on ChimiSwerve (Team 1684) White Paper.
// Hardware: 4x MK3.5 Swerve Modules with Kraken X60 (TalonFX) & CTRE CANcoders.
// All swerve drive motors, steer motors, CANcoders, and Pigeon2 run on roboRIO CAN bus.
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
   * @brief Main teleop/auton swerve update function (ChimiSwerve Sections 2.1b & 2.3d)
   * @param x Strafe joystick input [-1, 1]
   * @param y Forward joystick input [-1, 1]
   * @param x2 Rotation joystick input [-1, 1]
   * @param GyroValue Current gyro heading in degrees (clockwise positive)
   * @param triggerL Left trigger modifier (slowdown)
   * @param triggerR Right trigger modifier (boost)
   * @param FieldCentric True for field-centric, false for robot-centric
   */
  void Update(double x, double y, double x2, double GyroValue,
              double triggerL, double triggerR, bool FieldCentric);

  /**
   * @brief ChimiSwerve Inversion Awareness & Azimuth Optimization (Pages 15-16)
   * Calculates continuous turn setpoint within +/- 0.25 turns (+/- 90 deg) of current position.
   * Negates wheel speed if target is >90 deg away.
   * @param waRad Target azimuth angle in radians from inverse kinematics [-pi, pi]
   * @param currentTurns Current steering motor position in turns (unbounded)
   * @param ws Reference to wheel speed (negated if flipped)
   * @return Continuous setpoint in turns for Phoenix 6 PositionDutyCycle
   */
  double ChimiOptimizeAzimuth(double waRad, double currentTurns, double& ws);

  /**
   * @brief ChimiSwerve Forward Kinematics & Odometry (Section 2.2b, Pages 17-19)
   */
  void odometryUpdate(double waFL, double waFR, double waBL, double waBR,
                      double wsFL, double wsFR, double wsBL, double wsBR,
                      double GyroValue);

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

  // ========== Odometry State (Accessible by Robot.cpp) ==========
  units::length::meter_t positionFWDField{0};  ///< Forward distance on field (meters)
  units::length::meter_t positionSTRField{0};  ///< Strafe distance on field (meters)
  frc::Rotation2d ROTField{units::angle::radian_t(0)}; ///< Field heading
  double odoSTR = 0.0;                         ///< Robot strafe velocity (m/s)
  double odoFWD = 0.0;                         ///< Robot forward velocity (m/s)

  double odoDeltaTime = 0.0;
  units::time::second_t odoLastTime{0};

  // ========== Hardware: Drive Motors (Kraken TalonFX) ==========
  // CAN IDs: FL=30, FR=20, BL=40, BR=10 (Native roboRIO CAN bus)
  ctre::phoenix6::hardware::TalonFX m_FL_Drive{30};  // Module 3
  ctre::phoenix6::hardware::TalonFX m_FR_Drive{20};  // Module 2
  ctre::phoenix6::hardware::TalonFX m_BL_Drive{40};  // Module 4
  ctre::phoenix6::hardware::TalonFX m_BR_Drive{10};  // Module 1

  // ========== Hardware: Steer Motors (Kraken TalonFX) ==========
  // CAN IDs: FL=31, FR=21, BL=41, BR=11 (Native roboRIO CAN bus)
  ctre::phoenix6::hardware::TalonFX m_FL_Steer{31};  // Module 3
  ctre::phoenix6::hardware::TalonFX m_FR_Steer{21};  // Module 2
  ctre::phoenix6::hardware::TalonFX m_BL_Steer{41};  // Module 4
  ctre::phoenix6::hardware::TalonFX m_BR_Steer{11};  // Module 1

 private:
  // ========== Hardware: Absolute CANcoders ==========
  // CAN IDs: FL=32, FR=22, BL=42, BR=12 (Native roboRIO CAN bus)
  ctre::phoenix6::hardware::CANcoder CANcoderFL{32};  // Module 3
  ctre::phoenix6::hardware::CANcoder CANcoderFR{22};  // Module 2
  ctre::phoenix6::hardware::CANcoder CANcoderBL{42};  // Module 4
  ctre::phoenix6::hardware::CANcoder CANcoderBR{12};  // Module 1

  // ========== Motor Configurations ==========
  ctre::phoenix6::configs::TalonFXConfiguration driveConfigFL{};
  ctre::phoenix6::configs::TalonFXConfiguration driveConfigFR{};
  ctre::phoenix6::configs::TalonFXConfiguration driveConfigBL{};
  ctre::phoenix6::configs::TalonFXConfiguration driveConfigBR{};
  ctre::phoenix6::configs::TalonFXConfiguration steerConfigFL{};
  ctre::phoenix6::configs::TalonFXConfiguration steerConfigFR{};
  ctre::phoenix6::configs::TalonFXConfiguration steerConfigBL{};
  ctre::phoenix6::configs::TalonFXConfiguration steerConfigBR{};
  ctre::phoenix6::configs::Slot0Configs steerSlot0{};

  ctre::phoenix6::controls::PositionDutyCycle steerRequest{0_tr};

  // ========== Module State ==========
  double lastTargetTurnsFL = 0.0;
  double lastTargetTurnsFR = 0.0;
  double lastTargetTurnsBL = 0.0;
  double lastTargetTurnsBR = 0.0;

  double storedYaw = 0.0;  ///< ChimiSwerve Section 2.3d stored heading
  bool hasBootstrapped = false;

  units::time::second_t lastTime{0};
};

#endif  // DRIVETRAIN_H
