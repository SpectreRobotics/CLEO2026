// ============================================================================
// FRC Team 8753 - 2026 Swerve Drivetrain (Restored from swerve-base / 2026_Bot)
// ============================================================================

#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <algorithm>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/DutyCycleOut.hpp>
#include <ctre/phoenix6/controls/PositionDutyCycle.hpp>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/Timer.h>
#include <frc/DriverStation.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>

#include "DrivetrainConstants.h"

#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

struct ModuleStates {
  double wsFL = 0, wsFR = 0, wsBL = 0, wsBR = 0;
  double waFL = 0, waFR = 0, waBL = 0, waBR = 0;
};

class Drivetrain {
 public:
  Drivetrain();

  void ConfigureMotors();
  bool AreConfigsApplied() const { return true; }

  void DisabledInit();
  void DisabledPeriodic();

  void Update(double x, double y, double x2, double GyroValue,
              double triggerL, double triggerR, bool FieldCentric);

  double MinimizeRotation(double targetAngle, double currentAngle);
  static double MinimizeRotation(double targetAngleRad, double currentAngleRad, double& speedPercent);
  double WrapError(double targetAngle, double currentAngle);
  void odometryUpdate(double angleFL, double angleFR, double angleBL, double angleBR,
                      double wheelSpeedFL, double wheelSpeedFR, double wheelSpeedBL,
                      double wheelSpeedBR, double GyroValue);

  void TestZeroAzimuth();
  void TestRotateModules(double speed);

  void ResetHeading(double heading = 0.0) {
    targetAngle = heading;
    lastError = 0.0;
    iError = 0.0;
  }

  void ResetOdometry() {
    positionFWDField = units::length::meter_t(0);
    positionSTRField = units::length::meter_t(0);
    ROTField = frc::Rotation2d(units::radian_t(0));
    odoFWD = 0;
    odoSTR = 0;
    odoROT = 0;
  }

  void UpdateSim(units::time::second_t dt);

  // Unit test helpers
  static ModuleStates InverseKinematics(double FWD, double STR, double ROT);
  static double ChimiOptimizeAzimuth(double waRad, double currentTurns, double& ws);
  static double calcYawStraight(double targetAngle, double currentAngle, double kP);

  double GetFieldFWD() const { return positionFWDField.value(); }
  double GetFieldSTR() const { return positionSTRField.value(); }

  units::length::meter_t positionFWDField = units::length::meter_t(0);
  units::length::meter_t positionSTRField = units::length::meter_t(0);
  frc::Rotation2d ROTField = frc::Rotation2d(units::radian_t(0));

  double odoROT = 0;
  double odoFWD = 0;
  double odoSTR = 0;

  double odoDeltaTime = 0;
  units::time::second_t odoLastTime = units::time::second_t(0);

  // Kraken Drive motors *0
  ctre::phoenix6::hardware::TalonFX m_FL_Drive{30};
  ctre::phoenix6::hardware::TalonFX m_FR_Drive{20};
  ctre::phoenix6::hardware::TalonFX m_BL_Drive{40};
  ctre::phoenix6::hardware::TalonFX m_BR_Drive{10};

  // Kraken Steering motors *1
  ctre::phoenix6::hardware::TalonFX m_FL_Steer{31};
  ctre::phoenix6::hardware::TalonFX m_FR_Steer{21};
  ctre::phoenix6::hardware::TalonFX m_BL_Steer{41};
  ctre::phoenix6::hardware::TalonFX m_BR_Steer{11};

  // CANcoders *2 (FL CANcoder 32 on "intakebus", FR/BL/BR on "rio")
  ctre::phoenix6::hardware::CANcoder CANcoderFL{32, ctre::phoenix6::CANBus{DrivetrainConstants::kIntakeCanBus}};
  ctre::phoenix6::hardware::CANcoder CANcoderFR{22, ctre::phoenix6::CANBus{DrivetrainConstants::kCanBus}};
  ctre::phoenix6::hardware::CANcoder CANcoderBL{42, ctre::phoenix6::CANBus{DrivetrainConstants::kCanBus}};
  ctre::phoenix6::hardware::CANcoder CANcoderBR{12, ctre::phoenix6::CANBus{DrivetrainConstants::kCanBus}};

  double GetFLCANcoderTurns();

  // WPILib 4-Module Swerve Kinematics & Odometry
  frc::Translation2d m_flLocation{units::meter_t{DrivetrainConstants::L / 2.0}, units::meter_t{DrivetrainConstants::W / 2.0}};
  frc::Translation2d m_frLocation{units::meter_t{DrivetrainConstants::L / 2.0}, units::meter_t{-DrivetrainConstants::W / 2.0}};
  frc::Translation2d m_blLocation{units::meter_t{-DrivetrainConstants::L / 2.0}, units::meter_t{DrivetrainConstants::W / 2.0}};
  frc::Translation2d m_brLocation{units::meter_t{-DrivetrainConstants::L / 2.0}, units::meter_t{-DrivetrainConstants::W / 2.0}};
  frc::SwerveDriveKinematics<4> m_kinematics{m_flLocation, m_frLocation, m_blLocation, m_brLocation};
  frc::SwerveDriveOdometry<4> m_odometry{
      m_kinematics,
      frc::Rotation2d{0_deg},
      {frc::SwerveModulePosition{}, frc::SwerveModulePosition{},
       frc::SwerveModulePosition{}, frc::SwerveModulePosition{}}};

 private:
  ctre::phoenix6::configs::Slot0Configs Posconfig{};
  ctre::phoenix6::configs::TalonFXConfiguration driveConfig{};
  ctre::phoenix6::configs::TalonFXConfiguration configFL{};
  ctre::phoenix6::configs::TalonFXConfiguration configFR{};
  ctre::phoenix6::configs::TalonFXConfiguration configBL{};
  ctre::phoenix6::configs::TalonFXConfiguration configBR{};

  float ModuleP = 2.0;
  float ModuleI = 0.0;
  float ModuleD = 0.0;
  float straightP = 0.0;
  float straightI = 0.0;
  float straightD = 0.0;

  const double L = 0.5461;
  const double W = 0.5461;

  double tempangleFL = 0, tempangleFR = 0, tempangleBL = 0, tempangleBR = 0;
  double angleFL = 0, angleFR = 0, angleBL = 0, angleBR = 0;
  double speedFL = 0, speedFR = 0, speedBL = 0, speedBR = 0;

  double oldspeedFL = 0, oldspeedFR = 0, oldspeedBL = 0, oldspeedBR = 0;

  units::time::second_t lastTime = units::time::second_t(0);
  double lastError = 0;
  double iError = 0;

  double ROT = 0;
  double targetAngle = 0;
};

#endif  // DRIVETRAIN_H
