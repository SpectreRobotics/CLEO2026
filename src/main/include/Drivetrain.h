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
#include <ctre/phoenix6/controls/NeutralOut.hpp>
#include <ctre/phoenix6/controls/PositionDutyCycle.hpp>
#include <frc/geometry/Rotation2d.h>
#include <frc/Timer.h>
#include <frc/DriverStation.h>
#include <units/length.h>
#include <units/time.h>

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
              double triggerL, double triggerR, bool FieldCentric,
              bool isTankDrive = true, bool holdSteer = true);

  static std::pair<double, double> TankDriveKinematics(double throttle, double turn);

  double MinimizeRotation(double targetAngle, double currentAngle);
  static double MinimizeRotation(double targetAngleRad, double currentAngleRad, double& speedPercent);
  double WrapError(double targetAngle, double currentAngle);
  void odometryUpdate(double angleFL, double angleFR, double angleBL, double angleBR,
                      double wheelSpeedFL, double wheelSpeedFR, double wheelSpeedBL,
                      double wheelSpeedBR, double GyroValue);

  void TestZeroAzimuth();
  void TestRotateModules(double speed);
  double GetCANcoderFLTurns();
  bool IsCANcoderFLConnected();

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
  ctre::phoenix6::hardware::TalonFX m_FR_Drive{DrivetrainConstants::kFRDriveId};
  ctre::phoenix6::hardware::TalonFX m_BL_Drive{40};
  ctre::phoenix6::hardware::TalonFX m_BR_Drive{10};

  // Kraken Steering motors *1
  ctre::phoenix6::hardware::TalonFX m_FL_Steer{31};
  ctre::phoenix6::hardware::TalonFX m_FR_Steer{DrivetrainConstants::kFRSteerId};
  ctre::phoenix6::hardware::TalonFX m_BL_Steer{41};
  ctre::phoenix6::hardware::TalonFX m_BR_Steer{11};

  // CANcoders *2 (All 4 on "rio"; uncomment the "intakebus" line below if CANcoder 32 is moved back)
  // ctre::phoenix6::hardware::CANcoder CANcoderFL{32, ctre::phoenix6::CANBus{"intakebus"}};
  ctre::phoenix6::hardware::CANcoder CANcoderFL{32, ctre::phoenix6::CANBus{"rio"}};
  ctre::phoenix6::hardware::CANcoder CANcoderFR{22, ctre::phoenix6::CANBus{"rio"}};
  ctre::phoenix6::hardware::CANcoder CANcoderBL{42, ctre::phoenix6::CANBus{"rio"}};
  ctre::phoenix6::hardware::CANcoder CANcoderBR{12, ctre::phoenix6::CANBus{"rio"}};

 private:
  void SyncSmartDashboardPID();

  ctre::phoenix6::configs::Slot0Configs Posconfig{};
  ctre::phoenix6::configs::TalonFXConfiguration driveConfig{};
  ctre::phoenix6::configs::TalonFXConfiguration configFL{};
  ctre::phoenix6::configs::TalonFXConfiguration configFR{};
  ctre::phoenix6::configs::TalonFXConfiguration configBL{};
  ctre::phoenix6::configs::TalonFXConfiguration configBR{};

  float ModuleP = 2.0f;
  float ModuleI = 0.0f;
  float ModuleD = 1.5f;
  float ModuleS = 0.0f;
  float ModuleRampSeconds = 0.01f;
  float ModulePeakOutput = 0.30f;

  // Dedicated PID gains for Swerve 3 if using intakebus software fallback:
  // float FL_P = 1.2f;
  // float FL_I = 0.0f;
  // float FL_D = 0.4f;

  float straightP = 0.004f;
  float straightI = 0.0f;
  float straightD = 0.0f;

  const double L = 0.5461;
  const double W = 0.5461;

  double tempangleFL = 0, tempangleFR = 0, tempangleBL = 0, tempangleBR = 0;
  double angleFL = 0, angleFR = 0, angleBL = 0, angleBR = 0;
  double speedFL = 0, speedFR = 0, speedBL = 0, speedBR = 0;

  double oldspeedFL = 0, oldspeedFR = 0, oldspeedBL = 0, oldspeedBR = 0;
  double m_contTargetFL = 0, m_contTargetFR = 0, m_contTargetBL = 0, m_contTargetBR = 0;
  bool m_flipFL = false, m_flipFR = false, m_flipBL = false, m_flipBR = false;
  bool m_wasDriving = false;
  bool m_steerActiveFL = false, m_steerActiveFR = false, m_steerActiveBL = false, m_steerActiveBR = false;

  units::time::second_t lastTime = units::time::second_t(0);
  double lastError = 0;
  double iError = 0;

  double ROT = 0;
  double targetAngle = 0;
};

#endif  // DRIVETRAIN_H
