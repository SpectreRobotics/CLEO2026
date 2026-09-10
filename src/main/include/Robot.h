// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/Orchestra.hpp>
#include <ctre/phoenix6/CANcoder.hpp>

#include "Drivetrain.h"
#include "DrivetrainConstants.h"
#include "Turret.h"
#include "TurretConstants.h"

class Robot : public frc::TimedRobot {
 public:
  Robot();
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;
  void Intake();

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  // ========== Intake & Slide Mechanisms ==========
  ctre::phoenix6::hardware::TalonFX m_slideMotor{1};   // Kraken X44 sliding intake motor
  ctre::phoenix6::hardware::TalonFX m_IntakeMotor{2};  // Kraken X60 intake roller motor
  ctre::phoenix6::hardware::TalonFX m_indexer{3};
  ctre::phoenix6::hardware::TalonFX m_feeder{4};

  static constexpr double kIntakeSpeed = 0.8;
  static constexpr double kSlideSpeed = 0.4;
  static constexpr double kSlideOneFootRotations = 15.0; // Adjust for 1 foot of travel based on mechanism gearing

  frc::XboxController m_controller{0};
  frc::Timer m_timer;

  bool m_intakeOut = false;

  // ========== Swerve Drivetrain ==========
  Drivetrain m_swerve;
  ctre::phoenix6::hardware::Pigeon2 m_pigeon{2};

  // Drive Control Variables
  double GyroValue = 0.0;
  double x = 0.0;
  double y = 0.0;
  double x2 = 0.0;
  float triggerL = 0.0f;
  float triggerR = 0.0f;
  bool FieldCentric = true;

  frc::SendableChooser<std::string> m_driveModeChooser;

  // ========== Turret Subsystem ==========
  Turret m_turret;
};
