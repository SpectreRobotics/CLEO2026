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
  void Turret();

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

ctre::phoenix6::hardware::TalonFX m_slideMotor{1};
ctre::phoenix6::hardware::TalonFX m_IntakeMotor{2};
ctre::phoenix6::hardware::TalonFX m_indexer{3};
ctre::phoenix6::hardware::TalonFX m_feeder{4};


  frc::XboxController m_controller{0};
  frc::Timer m_timer;

  bool m_intakeOut = false;
};
