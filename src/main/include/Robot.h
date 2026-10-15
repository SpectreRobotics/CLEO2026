// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <frc/TimedRobot.h>
#include <frc/PS5Controller.h>
#include <frc/Timer.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/geometry/Pose2d.h>

#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>

#include "Drivetrain.h"
#include "DrivetrainConstants.h"

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

  // ========== Driver Controller Helpers (PS5 DualSense) ==========
  double GetDriverLeftX() const;
  double GetDriverLeftY() const;
  double GetDriverRightX() const;
  double GetDriverLeftTrigger() const;
  double GetDriverRightTrigger() const;
  bool GetDriverRightBumperPressed();
  bool GetDriverResetGyroPressed();
  int GetDriverPOV() const;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  // ========== Driver Controllers (PS5 DualSense - Port 0 with Port 1 fallback) ==========
  frc::PS5Controller m_ps5Controller0{0};
  frc::PS5Controller m_ps5Controller1{1};
  frc::Timer m_timer;

  // ========== Swerve Drivetrain ==========
  Drivetrain m_swerve;
  ctre::phoenix6::hardware::Pigeon2 m_pigeon{DrivetrainConstants::kPigeonId,
                                             ctre::phoenix6::CANBus{DrivetrainConstants::kCanBus}};

  // Drive Control Variables (ChimiSwerve convention: +X right, +Y forward, +X2 CW, Gyro CW+)
  double GyroValue = 0.0;
  double x = 0.0;
  double y = 0.0;
  double x2 = 0.0;
  double triggerL = 0.0;
  double triggerR = 0.0;
  bool FieldCentric = true;

  frc::SendableChooser<std::string> m_driveModeChooser;

  // Setup indexer, feeder, and both turret motors. 
  // Motor setups 
  ctre::phoenix6::hardware::TalonFX BottomFeederMotor {55};  
  ctre::phoenix6::hardware::TalonFX TopFeederMotor {57};  
  ctre::phoenix6::hardware::TalonFX IndexerMotor {50};  
  ctre::phoenix6::hardware::TalonFX OuttakeLeft {60};   
  ctre::phoenix6::hardware::TalonFX OuttakeRight {61};  
  ctre::phoenix6::hardware::TalonFX IntakeMotor {54}; 
  ctre::phoenix6::hardware::TalonFX RollerbedMotor {56};
  ctre::phoenix6::hardware::TalonFX DriveMotor1 {30};
  ctre::phoenix6::hardware::TalonFX DriveMotor2 {20};
  ctre::phoenix6::hardware::TalonFX DriveMotor3 {10};
  ctre::phoenix6::hardware::TalonFX DriveMotor4 {40};
  ctre::phoenix6::hardware::TalonFX SteerMotor1 {31};
  ctre::phoenix6::hardware::TalonFX SteerMotor2 {21};
  ctre::phoenix6::hardware::TalonFX SteerMotor3 {11};
  ctre::phoenix6::hardware::TalonFX Steermotor4 {41};
  ctre::phoenix6::hardware::TalonFX CANCoderSwerve1 {12};
  ctre::phoenix6::hardware::TalonFX CANCoderSwerve2 {22};
  ctre::phoenix6::hardware::TalonFX CANCoderSwerve3 {32};
  ctre::phoenix6::hardware::TalonFX CANCoderSwerve4 {42};
  // These are the fixed CAN ID's and updated motor names.

 
  int GetDriverPort() const;
};
