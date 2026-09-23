// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/PS5Controller.h>
#include <frc/Timer.h>
#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/struct/Pose2dStruct.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/struct/Pose3dStruct.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/struct/SwerveModuleStateStruct.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructTopic.h>
#include <networktables/StructArrayTopic.h>

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

  // ========== Unified Driver Controller Helpers (Xbox & PS5 DualSense) ==========
  bool IsPS5() const;
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

  // ========== Intake & Slide Mechanisms ==========
  ctre::phoenix6::hardware::TalonFX m_slideMotor{1};   // Kraken X44 sliding intake motor
  ctre::phoenix6::hardware::TalonFX m_IntakeMotor{2};  // Kraken X60 intake roller motor
  ctre::phoenix6::hardware::TalonFX m_indexer{3};
  ctre::phoenix6::hardware::TalonFX m_feeder{4};

  static constexpr double kIntakeSpeed = 0.8;
  static constexpr double kSlideSpeed = 0.4;
  static constexpr double kSlideOneFootRotations = 15.0; // Adjust for 1 foot of travel based on mechanism gearing

  // ========== Driver Controllers (Xbox & PS5 DualSense on Port 0) ==========
  frc::XboxController m_xboxController{0};
  frc::PS5Controller m_ps5Controller{0};
  frc::SendableChooser<std::string> m_controllerChooser;
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

  // ========== Team Color Chooser ==========
  frc::SendableChooser<std::string> m_teamColorChooser;

  // ========== AdvantageScope & Dashboard Telemetry ==========
  frc::Field2d m_field;
  nt::StructPublisher<frc::Pose2d> m_posePub;
  nt::StructArrayPublisher<frc::SwerveModuleState> m_moduleStatesPub;
  nt::StructPublisher<frc::Pose3d> m_robotPose3dPub;
  nt::StructArrayPublisher<frc::Pose3d> m_componentPosesPub;
};
