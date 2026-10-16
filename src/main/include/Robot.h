// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <frc/TimedRobot.h>
#include <frc/PS5Controller.h>
#include <frc/XboxController.h>
#include <frc/Timer.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/geometry/Pose2d.h>

#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <rev/ServoHub.h>
#include <rev/ServoChannel.h>

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

  // ========== Driver Controller Helpers (PS5 DualSense & XInput Auto-Detect) ==========
  bool IsPS5() const;
  double GetDriverLeftX() const;
  double GetDriverLeftY() const;
  double GetDriverRightX() const;
  double GetDriverLeftTrigger() const;
  double GetDriverRightTrigger() const;
  bool GetDriverRightBumperPressed();
  bool GetDriverResetGyroPressed();
  bool GetDriverCrossButton() const;
  bool GetDriverCircleButton() const;
  int GetDriverPOV() const;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  // ========== Driver Controllers (PS5 DualSense & XInput - Port 0/1 with Auto-Detect) ==========
  frc::PS5Controller m_ps5Controller0{0};
  frc::PS5Controller m_ps5Controller1{1};
  frc::XboxController m_xboxController0{0};
  frc::XboxController m_xboxController1{1};
  frc::SendableChooser<std::string> m_controllerChooser;
  frc::Timer m_timer;

  // ========== Swerve Drivetrain (ChimiSwerve White Paper) ==========
  // Swerve CAN IDs (owned by m_swerve, defined in DrivetrainConstants.h):
  //   FL (Mod 3): Drive 30, Steer 31, CANcoder 32
  //   FR (Mod 2): Drive 20, Steer 21, CANcoder 22
  //   BL (Mod 4): Drive 40, Steer 41, CANcoder 42
  //   BR (Mod 1): Drive 10, Steer 11, CANcoder 12
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
  bool FieldCentric = false;

  frc::SendableChooser<std::string> m_driveModeChooser;

  // ========== REV Servo Hub (CAN ID 63 on "rio" bus) ==========
  // Two servos defined on channels 2 and 3
  rev::servohub::ServoHub m_servoHub{DrivetrainConstants::kServoHubId};
  rev::servohub::ServoChannel& m_servoChannel2 = m_servoHub.GetServoChannel(rev::servohub::ServoChannel::ChannelId::kChannelId2);
  rev::servohub::ServoChannel& m_servoChannel3 = m_servoHub.GetServoChannel(rev::servohub::ServoChannel::ChannelId::kChannelId3);

  // ========== Mechanism Motors (Commented out - Swerve only) ==========
  // // "Tophalf" CAN bus: Feeders, Indexer, Outtake
  // ctre::phoenix6::hardware::TalonFX BottomFeederMotor{DrivetrainConstants::kBottomFeederMotorId,
  //                                                     ctre::phoenix6::CANBus{DrivetrainConstants::kTophalfCanBus}};
  // ctre::phoenix6::hardware::TalonFX TopFeederMotor{DrivetrainConstants::kTopFeederMotorId,
  //                                                  ctre::phoenix6::CANBus{DrivetrainConstants::kTophalfCanBus}};
  // ctre::phoenix6::hardware::TalonFX IndexerMotor{DrivetrainConstants::kIndexerMotorId,
  //                                                ctre::phoenix6::CANBus{DrivetrainConstants::kTophalfCanBus}};
  // ctre::phoenix6::hardware::TalonFX OuttakeLeft{DrivetrainConstants::kOuttakeLeftMotorId,
  //                                               ctre::phoenix6::CANBus{DrivetrainConstants::kTophalfCanBus}};
  // ctre::phoenix6::hardware::TalonFX OuttakeRight{DrivetrainConstants::kOuttakeRightMotorId,
  //                                                ctre::phoenix6::CANBus{DrivetrainConstants::kTophalfCanBus}};
  //
  // // "intakebus" CAN bus: Intake and Rollerbed
  // ctre::phoenix6::hardware::TalonFX IntakeMotor{DrivetrainConstants::kIntakeMotorId,
  //                                               ctre::phoenix6::CANBus{DrivetrainConstants::kIntakeCanBus}};
  // ctre::phoenix6::hardware::TalonFX RollerbedMotor{DrivetrainConstants::kRollerbedMotorId,
  //                                                  ctre::phoenix6::CANBus{DrivetrainConstants::kIntakeCanBus}};

  int GetDriverPort() const;
};
