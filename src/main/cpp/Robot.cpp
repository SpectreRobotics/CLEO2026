// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/time.h>
#include <wpi/print.h>

#define _USE_MATH_DEFINES
#include <cmath>

#ifdef GetObject
#undef GetObject
#endif

Robot::Robot() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  // Drive mode chooser - defaults to FieldCentric
  m_driveModeChooser.SetDefaultOption("FieldCentric", "FieldCentric");
  m_driveModeChooser.AddOption("RobotCentric", "RobotCentric");
  frc::SmartDashboard::PutData("Drive Mode", &m_driveModeChooser);

  // Configure swerve drivetrain motors
  m_swerve.ConfigureMotors();

  // Controller type chooser - defaults to PS5 controller
  m_controllerChooser.SetDefaultOption("PS5", "PS5");
  m_controllerChooser.AddOption("Xbox", "Xbox");
  frc::SmartDashboard::PutData("Controller Type", &m_controllerChooser);
}

void Robot::RobotPeriodic() {
  // Pigeon2 yaw is CCW-positive; ChimiSwerve convention is CLOCKWISE-positive.
  GyroValue = -m_pigeon.GetYaw().GetValueAsDouble();

  // Reset gyro heading: 'A' button (Xbox) or 'Cross' / 'Touchpad' (PS5)
  bool resetPressed = GetDriverResetGyroPressed();
  if (resetPressed) {
    m_pigeon.Reset();
    GyroValue = 0.0;
    m_swerve.ResetHeading(0.0);
    m_swerve.ResetOdometry();
  }

  frc::SmartDashboard::PutBoolean("Driver/ResetButtonPressed", resetPressed);
  frc::SmartDashboard::PutNumber("Gyro Heading (CW deg)", GyroValue);
  frc::SmartDashboard::PutString("Controller/ActiveType", IsPS5() ? "PS5 DualSense" : "Xbox");
}

void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  wpi::print("Auto selected: {}\n", m_autoSelected);
  m_swerve.ResetHeading(GyroValue);
}

void Robot::AutonomousPeriodic() {
  // Autonomous routines go here
}

void Robot::TeleopInit() {
  m_swerve.ResetHeading(GyroValue);
}

void Robot::TeleopPeriodic() {
  using namespace units::literals;

  // ========== Swerve Driving (Xbox or PS5 DualSense) ==========
  double rawx = GetDriverLeftX();
  double rawy = GetDriverLeftY();
  double rawx2 = GetDriverRightX();

  triggerL = std::clamp(GetDriverLeftTrigger(), 0.0, 1.0);
  triggerR = std::clamp(GetDriverRightTrigger(), 0.0, 1.0);
  if (triggerL < 0.05) triggerL = 0.0;
  if (triggerR < 0.05) triggerR = 0.0;

  // Deadband helper
  auto deadband = [](double val, double d) {
    if (std::abs(val) < d) return 0.0;
    return std::copysign((std::abs(val) - d) / (1.0 - d), val);
  };

  // Controller mapping for ChimiSwerve:
  //   Left stick Y: push up = -Y on controller -> +1.0 forward
  //   Left stick X: push right = +X on controller -> +1.0 strafe right
  //   Right stick X: push right = +X on controller -> +1.0 clockwise rotation
  y = deadband(-rawy, DrivetrainConstants::ydeadz);
  x = deadband(rawx, DrivetrainConstants::xdeadz);
  x2 = deadband(rawx2, DrivetrainConstants::x2deadz);

  FieldCentric = (m_driveModeChooser.GetSelected() == "FieldCentric");

  // Telemetry for verifying driver inputs live on SmartDashboard
  int activePort = GetDriverPort();
  frc::SmartDashboard::PutNumber("Driver/ActivePort", activePort);
  frc::SmartDashboard::PutString("Driver/ControllerType", IsPS5() ? "PS5 DualSense" : "Xbox");
  frc::SmartDashboard::PutNumber("Driver/RawLeftX", rawx);
  frc::SmartDashboard::PutNumber("Driver/RawLeftY", rawy);
  frc::SmartDashboard::PutNumber("Driver/RawRightX", rawx2);
  frc::SmartDashboard::PutNumber("Driver/RawL2", triggerL);
  frc::SmartDashboard::PutNumber("Driver/RawR2", triggerR);
  frc::SmartDashboard::PutNumber("Driver/Cmd_FWD", y);
  frc::SmartDashboard::PutNumber("Driver/Cmd_STR", x);
  frc::SmartDashboard::PutNumber("Driver/Cmd_RCW", x2);

  m_swerve.Update(x, y, x2, GyroValue, triggerL, triggerR, FieldCentric);
}

void Robot::DisabledInit() {
  m_swerve.DisabledInit();
}

void Robot::DisabledPeriodic() {
  m_swerve.DisabledPeriodic();
}

void Robot::TestInit() {}

void Robot::TestPeriodic() {
  m_swerve.DisabledPeriodic();
}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {
  using namespace units::literals;

  // Step drivetrain swerve module simulation physics
  m_swerve.UpdateSim(20_ms);

  // Update simulated Pigeon2 gyro yaw based on rotation input (Pigeon2 is CCW+, so CW rotation is negative)
  if (std::abs(x2) > 0.05) {
    m_pigeon.GetSimState().AddYaw(units::angle::degree_t(-x2 * 360.0 * 0.02));
  }
}

// ========== Unified Driver Controller Helpers (Xbox & PS5 DualSense) ==========

int Robot::GetDriverPort() const {
  for (int p = 0; p < 6; ++p) {
    if (frc::DriverStation::IsJoystickConnected(p)) {
      return p;
    }
  }
  return 0;
}

bool Robot::IsPS5() const {
  std::string choice = m_controllerChooser.GetSelected();
  if (choice == "Xbox") return false;
  return true;
}

double Robot::GetDriverLeftX() const {
  int port = GetDriverPort();
  if (IsPS5()) {
    if (port == 0) return m_ps5Controller0.GetLeftX();
    if (port == 1) return m_ps5Controller1.GetLeftX();
    return frc::PS5Controller{port}.GetLeftX();
  } else {
    if (port == 0) return m_xboxController0.GetLeftX();
    if (port == 1) return m_xboxController1.GetLeftX();
    return frc::XboxController{port}.GetLeftX();
  }
}

double Robot::GetDriverLeftY() const {
  int port = GetDriverPort();
  if (IsPS5()) {
    if (port == 0) return m_ps5Controller0.GetLeftY();
    if (port == 1) return m_ps5Controller1.GetLeftY();
    return frc::PS5Controller{port}.GetLeftY();
  } else {
    if (port == 0) return m_xboxController0.GetLeftY();
    if (port == 1) return m_xboxController1.GetLeftY();
    return frc::XboxController{port}.GetLeftY();
  }
}

double Robot::GetDriverRightX() const {
  int port = GetDriverPort();
  if (IsPS5()) {
    if (port == 0) return m_ps5Controller0.GetRightX();
    if (port == 1) return m_ps5Controller1.GetRightX();
    return frc::PS5Controller{port}.GetRightX();
  } else {
    if (port == 0) return m_xboxController0.GetRightX();
    if (port == 1) return m_xboxController1.GetRightX();
    return frc::XboxController{port}.GetRightX();
  }
}

double Robot::GetDriverLeftTrigger() const {
  int port = GetDriverPort();
  if (IsPS5()) {
    if (port == 0) return m_ps5Controller0.GetL2Axis();
    if (port == 1) return m_ps5Controller1.GetL2Axis();
    return frc::PS5Controller{port}.GetL2Axis();
  } else {
    if (port == 0) return m_xboxController0.GetLeftTriggerAxis();
    if (port == 1) return m_xboxController1.GetLeftTriggerAxis();
    return frc::XboxController{port}.GetLeftTriggerAxis();
  }
}

double Robot::GetDriverRightTrigger() const {
  int port = GetDriverPort();
  if (IsPS5()) {
    if (port == 0) return m_ps5Controller0.GetR2Axis();
    if (port == 1) return m_ps5Controller1.GetR2Axis();
    return frc::PS5Controller{port}.GetR2Axis();
  } else {
    if (port == 0) return m_xboxController0.GetRightTriggerAxis();
    if (port == 1) return m_xboxController1.GetRightTriggerAxis();
    return frc::XboxController{port}.GetRightTriggerAxis();
  }
}

bool Robot::GetDriverRightBumperPressed() {
  int port = GetDriverPort();
  if (IsPS5()) {
    if (port == 0) return m_ps5Controller0.GetR1ButtonPressed();
    if (port == 1) return m_ps5Controller1.GetR1ButtonPressed();
    return frc::PS5Controller{port}.GetR1ButtonPressed();
  } else {
    if (port == 0) return m_xboxController0.GetRightBumperButtonPressed();
    if (port == 1) return m_xboxController1.GetRightBumperButtonPressed();
    return frc::XboxController{port}.GetRightBumperButtonPressed();
  }
}

bool Robot::GetDriverResetGyroPressed() {
  int port = GetDriverPort();
  if (IsPS5()) {
    if (port == 0) {
      return m_ps5Controller0.GetCrossButtonPressed() ||
             m_ps5Controller0.GetCreateButtonPressed() ||
             m_ps5Controller0.GetOptionsButtonPressed() ||
             m_ps5Controller0.GetTouchpadButtonPressed();
    }
    if (port == 1) {
      return m_ps5Controller1.GetCrossButtonPressed() ||
             m_ps5Controller1.GetCreateButtonPressed() ||
             m_ps5Controller1.GetOptionsButtonPressed() ||
             m_ps5Controller1.GetTouchpadButtonPressed();
    }
    return frc::PS5Controller{port}.GetCrossButtonPressed() ||
           frc::PS5Controller{port}.GetCreateButtonPressed() ||
           frc::PS5Controller{port}.GetOptionsButtonPressed() ||
           frc::PS5Controller{port}.GetTouchpadButtonPressed();
  }
  // Xbox: 'A' button resets gyro (Back button also supported as fallback)
  if (port == 0) {
    return m_xboxController0.GetAButtonPressed() || m_xboxController0.GetBackButtonPressed();
  }
  if (port == 1) {
    return m_xboxController1.GetAButtonPressed() || m_xboxController1.GetBackButtonPressed();
  }
  return frc::XboxController{port}.GetAButtonPressed() || frc::XboxController{port}.GetBackButtonPressed();
}

int Robot::GetDriverPOV() const {
  int port = GetDriverPort();
  return frc::DriverStation::GetStickPOV(port, 0);
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
