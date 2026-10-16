// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <algorithm>
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

  // Drive mode chooser - defaults to FieldCentric (ChimiSwerve Section 2.1a)
  m_driveModeChooser.SetDefaultOption("FieldCentric", "FieldCentric");
  m_driveModeChooser.AddOption("RobotCentric", "RobotCentric");
  frc::SmartDashboard::PutData("Drive Mode", &m_driveModeChooser);

  // Configure swerve drivetrain motors (preserves current CANcoder magnet offsets)
  m_swerve.ConfigureMotors();
}

void Robot::RobotPeriodic() {
  // Pigeon2 yaw is CCW-positive; ChimiSwerve convention is CLOCKWISE-positive.
  GyroValue = -std::fmod(m_pigeon.GetYaw().GetValueAsDouble(), 360.0);

  // Reset gyro heading: 'Cross', 'Create' (Share), 'Options', or 'Touchpad' (PS5)
  bool resetPressed = GetDriverResetGyroPressed();
  if (resetPressed) {
    m_pigeon.Reset();
    GyroValue = 0.0;
    m_swerve.ResetHeading(0.0);
    m_swerve.ResetOdometry();
  }

  frc::SmartDashboard::PutBoolean("Driver/ResetButtonPressed", resetPressed);
  frc::SmartDashboard::PutNumber("Gyro Heading", GyroValue);
  frc::SmartDashboard::PutNumber("Gyro Heading (CW deg)", GyroValue);
  frc::SmartDashboard::PutBoolean("Gyro/Connected", m_pigeon.IsConnected());
  frc::SmartDashboard::PutString("Controller/ActiveType", "PS5 DualSense");
}

void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  wpi::print("Auto selected: {}\n", m_autoSelected);
  m_swerve.ResetHeading(GyroValue);
  m_swerve.ResetOdometry();
}

void Robot::AutonomousPeriodic() {
  // Autonomous routines go here
}

void Robot::TeleopInit() {
  if (!m_swerve.AreConfigsApplied()) {
    m_swerve.ConfigureMotors();
  }
  m_swerve.ResetHeading(GyroValue);
}

void Robot::TeleopPeriodic() {
  using namespace units::literals;

  // ========== Swerve Driving (PS5 DualSense Controller) ==========
  double rawx = GetDriverLeftX();
  double rawy = GetDriverLeftY();
  double rawx2 = GetDriverRightX();

  triggerL = std::clamp(GetDriverLeftTrigger(), 0.0, 1.0);
  triggerR = std::clamp(GetDriverRightTrigger(), 0.0, 1.0);
  if (triggerL < 0.05) triggerL = 0.0;
  if (triggerR < 0.05) triggerR = 0.0;

  // ChimiSwerve Section 3.5 (Page 40): Radial deadzone interpolation for translation stick
  //   Left stick Y: push up = -Y on PS5 controller -> +1.0 forward (FWD)
  //   Left stick X: push right = +X on PS5 controller -> +1.0 strafe right (STR)
  const double rawSTR = rawx;
  const double rawFWD = -rawy;
  const double mag = std::hypot(rawSTR, rawFWD);
  if (mag < DrivetrainConstants::xdeadz) {
    x = 0.0;
    y = 0.0;
  } else {
    const double scaledMag = std::min(
        1.0, (mag - DrivetrainConstants::xdeadz) / (1.0 - DrivetrainConstants::xdeadz));
    x = (rawSTR / mag) * scaledMag;
    y = (rawFWD / mag) * scaledMag;
  }

  // Right stick X: push right = +X on PS5 controller -> +1.0 clockwise rotation (ROT)
  if (std::abs(rawx2) < DrivetrainConstants::x2deadz) {
    x2 = 0.0;
  } else {
    x2 = std::copysign(
        (std::abs(rawx2) - DrivetrainConstants::x2deadz) / (1.0 - DrivetrainConstants::x2deadz),
        rawx2);
  }

  // ChimiSwerve Section 3.7 (Page 40): Automatic fallback to RobotCentric if gyro faults
  bool wantFieldCentric = (m_driveModeChooser.GetSelected() == "FieldCentric");
  FieldCentric = wantFieldCentric && m_pigeon.IsConnected();

  // Telemetry for verifying driver inputs live on SmartDashboard
  int activePort = GetDriverPort();
  frc::SmartDashboard::PutNumber("Driver/ActivePort", activePort);
  frc::SmartDashboard::PutString("Driver/ControllerType", "PS5 DualSense");
  frc::SmartDashboard::PutNumber("Driver/RawLeftX", rawx);
  frc::SmartDashboard::PutNumber("Driver/RawLeftY", rawy);
  frc::SmartDashboard::PutNumber("Driver/RawRightX", rawx2);
  frc::SmartDashboard::PutNumber("Driver/RawL2", triggerL);
  frc::SmartDashboard::PutNumber("Driver/RawR2", triggerR);
  frc::SmartDashboard::PutNumber("Driver/Cmd_FWD", y);
  frc::SmartDashboard::PutNumber("Driver/Cmd_STR", x);
  frc::SmartDashboard::PutNumber("Driver/Cmd_ROT", x2);

  m_swerve.Update(x, y, x2, GyroValue, triggerL, triggerR, FieldCentric);

  // ========== Mechanism Controls (Commented out - Swerve only) ==========
  // // Shooting / Outtake control using right trigger (R2)
  // if (triggerR > 0.05) {
  //   BottomFeederMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.70});
  //   TopFeederMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.70});
  //   OuttakeLeft.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.70});
  //   OuttakeRight.SetControl(ctre::phoenix6::controls::DutyCycleOut{-0.70});
  // } else {
  //   BottomFeederMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
  //   TopFeederMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
  //   OuttakeLeft.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
  //   OuttakeRight.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
  // }
  //
  // // Intake control using left trigger (L2)
  // if (triggerL > 0.05) {
  //   IntakeMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.70});
  //   RollerbedMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.70});
  // } else {
  //   IntakeMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
  //   RollerbedMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
  // }
  //
  // // Shared indexer control, runs if either trigger is pressed
  // if (triggerR > 0.05 || triggerL > 0.05) {
  //   IndexerMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.70});
  // } else {
  //   IndexerMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
  // }
}

void Robot::DisabledInit() {
  m_swerve.DisabledInit();
}

void Robot::DisabledPeriodic() {
  m_swerve.DisabledPeriodic();
}

void Robot::TestInit() {
  if (!m_swerve.AreConfigsApplied()) {
    m_swerve.ConfigureMotors();
  }
}

// ChimiSwerve Section 3.1 (Page 35): Zero Azimuth Position & Pre-Match Module Testing
//   - Hold Cross (X) on PS5 controller: command all wheels to 0 deg (straight forward)
//   - Hold Circle (O) on PS5 controller: slowly rotate all wheels to verify smooth steering
void Robot::TestPeriodic() {
  if (GetDriverCrossButton()) {
    m_swerve.TestZeroAzimuth();
  } else if (GetDriverCircleButton()) {
    m_swerve.TestRotateModules(0.02);
  } else {
    m_swerve.DisabledPeriodic();
  }
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

// ========== Driver Controller Helpers (PS5 DualSense) ==========

int Robot::GetDriverPort() const {
  for (int p = 0; p < 6; ++p) {
    if (frc::DriverStation::IsJoystickConnected(p)) {
      return p;
    }
  }
  return 0;
}

double Robot::GetDriverLeftX() const {
  int port = GetDriverPort();
  if (port == 0) return m_ps5Controller0.GetLeftX();
  if (port == 1) return m_ps5Controller1.GetLeftX();
  return frc::PS5Controller{port}.GetLeftX();
}

double Robot::GetDriverLeftY() const {
  int port = GetDriverPort();
  if (port == 0) return m_ps5Controller0.GetLeftY();
  if (port == 1) return m_ps5Controller1.GetLeftY();
  return frc::PS5Controller{port}.GetLeftY();
}

double Robot::GetDriverRightX() const {
  int port = GetDriverPort();
  if (port == 0) return m_ps5Controller0.GetRightX();
  if (port == 1) return m_ps5Controller1.GetRightX();
  return frc::PS5Controller{port}.GetRightX();
}

double Robot::GetDriverLeftTrigger() const {
  int port = GetDriverPort();
  if (port == 0) return m_ps5Controller0.GetL2Axis();
  if (port == 1) return m_ps5Controller1.GetL2Axis();
  return frc::PS5Controller{port}.GetL2Axis();
}

double Robot::GetDriverRightTrigger() const {
  int port = GetDriverPort();
  if (port == 0) return m_ps5Controller0.GetR2Axis();
  if (port == 1) return m_ps5Controller1.GetR2Axis();
  return frc::PS5Controller{port}.GetR2Axis();
}

bool Robot::GetDriverRightBumperPressed() {
  int port = GetDriverPort();
  if (port == 0) return m_ps5Controller0.GetR1ButtonPressed();
  if (port == 1) return m_ps5Controller1.GetR1ButtonPressed();
  return frc::PS5Controller{port}.GetR1ButtonPressed();
}

bool Robot::GetDriverResetGyroPressed() {
  int port = GetDriverPort();
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

bool Robot::GetDriverCrossButton() const {
  int port = GetDriverPort();
  if (port == 0) return m_ps5Controller0.GetCrossButton();
  if (port == 1) return m_ps5Controller1.GetCrossButton();
  return frc::PS5Controller{port}.GetCrossButton();
}

bool Robot::GetDriverCircleButton() const {
  int port = GetDriverPort();
  if (port == 0) return m_ps5Controller0.GetCircleButton();
  if (port == 1) return m_ps5Controller1.GetCircleButton();
  return frc::PS5Controller{port}.GetCircleButton();
}

int Robot::GetDriverPOV() const {
  int port = GetDriverPort();
  if (port == 0) return m_ps5Controller0.GetPOV();
  if (port == 1) return m_ps5Controller1.GetPOV();
  return frc::PS5Controller{port}.GetPOV();
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
