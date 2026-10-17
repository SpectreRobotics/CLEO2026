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

  // Drive mode chooser - defaults to RobotCentric
  m_driveModeChooser.SetDefaultOption("RobotCentric", "RobotCentric");
  m_driveModeChooser.AddOption("FieldCentric", "FieldCentric");
  frc::SmartDashboard::PutData("Drive Mode", &m_driveModeChooser);
  frc::SmartDashboard::PutBoolean("Drive/FieldCentric", false);

  // Robot orientation chooser - reverses forward and backward facing direction
  m_orientationChooser.SetDefaultOption("Forward (Normal)", "Forward");
  m_orientationChooser.AddOption("Reversed (180 deg)", "Reversed");
  frc::SmartDashboard::PutData("Robot Orientation", &m_orientationChooser);
  frc::SmartDashboard::PutBoolean("Drive/ReverseDirection", false);

  // Button to reset gyro heading from SmartDashboard
  frc::SmartDashboard::PutBoolean("Reset Gyro", false);
  frc::SmartDashboard::PutBoolean("Reset Field Centric Heading", false);
  frc::SmartDashboard::PutBoolean("Reset Odometry", false);

  // Configure swerve drivetrain motors and zero gyro heading on boot
  m_pigeon.Reset();
  m_swerve.ConfigureMotors();
}

void Robot::RobotPeriodic() {
  // Pigeon2 yaw is CCW-positive; ChimiSwerve convention is CLOCKWISE-positive.
  GyroValue = -std::fmod(m_pigeon.GetYaw().GetValueAsDouble(), 360.0);

  // Reset gyro heading: 'Touchpad' (box), 'Square' (box), 'Create', 'Options', or SmartDashboard buttons
  bool dashResetField = frc::SmartDashboard::GetBoolean("Reset Field Centric Heading", false);
  bool dashResetGyro = frc::SmartDashboard::GetBoolean("Reset Gyro", false);
  bool resetPressed = GetDriverResetGyroPressed() || dashResetField || dashResetGyro;
  if (resetPressed) {
    m_pigeon.Reset();
    GyroValue = 0.0;
    m_swerve.ResetHeading(0.0);
    m_swerve.ResetOdometry();
    if (dashResetField) {
      frc::SmartDashboard::PutBoolean("Reset Field Centric Heading", false);
    }
    if (dashResetGyro) {
      frc::SmartDashboard::PutBoolean("Reset Gyro", false);
    }
  }

  // SmartDashboard button to reset odometry
  if (frc::SmartDashboard::GetBoolean("Reset Odometry", false)) {
    m_swerve.ResetOdometry();
    frc::SmartDashboard::PutBoolean("Reset Odometry", false);
  }

  frc::SmartDashboard::PutBoolean("Driver/ResetButtonPressed", resetPressed);
  frc::SmartDashboard::PutNumber("Gyro Heading (CW deg)", GyroValue);
  frc::SmartDashboard::PutBoolean("Gyro/Connected", m_pigeon.IsConnected());
  frc::SmartDashboard::PutString("Controller/ActiveType", "PS5 DualSense");

  // Diagnostics panel
  frc::SmartDashboard::PutString("Diagnostics/Drive_Mode", FieldCentric ? "FieldCentric" : "RobotCentric");
  frc::SmartDashboard::PutString("Diagnostics/Active_Modules", "FL (30), FR (20), BL (40), BR (10)");
  frc::SmartDashboard::PutString("Diagnostics/FL_Status", "Active (Swerve 3 Connected)");
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
  m_swerve.ConfigureMotors();
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

  // ChimiSwerve Section 3.5: Deadzone interpolation for rotation stick (Right X)
  if (std::abs(rawx2) < DrivetrainConstants::x2deadz) {
    x2 = 0.0;
  } else {
    x2 = std::copysign(
        std::min(1.0, (std::abs(rawx2) - DrivetrainConstants::x2deadz) /
                          (1.0 - DrivetrainConstants::x2deadz)),
        rawx2);
  }

  // Reverse orientation toggle from SmartDashboard (dropdown chooser or checkbox toggle)
  bool reverseFromChooser = (m_orientationChooser.GetSelected() == "Reversed");
  bool reverseFromToggle = frc::SmartDashboard::GetBoolean("Drive/ReverseDirection", false);
  bool reverseForward = reverseFromChooser || reverseFromToggle;
  if (reverseForward) {
    x = -x;
    y = -y;
  }
  frc::SmartDashboard::PutBoolean("Drive/IsReversed", reverseForward);

  // Drive mode: default is RobotCentric; only FieldCentric when explicitly selected and gyro connected
  bool wantFieldFromChooser = (m_driveModeChooser.GetSelected() == "FieldCentric");
  bool wantFieldFromToggle = frc::SmartDashboard::GetBoolean("Drive/FieldCentric", false);
  FieldCentric = (wantFieldFromChooser || wantFieldFromToggle) && m_pigeon.IsConnected();
  frc::SmartDashboard::PutBoolean("Drive/IsFieldCentric", FieldCentric);

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

// ========== Driver Controller Helpers (PS5 DualSense Only) ==========

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
  if (port == 1) return m_ps5Controller1.GetLeftX();
  return m_ps5Controller0.GetLeftX();
}

double Robot::GetDriverLeftY() const {
  int port = GetDriverPort();
  if (port == 1) return m_ps5Controller1.GetLeftY();
  return m_ps5Controller0.GetLeftY();
}

double Robot::GetDriverRightX() const {
  int port = GetDriverPort();
  if (port == 1) return m_ps5Controller1.GetRightX();
  return m_ps5Controller0.GetRightX();
}

double Robot::GetDriverLeftTrigger() const {
  int port = GetDriverPort();
  if (port == 1) return m_ps5Controller1.GetL2Axis();
  return m_ps5Controller0.GetL2Axis();
}

double Robot::GetDriverRightTrigger() const {
  int port = GetDriverPort();
  if (port == 1) return m_ps5Controller1.GetR2Axis();
  return m_ps5Controller0.GetR2Axis();
}

bool Robot::GetDriverRightBumperPressed() {
  int port = GetDriverPort();
  if (port == 1) return m_ps5Controller1.GetR1ButtonPressed();
  return m_ps5Controller0.GetR1ButtonPressed();
}

bool Robot::GetDriverResetGyroPressed() {
  int port = GetDriverPort();
  auto& c = (port == 1) ? m_ps5Controller1 : m_ps5Controller0;
  return c.GetTouchpadButtonPressed() ||
         c.GetSquareButtonPressed() ||
         c.GetCreateButtonPressed() ||
         c.GetOptionsButtonPressed() ||
         c.GetCrossButtonPressed();
}

bool Robot::GetDriverCrossButton() const {
  int port = GetDriverPort();
  if (port == 1) return m_ps5Controller1.GetCrossButton();
  return m_ps5Controller0.GetCrossButton();
}

bool Robot::GetDriverCircleButton() const {
  int port = GetDriverPort();
  if (port == 1) return m_ps5Controller1.GetCircleButton();
  return m_ps5Controller0.GetCircleButton();
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
