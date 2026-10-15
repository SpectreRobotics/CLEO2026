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
#include <numbers>

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

  // Controller type chooser - AutoDetect so Xbox axes aren't read with the PS5 map
  m_controllerChooser.SetDefaultOption("AutoDetect", "AutoDetect");
  m_controllerChooser.AddOption("PS5", "PS5");
  m_controllerChooser.AddOption("Xbox", "Xbox");
  frc::SmartDashboard::PutData("Controller Type", &m_controllerChooser);
}

void Robot::RobotPeriodic() {
  // Reset gyro heading: 'A' button (Xbox) or 'Cross' (PS5)
  bool resetPressed = GetDriverResetGyroPressed();
  if (resetPressed) {
    m_pigeon.Reset();
    m_swerve.ResetOdometry();
    m_swerve.ResetRunaway();
  }
  frc::SmartDashboard::PutBoolean("Driver/ResetButtonPressed", resetPressed);
  // Pigeon2 yaw is CCW-positive, which matches the drivetrain convention directly.
  GyroValue = m_pigeon.GetYaw().GetValueAsDouble();
  frc::SmartDashboard::PutNumber("Gyro Heading", GyroValue);
  frc::SmartDashboard::PutString("Controller/ActiveType", IsPS5() ? "PS5 DualSense" : "Xbox");

  // Swerve telemetry + odometry (every mode)
  m_swerve.Periodic(GyroValue * std::numbers::pi / 180.0);
}

void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  wpi::print("Auto selected: {}\n", m_autoSelected);
}

void Robot::AutonomousPeriodic() {
  // Autonomous routines go here
}

void Robot::TeleopInit() {
  if (!m_swerve.AreConfigsApplied()) {
    m_swerve.ConfigureMotors();
  }
  m_swerve.ResetRunaway();
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


  // Deadband that rescales so output starts at 0 right after the deadband.
  auto deadband = [](double v) {
    double d = DrivetrainConstants::kDeadband;
    if (std::abs(v) < d) return 0.0;
    return std::copysign((std::abs(v) - d) / (1.0 - d), v);
  }

  // Joystick: up = -Y, right = +X. Drivetrain wants +forward, +left, +CCW.
  double forward = deadband(-rawy);
  double left = deadband(-rawx);
  double rotCCW = deadband(-rawx2);
  y = forward;
  x = left;
  x2 = rotCCW;

  // Speed: normal 50%, right trigger -> boost up to 100%, left trigger -> slow to 20%.
  double scale = DrivetrainConstants::kDriveNormalScale +
                 triggerR * (DrivetrainConstants::kDriveBoostScale - DrivetrainConstants::kDriveNormalScale) +
                 triggerL * (DrivetrainConstants::kDriveSlowScale - DrivetrainConstants::kDriveNormalScale);
  scale = std::clamp(scale, DrivetrainConstants::kDriveSlowScale, DrivetrainConstants::kDriveBoostScale);

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
  frc::SmartDashboard::PutNumber("Driver/Cmd_Forward", forward);
  frc::SmartDashboard::PutNumber("Driver/Cmd_Left", left);
  frc::SmartDashboard::PutNumber("Driver/Cmd_RotCCW", rotCCW);
  frc::SmartDashboard::PutNumber("Driver/SpeedScale", scale);

  m_swerve.Drive(forward, left, rotCCW, scale, GyroValue * std::numbers::pi / 180.0, FieldCentric);

  // Shooting control using right trigger
if (triggerR > 0.05) {
    FeederMotor.SetControl(
        ctre::phoenix6::controls::DutyCycleOut{0.70}
    );

    TurretMotor.SetControl(
        ctre::phoenix6::controls::DutyCycleOut{0.70}
    );

    TurretMotor2.SetControl(
        ctre::phoenix6::controls::DutyCycleOut{-0.70}
    );
} else {
    FeederMotor.SetControl(
        ctre::phoenix6::controls::DutyCycleOut{0.0}
    );

    TurretMotor.SetControl(
        ctre::phoenix6::controls::DutyCycleOut{0.0}
    );

    TurretMotor2.SetControl(
        ctre::phoenix6::controls::DutyCycleOut{0.0}
    );
}

// Intake control using the left trigger
if (triggerL > 0.05) {
    IntakeMotor.SetControl(
        ctre::phoenix6::controls::DutyCycleOut{0.70}
    );
} else {
    IntakeMotor.SetControl(
        ctre::phoenix6::controls::DutyCycleOut{0.0}
    );
}

// Shared indexer control, runs if either trigger is pressed
if (triggerR > 0.05 || triggerL > 0.05) {
    IndexerMotor.SetControl(
        ctre::phoenix6::controls::DutyCycleOut{0.70}
    );
} else {
    IndexerMotor.SetControl(
        ctre::phoenix6::controls::DutyCycleOut{0.0}
    );
}

void Robot::TeleopPeriodic() { 
    // 1. Choose target position 
    if (m_controller.GetAButton()) { 
        m_targetPosition = kPositionOut; 
    } 
 
    if (m_controller.GetBButton()) { 
        m_targetPosition = kPositionIn; 
    } 
 
    // 2. Read current Kraken position (rotations) 
    double current = 
        m_motor.GetPosition().GetValueAsDouble(); 
 
    // 3. Calculate error 
    double error = m_targetPosition - current; 
 
    // 4. Calculate time between loops (20 ms) 
    double dt = 0.020; 
 
    // 5. Manually calculate PID terms 
    double kP = 0.15; 
    double kI = 0.0; 
    double kD = 0.0; 
 
    double proportional = kP * error; 
 
    m_integral += error * dt; 
    double integral = kI * m_integral; 
 
    double derivative = (error - m_previousError) / dt; 
    double derivativeTerm = kD * derivative; 
 
    // 6. Add the terms together 
    double power = proportional + integral + derivativeTerm; 
 
    // 7. Limit motor output 
    power = std::clamp(power, -0.3, 0.3); 
 
    // 8. Stop when close to target 
    if (std::abs(error) < 0.05) { 
        power = 0.0; 
        m_integral = 0.0; 
    } 
 
    // 9. Send power to motor 
    m_motor.Set(power); 
 
    // 10. Save error for next loop 
    m_previousError = error; 
} 
void Robot::DisabledInit() {
  m_swerve.Stop();
}
}
void Robot::DisabledPeriodic() {
  // Retry configs once per second until every motor accepts them.
  static int loops = 0;
  if (!m_swerve.AreConfigsApplied() && (++loops % 50 == 0)) {
    m_swerve.ConfigureMotors();
  }
}

// TEST MODE = steer phase check (robot on blocks!). Every module's
// "Swerve/<mod>/SteerPhaseOK" must read true. See DrivetrainConstants.h.
void Robot::TestInit() {
  m_swerve.ResetRunaway();
}

void Robot::TestPeriodic() {
  m_swerve.TestPhase();
}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {
  using namespace units::literals;

  // Step drivetrain swerve module simulation physics
  m_swerve.UpdateSim(0.02);

  // Update simulated Pigeon2 gyro yaw based on rotation input (CCW+)
  if (std::abs(x2) > 0.05) {
    m_pigeon.GetSimState().AddYaw(units::angle::degree_t(x2 * 360.0 * 0.02));
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
  if (choice == "PS5") return true;
  if (choice == "Xbox") return false;

  // AutoDetect: Check whether DriverStation recognizes the device as an Xbox controller
  int port = GetDriverPort();
  if (frc::DriverStation::GetJoystickIsXbox(port)) {
    return false;
  }
  // Any non-Xbox joystick (PS5 DualSense, DirectInput, etc.) defaults to PS5
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
             m_ps5Controller0.GetTouchpadButtonPressed();
    }
    if (port == 1) {
      return m_ps5Controller1.GetCrossButtonPressed() ||
             m_ps5Controller1.GetCreateButtonPressed() ||
             m_ps5Controller1.GetTouchpadButtonPressed();
    }
    return frc::PS5Controller{port}.GetCrossButtonPressed() ||
           frc::PS5Controller{port}.GetCreateButtonPressed() ||
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
