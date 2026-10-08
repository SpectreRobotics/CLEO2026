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

  // Drive mode chooser
  m_driveModeChooser.SetDefaultOption("FieldCentric", "FieldCentric");
  m_driveModeChooser.AddOption("RobotCentric", "RobotCentric");
  frc::SmartDashboard::PutData("Drive Mode", &m_driveModeChooser);

  // Configure swerve drivetrain motors
  m_swerve.ConfigureMotors();

  // All other motors (slide, intake, feeder, indexer, turret) are commented out (swerve-only mode)
  /*
  ctre::phoenix6::configs::TalonFXConfiguration slideConfig{};
  slideConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
  slideConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  slideConfig.CurrentLimits.StatorCurrentLimit = units::current::ampere_t(40.0);
  m_slideMotor.GetConfigurator().Apply(slideConfig);
  m_slideMotor.SetPosition(0_tr);

  ctre::phoenix6::configs::CurrentLimitsConfigs mechLimits{};
  mechLimits.StatorCurrentLimitEnable = true;
  mechLimits.StatorCurrentLimit = units::current::ampere_t(40.0);
  m_IntakeMotor.GetConfigurator().Apply(mechLimits);
  m_indexer.GetConfigurator().Apply(mechLimits);
  m_feeder.GetConfigurator().Apply(mechLimits);

  m_turret.ConfigureMotors();
  */

  // Team color chooser - select your alliance for Hub targeting
  m_teamColorChooser.SetDefaultOption("Blue", "Blue");
  m_teamColorChooser.AddOption("Red", "Red");
  frc::SmartDashboard::PutData("Team Color", &m_teamColorChooser);

  // Controller type chooser - Default to Xbox for robust standard joystick mapping
  m_controllerChooser.SetDefaultOption("Xbox", "Xbox");
  m_controllerChooser.AddOption("PS5", "PS5");
  m_controllerChooser.AddOption("AutoDetect", "AutoDetect");
  frc::SmartDashboard::PutData("Controller Type", &m_controllerChooser);

  // Pump mode toggle
  frc::SmartDashboard::PutBoolean("Intake/PumpMode", false);
}

void Robot::Intake() {

}

void Robot::RobotPeriodic() {
  // Reset gyro heading: Back button (Xbox) or Create / Touchpad (PS5)
  if (GetDriverResetGyroPressed()) {
    m_pigeon.Reset();
  }
  GyroValue = -std::fmod(m_pigeon.GetYaw().GetValueAsDouble(), 360.0);
  frc::SmartDashboard::PutNumber("Gyro Heading", GyroValue);
  frc::SmartDashboard::PutNumber("GyroYawDeg", m_pigeon.GetYaw().GetValueAsDouble());
  frc::SmartDashboard::PutString("Controller/ActiveType", IsPS5() ? "PS5 DualSense" : "Xbox");

  // Non-swerve telemetry commented out (swerve-only mode)
  /*
  double slidePos = m_slideMotor.GetPosition().GetValueAsDouble();
  frc::SmartDashboard::PutNumber("Slide Position Rot", slidePos);
  frc::SmartDashboard::PutBoolean("Intake Out", m_intakeOut);

  // Periodic turret telemetry
  m_turret.Periodic();
  */

  // Display odometry on SmartDashboard
  frc::SmartDashboard::PutNumber("Odometry/FusedX", m_swerve.positionFWDField.value());
  frc::SmartDashboard::PutNumber("Odometry/FusedY", m_swerve.positionSTRField.value());
}

void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  wpi::print("Auto selected: {}\n", m_autoSelected);
}

void Robot::AutonomousPeriodic() {
  // Autonomous routines go here
}

void Robot::TeleopInit() {

}

void Robot::TeleopPeriodic() {
  using namespace units::literals;

  // ========== Swerve Driving (Xbox or PS5 on Port 0 / 1) ==========
  double rawx = GetDriverLeftX();
  double rawy = GetDriverLeftY();
  double rawx2 = GetDriverRightX();

  triggerL = GetDriverLeftTrigger();
  triggerR = GetDriverRightTrigger();

  // Apply deadzones (forward on controller stick is negative Y, so invert rawy)
  x = (std::abs(rawx) >= DrivetrainConstants::xdeadz) ? rawx : 0.0;
  y = (std::abs(rawy) >= DrivetrainConstants::ydeadz) ? -rawy : 0.0;
  x2 = (std::abs(rawx2) >= DrivetrainConstants::x2deadz) ? rawx2 : 0.0;

  FieldCentric = (m_driveModeChooser.GetSelected() == "FieldCentric");

  // Telemetry for verifying driver inputs live on SmartDashboard
  frc::SmartDashboard::PutNumber("Driver/RawLeftX", rawx);
  frc::SmartDashboard::PutNumber("Driver/RawLeftY", rawy);
  frc::SmartDashboard::PutNumber("Driver/RawRightX", rawx2);
  frc::SmartDashboard::PutNumber("Driver/Cmd_X", x);
  frc::SmartDashboard::PutNumber("Driver/Cmd_Y", y);
  frc::SmartDashboard::PutNumber("Driver/Cmd_ROT", x2);
  frc::SmartDashboard::PutNumber("Driver/ActivePort", GetDriverPort());
  frc::SmartDashboard::PutBoolean("Driver/Port0Connected", frc::DriverStation::IsJoystickConnected(0));
  frc::SmartDashboard::PutBoolean("Driver/Port1Connected", frc::DriverStation::IsJoystickConnected(1));

  m_swerve.Update(x, y, x2, GyroValue, triggerL, triggerR, FieldCentric);

  // All other mechanisms (slide, intake, turret, indexer, feeder) commented out (swerve-only mode)
  /*
  double slidePos = m_slideMotor.GetPosition().GetValueAsDouble();
  int pov = GetDriverPOV();
  bool isShooting = triggerL > 0.15;

  if (GetDriverRightBumperPressed()) {
    m_intakeOut = !m_intakeOut;
  }

  if (pov == 0) {
    m_slideMotor.Set(kSlideSpeed);
  } else if (pov == 180) {
    m_slideMotor.Set(-kSlideSpeed);
  } else if (m_intakeOut) {
    if (slidePos < kSlideOneFootRotations) {
      m_slideMotor.Set(kSlideSpeed);
    } else {
      m_slideMotor.Set(0.0);
    }
    m_IntakeMotor.Set(kIntakeSpeed);
  } else {
    m_IntakeMotor.Set(0.0);
    if (slidePos > 0.1) {
      m_slideMotor.Set(-kSlideSpeed);
    } else {
      m_slideMotor.Set(0.0);
    }
  }

  bool pumpEnabled = frc::SmartDashboard::GetBoolean("Intake/PumpMode", false);
  if (isShooting && pumpEnabled) {
    double pumpTime = std::fmod(frc::Timer::GetFPGATimestamp().value(), 0.8);
    if (pumpTime < 0.4) {
      m_IntakeMotor.Set(0.3);
    } else {
      m_IntakeMotor.Set(-0.3);
    }
  }

  m_turret.SetAllianceRed(m_teamColorChooser.GetSelected() == "Red");
  m_turret.SetTargetingMode(Turret::TargetingMode::kAuto);
  m_turret.UpdateAutoTarget(
      m_swerve.positionFWDField,
      m_swerve.positionSTRField,
      units::angle::degree_t(m_pigeon.GetYaw().GetValueAsDouble()));

  m_turret.Shoot(triggerL);

  bool isReadyToShoot = m_turret.IsReadyToShoot();
  frc::SmartDashboard::PutBoolean("Turret/ReadyToShoot", isReadyToShoot);

  if (isShooting && isReadyToShoot) {
    m_indexer.Set(0.8);
    m_feeder.Set(0.8);
  } else {
    m_indexer.Set(0.0);
    m_feeder.Set(0.0);
  }

  frc::SmartDashboard::PutBoolean("Turret/Locked", m_turret.IsTargetLocked());
  frc::SmartDashboard::PutNumber("Turret/DistToHub", m_turret.GetTargetDistanceMeters());
  */
}

void Robot::DisabledInit() {
  m_swerve.DisabledInit();
}

void Robot::DisabledPeriodic() {
  m_swerve.DisabledPeriodic();
}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {
  using namespace units::literals;

  // Step drivetrain swerve module simulation physics
  m_swerve.UpdateSim(20_ms);

  // Update simulated Pigeon2 gyro yaw based on rotation input
  if (std::abs(x2) > 0.05) {
    m_pigeon.GetSimState().AddYaw(units::angle::degree_t(-x2 * 360.0 * 0.02));
  }

  // Mechanism simulation commented out (swerve-only mode)
  /*
  double currentSlideRot = m_slideMotor.GetPosition().GetValueAsDouble();
  double nextSlideRot = currentSlideRot + (m_slideMotor.Get() * 25.0 * 0.02);
  nextSlideRot = std::clamp(nextSlideRot, 0.0, kSlideOneFootRotations);
  m_slideMotor.GetSimState().SetRawRotorPosition(units::angle::turn_t(nextSlideRot));

  m_turret.UpdateSim(20_ms);
  */
}

// ========== Unified Driver Controller Helpers (Xbox & PS5 DualSense) ==========

int Robot::GetDriverPort() const {
  if (frc::DriverStation::IsJoystickConnected(0)) return 0;
  if (frc::DriverStation::IsJoystickConnected(1)) return 1;
  return 0;
}

bool Robot::IsPS5() const {
  std::string choice = m_controllerChooser.GetSelected();
  if (choice == "PS5") return true;
  if (choice == "Xbox") return false;

  int port = GetDriverPort();
  if (frc::DriverStation::GetJoystickIsXbox(port)) return false;

  std::string name = frc::DriverStation::GetJoystickName(port);
  if (name.find("PS5") != std::string::npos ||
      name.find("DualSense") != std::string::npos ||
      name.find("Wireless Controller") != std::string::npos ||
      name.find("Sony") != std::string::npos ||
      name.find("PlayStation") != std::string::npos) {
    return true;
  }

  return false;
}

double Robot::GetDriverLeftX() const {
  int port = GetDriverPort();
  if (IsPS5()) {
    return (port == 0) ? m_ps5Controller0.GetLeftX() : m_ps5Controller1.GetLeftX();
  } else {
    return (port == 0) ? m_xboxController0.GetLeftX() : m_xboxController1.GetLeftX();
  }
}

double Robot::GetDriverLeftY() const {
  int port = GetDriverPort();
  if (IsPS5()) {
    return (port == 0) ? m_ps5Controller0.GetLeftY() : m_ps5Controller1.GetLeftY();
  } else {
    return (port == 0) ? m_xboxController0.GetLeftY() : m_xboxController1.GetLeftY();
  }
}

double Robot::GetDriverRightX() const {
  int port = GetDriverPort();
  if (IsPS5()) {
    return (port == 0) ? m_ps5Controller0.GetRightX() : m_ps5Controller1.GetRightX();
  } else {
    return (port == 0) ? m_xboxController0.GetRightX() : m_xboxController1.GetRightX();
  }
}

double Robot::GetDriverLeftTrigger() const {
  int port = GetDriverPort();
  if (IsPS5()) {
    return (port == 0) ? m_ps5Controller0.GetL2Axis() : m_ps5Controller1.GetL2Axis();
  } else {
    return (port == 0) ? m_xboxController0.GetLeftTriggerAxis() : m_xboxController1.GetLeftTriggerAxis();
  }
}

double Robot::GetDriverRightTrigger() const {
  int port = GetDriverPort();
  if (IsPS5()) {
    return (port == 0) ? m_ps5Controller0.GetR2Axis() : m_ps5Controller1.GetR2Axis();
  } else {
    return (port == 0) ? m_xboxController0.GetRightTriggerAxis() : m_xboxController1.GetRightTriggerAxis();
  }
}

bool Robot::GetDriverRightBumperPressed() {
  int port = GetDriverPort();
  if (IsPS5()) {
    return (port == 0) ? m_ps5Controller0.GetR1ButtonPressed() : m_ps5Controller1.GetR1ButtonPressed();
  } else {
    return (port == 0) ? m_xboxController0.GetRightBumperButtonPressed() : m_xboxController1.GetRightBumperButtonPressed();
  }
}

bool Robot::GetDriverResetGyroPressed() {
  int port = GetDriverPort();
  if (IsPS5()) {
    return (port == 0)
        ? (m_ps5Controller0.GetCreateButtonPressed() || m_ps5Controller0.GetTouchpadButtonPressed())
        : (m_ps5Controller1.GetCreateButtonPressed() || m_ps5Controller1.GetTouchpadButtonPressed());
  }
  return (port == 0) ? m_xboxController0.GetBackButtonPressed() : m_xboxController1.GetBackButtonPressed();
}

int Robot::GetDriverPOV() const {
  int port = GetDriverPort();
  return (port == 0) ? m_xboxController0.GetPOV() : m_xboxController1.GetPOV();
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
