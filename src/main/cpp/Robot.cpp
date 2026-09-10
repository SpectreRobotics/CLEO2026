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

  // Configure slide motor (Kraken X44) in brake mode and zero position
  ctre::phoenix6::configs::MotorOutputConfigs slideConfig{};
  slideConfig.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
  m_slideMotor.GetConfigurator().Apply(slideConfig);
  m_slideMotor.SetPosition(0_tr);

  // Configure modular turret subsystem (Kraken X44 rotation, 2x Kraken X60 shooter, 2x REV servos)
  m_turret.ConfigureMotors();
}

void Robot::Intake() {

}

void Robot::RobotPeriodic() {
  // Back button (button 7) resets gyro heading
  if (m_controller.GetBackButtonPressed()) {
    m_pigeon.Reset();
  }
  GyroValue = -std::fmod(m_pigeon.GetYaw().GetValueAsDouble(), 360.0);
  frc::SmartDashboard::PutNumber("Gyro Heading", GyroValue);

  double slidePos = m_slideMotor.GetPosition().GetValueAsDouble();
  frc::SmartDashboard::PutNumber("Slide Position Rot", slidePos);
  frc::SmartDashboard::PutBoolean("Intake Out", m_intakeOut);

  // Periodic turret telemetry
  m_turret.Periodic();
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

  // ========== Swerve Driving ==========
  float rawx = m_controller.GetLeftX();
  float rawy = m_controller.GetLeftY();
  float rawx2 = m_controller.GetRightX();

  triggerL = m_controller.GetLeftTriggerAxis();
  triggerR = m_controller.GetRightTriggerAxis();

  // Apply deadzones (forward on Xbox stick is negative Y, so invert rawy)
  x = (std::abs(rawx) >= DrivetrainConstants::xdeadz) ? rawx : 0.0;
  y = (std::abs(rawy) >= DrivetrainConstants::ydeadz) ? -rawy : 0.0;
  x2 = (std::abs(rawx2) >= DrivetrainConstants::x2deadz) ? rawx2 : 0.0;

  FieldCentric = (m_driveModeChooser.GetSelected() == "FieldCentric");

  m_swerve.Update(x, y, x2, GyroValue, triggerL, triggerR, FieldCentric);

  // ========== Slide & Intake Controls ==========
  double slidePos = m_slideMotor.GetPosition().GetValueAsDouble();
  int pov = m_controller.GetPOV();

  // D-pad Manual Backup Override
  if (pov == 0) {
    // D-pad UP: Manual slide forward
    m_slideMotor.Set(kSlideSpeed);
  } else if (pov == 180) {
    // D-pad DOWN: Manual slide backward
    m_slideMotor.Set(-kSlideSpeed);
  } else {
    // Automatic Right Bumper Toggle
    if (m_controller.GetRightBumperButtonPressed()) {
      m_intakeOut = !m_intakeOut;
    }

    if (m_intakeOut) {
      // Slide forward 1 foot
      if (slidePos < kSlideOneFootRotations) {
        m_slideMotor.Set(kSlideSpeed);
      } else {
        m_slideMotor.Set(0.0);
        m_IntakeMotor.Set(kIntakeSpeed); // Spin at 0.8 when out
      }
    } else {
      // Retract slide and turn off intake
      m_IntakeMotor.Set(0.0);
      if (slidePos > 0.1) {
        m_slideMotor.Set(-kSlideSpeed);
      } else {
        m_slideMotor.Set(0.0);
      }
    }
  }

  // Safety: If intake is all the way in, ensure intake motor is turned off
  if (slidePos <= 0.2 && !m_intakeOut) {
    m_IntakeMotor.Set(0.0);
  }

  // Indexer on button 8
  if (m_controller.GetRawButtonPressed(8)) {
    m_indexer.Set(0.8);
  } else {
    m_indexer.Set(0.0);
  }
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
