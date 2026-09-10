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
}

void Robot::Intake() {

}

void Robot::Turret() {
  
}

void Robot::RobotPeriodic() {
  // Back button (button 7) resets gyro heading
  if (m_controller.GetBackButtonPressed()) {
    m_pigeon.Reset();
  }
  GyroValue = -std::fmod(m_pigeon.GetYaw().GetValueAsDouble(), 360.0);
  frc::SmartDashboard::PutNumber("Gyro Heading", GyroValue);
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

  // ========== Mechanism Controls ==========
  int pov = m_controller.GetPOV();
  if (pov == 0) {
    m_slideMotor.Set(0.4);
  } 
  else if (pov == 180) {
    m_slideMotor.Set(-0.4);
  } else {
    // Timed intake/slide sequencing
    if (m_controller.GetRightBumperButtonPressed()) {
      m_intakeOut = !m_intakeOut;
      m_timer.Restart();
      if (m_intakeOut) {
        m_IntakeMotor.Set(0.8);
      } else {
        m_IntakeMotor.Set(0.0);
      }
    }

    units::second_t elapsedTime = m_timer.Get();

    if (m_intakeOut) {
      if (elapsedTime < 2.0_s) {
        m_slideMotor.Set(0.4);
      } else {
        m_slideMotor.Set(0.0);
        m_IntakeMotor.Set(0.6);
      }
    } else {
      if (elapsedTime < 2.0_s) {
        m_slideMotor.Set(-0.4);
      } else {
        m_slideMotor.Set(0.0);
      }
    }
  }

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
