// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/time.h>
#include <wpi/print.h>

Robot::Robot() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

void Robot::Intake() {

}

void Robot::Turret() {
  
}


void Robot::RobotPeriodic() {

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

  int pov = m_controller.GetPOV();
  if (pov == 0) {
    m_slideMotor.Set(0.4);
    return; 
  } 
  else if (pov == 180) {
    m_slideMotor.Set(-0.4);
    return;
  }

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
