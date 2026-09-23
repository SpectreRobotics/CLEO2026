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

  // Configure slide motor (Kraken X44) in brake mode and zero position
  ctre::phoenix6::configs::MotorOutputConfigs slideConfig{};
  slideConfig.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
  m_slideMotor.GetConfigurator().Apply(slideConfig);
  m_slideMotor.SetPosition(0_tr);

  // Configure modular turret subsystem (Kraken X44 rotation, 2x Kraken X60 shooter, 2x REV servos)
  m_turret.ConfigureMotors();

  // Team color chooser - select your alliance for Hub targeting
  m_teamColorChooser.SetDefaultOption("Blue", "Blue");
  m_teamColorChooser.AddOption("Red", "Red");
  frc::SmartDashboard::PutData("Team Color", &m_teamColorChooser);

  // Controller type chooser - AutoDetect, PS5, or Xbox
  m_controllerChooser.SetDefaultOption("AutoDetect", "AutoDetect");
  m_controllerChooser.AddOption("PS5", "PS5");
  m_controllerChooser.AddOption("Xbox", "Xbox");
  frc::SmartDashboard::PutData("Controller Type", &m_controllerChooser);

  // Pump mode toggle - when ON, intake oscillates forward/back during shooting
  frc::SmartDashboard::PutBoolean("Intake/PumpMode", false);

  // ========== AdvantageScope Data Logging & Telemetry ==========
  // Start on-robot .wpilog recording to USB / /home/lvuser/logs/
  frc::DataLogManager::Start();
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());

  // AdvantageScope & Dashboard 2D Field
  frc::SmartDashboard::PutData("Field", &m_field);

  // AdvantageScope NT4 struct publishers (Odometry and Swerve tabs)
  auto inst = nt::NetworkTableInstance::GetDefault();
  m_posePub = inst.GetTable("SmartDashboard")->GetStructTopic<frc::Pose2d>("RobotPose").Publish();
  m_moduleStatesPub = inst.GetTable("SmartDashboard")->GetStructArrayTopic<frc::SwerveModuleState>("SwerveStates").Publish();
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
  frc::SmartDashboard::PutString("Controller/ActiveType", IsPS5() ? "PS5 DualSense" : "Xbox");

  double slidePos = m_slideMotor.GetPosition().GetValueAsDouble();
  frc::SmartDashboard::PutNumber("Slide Position Rot", slidePos);
  frc::SmartDashboard::PutBoolean("Intake Out", m_intakeOut);

  // Periodic turret telemetry
  m_turret.Periodic();

  // Update dual Limelight vision-fused odometry (runs at all times)
  m_swerve.UpdateVision(GyroValue);

  // Display fused odometry on SmartDashboard
  frc::Pose2d fusedPose = m_swerve.GetFieldPose();
  frc::SmartDashboard::PutNumber("Odometry/FusedX", m_swerve.positionFWDField.value());
  frc::SmartDashboard::PutNumber("Odometry/FusedY", m_swerve.positionSTRField.value());

  // AdvantageScope 2D Field & NT4 struct telemetry
  m_field.SetRobotPose(fusedPose);
  m_posePub.Set(fusedPose);
  m_moduleStatesPub.Set(m_swerve.GetModuleStates());

  // Show Hub Target and Turret Direction on AdvantageScope 2D Field
  double targetX = (m_teamColorChooser.GetSelected() == "Red") ? TurretConstants::kRedHubX : TurretConstants::kBlueHubX;
  double targetY = (m_teamColorChooser.GetSelected() == "Red") ? TurretConstants::kRedHubY : TurretConstants::kBlueHubY;
  (m_field.GetObject)("HubTarget")->SetPose(frc::Pose2d(units::length::meter_t(targetX), units::length::meter_t(targetY), frc::Rotation2d{}));

  frc::Pose2d turretPose{fusedPose.Translation(), fusedPose.Rotation() + frc::Rotation2d(units::angle::degree_t(m_turret.GetRotationAngleDegrees()))};
  (m_field.GetObject)("Turret")->SetPose(turretPose);
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

  // ========== Swerve Driving (Xbox or PS5) ==========
  float rawx = GetDriverLeftX();
  float rawy = GetDriverLeftY();
  float rawx2 = GetDriverRightX();

  triggerL = GetDriverLeftTrigger();
  triggerR = GetDriverRightTrigger();

  // Apply deadzones (forward on controller stick is negative Y, so invert rawy)
  x = (std::abs(rawx) >= DrivetrainConstants::xdeadz) ? rawx : 0.0;
  y = (std::abs(rawy) >= DrivetrainConstants::ydeadz) ? -rawy : 0.0;
  x2 = (std::abs(rawx2) >= DrivetrainConstants::x2deadz) ? rawx2 : 0.0;

  FieldCentric = (m_driveModeChooser.GetSelected() == "FieldCentric");

  m_swerve.Update(x, y, x2, GyroValue, triggerL, triggerR, FieldCentric);

  // ========== Slide & Intake Controls ==========
  double slidePos = m_slideMotor.GetPosition().GetValueAsDouble();
  int pov = GetDriverPOV();
  bool isShooting = triggerL > 0.15;

  // Right bumper (RB on Xbox / R1 on PS5) toggles intake in/out
  if (GetDriverRightBumperPressed()) {
    m_intakeOut = !m_intakeOut;
  }

  // D-pad UP/DOWN: Manual slide override (safety backup)
  if (pov == 0) {
    m_slideMotor.Set(kSlideSpeed);
  } else if (pov == 180) {
    m_slideMotor.Set(-kSlideSpeed);
  } else if (m_intakeOut) {
    // ---- INTAKE OUT ----
    // Slide runs forward until it hits the physical hardstop
    m_slideMotor.Set(kSlideSpeed);
    // Intake roller spins when the slide is out
    m_IntakeMotor.Set(kIntakeSpeed);
  } else {
    // ---- INTAKE IN ----
    // Intake roller stops INSTANTLY when retracting
    m_IntakeMotor.Set(0.0);
    // Retract slide until it reaches home position
    if (slidePos > 0.1) {
      m_slideMotor.Set(-kSlideSpeed);
    } else {
      m_slideMotor.Set(0.0);
    }
  }

  // Pump Mode: optional SmartDashboard toggle
  // When shooting AND pump is enabled, intake roller oscillates forward/back
  // to help feed game pieces into the indexer
  bool pumpEnabled = frc::SmartDashboard::GetBoolean("Intake/PumpMode", false);
  if (isShooting && pumpEnabled) {
    // 0.8 second oscillation period: 0.4s forward, 0.4s backward
    double pumpTime = std::fmod(frc::Timer::GetFPGATimestamp().value(), 0.8);
    if (pumpTime < 0.4) {
      m_IntakeMotor.Set(0.3);   // Slow forward
    } else {
      m_IntakeMotor.Set(-0.3);  // Slow backward
    }
  }

  // ========== Indexer & Feeder (Run Together When Shooting) ==========
  if (isShooting) {
    m_indexer.Set(0.8);
    m_feeder.Set(0.8);
  } else {
    m_indexer.Set(0.0);
    m_feeder.Set(0.0);
  }

  // ========== Turret: Always Auto-Targets Hub ==========
  // Set alliance color from SmartDashboard chooser (affects which Hub to target)
  m_turret.SetAllianceRed(m_teamColorChooser.GetSelected() == "Red");

  // Turret always tracks the Hub using PhotonVision camera + odometry fallback
  m_turret.SetTargetingMode(Turret::TargetingMode::kAuto);
  m_turret.UpdateAutoTarget(
      m_swerve.positionFWDField,
      m_swerve.positionSTRField,
      units::angle::degree_t(GyroValue));

  // Left trigger controls shooting:
  //   Hold trigger  -> spin up flywheels + raise hood (based on distance)
  //   Release       -> stop flywheels + lower hood all the way down
  m_turret.Shoot(triggerL);

  // Display turret status
  frc::SmartDashboard::PutBoolean("Turret/Locked", m_turret.IsTargetLocked());
  frc::SmartDashboard::PutNumber("Turret/DistToHub", m_turret.GetTargetDistanceMeters());
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

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

  // Step slide motor simulation physics
  m_slideMotor.GetSimState().AddRotorPosition(
      units::angle::turn_t(m_slideMotor.Get() * 25.0 * 0.02));

  // Step turret simulation physics
  m_turret.UpdateSim(20_ms);
}

// ========== Unified Driver Controller Helpers (Xbox & PS5 DualSense) ==========

bool Robot::IsPS5() const {
  std::string choice = m_controllerChooser.GetSelected();
  if (choice == "PS5") return true;
  if (choice == "Xbox") return false;

  // AutoDetect: Check DriverStation joystick name and Xbox flag
  if (!frc::DriverStation::IsJoystickConnected(0)) return false;
  if (frc::DriverStation::GetJoystickIsXbox(0)) return false;

  std::string name = frc::DriverStation::GetJoystickName(0);
  if (name.find("PS5") != std::string::npos ||
      name.find("DualSense") != std::string::npos ||
      name.find("Wireless Controller") != std::string::npos ||
      name.find("Sony") != std::string::npos ||
      name.find("PlayStation") != std::string::npos) {
    return true;
  }

  // PS5 DualSense reports > 10 buttons (typically 14-16) vs Xbox 10
  if (frc::DriverStation::GetStickButtonCount(0) > 10) {
    return true;
  }

  return false;
}

double Robot::GetDriverLeftX() const {
  return IsPS5() ? m_ps5Controller.GetLeftX() : m_xboxController.GetLeftX();
}

double Robot::GetDriverLeftY() const {
  return IsPS5() ? m_ps5Controller.GetLeftY() : m_xboxController.GetLeftY();
}

double Robot::GetDriverRightX() const {
  return IsPS5() ? m_ps5Controller.GetRightX() : m_xboxController.GetRightX();
}

double Robot::GetDriverLeftTrigger() const {
  return IsPS5() ? m_ps5Controller.GetL2Axis() : m_xboxController.GetLeftTriggerAxis();
}

double Robot::GetDriverRightTrigger() const {
  return IsPS5() ? m_ps5Controller.GetR2Axis() : m_xboxController.GetRightTriggerAxis();
}

bool Robot::GetDriverRightBumperPressed() {
  return IsPS5() ? m_ps5Controller.GetR1ButtonPressed() : m_xboxController.GetRightBumperButtonPressed();
}

bool Robot::GetDriverResetGyroPressed() {
  if (IsPS5()) {
    return m_ps5Controller.GetCreateButtonPressed() || m_ps5Controller.GetTouchpadButtonPressed();
  }
  return m_xboxController.GetBackButtonPressed();
}

int Robot::GetDriverPOV() const {
  return IsPS5() ? m_ps5Controller.GetPOV() : m_xboxController.GetPOV();
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
