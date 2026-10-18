// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <algorithm>
#include <frc/MathUtil.h>
#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/time.h>
#include <wpi/print.h>

#define _USE_MATH_DEFINES
#include <cmath>

#ifdef GetObject
#undef GetObject
#endif

Robot::Robot() {
  m_chooser.SetDefaultOption(kAutoShootPreload, kAutoShootPreload);
  m_chooser.AddOption(kAutoDoNothing, kAutoDoNothing);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  // Auto shooting tunables & enable toggle (Reflect compatible)
  frc::SmartDashboard::PutBoolean("Auto/EnableAutoRoutine", true);
  frc::SmartDashboard::PutBoolean("Auto/IsAutoEnabled", true);
  frc::SmartDashboard::PutNumber("Auto/SpinupSeconds", 1.0);
  frc::SmartDashboard::PutNumber("Auto/FeedDurationSeconds", 2.0);
  frc::SmartDashboard::PutNumber("Auto/OuttakeSpeed", DrivetrainConstants::kOuttakeSpeed);
  frc::SmartDashboard::PutNumber("Auto/FeederSpeed", DrivetrainConstants::kFeederSpeed);
  frc::SmartDashboard::PutString("Auto/State", "Ready");

  // Top half speeds & manual / reversal controls
  frc::SmartDashboard::PutNumber("TopHalf/OuttakeSpeed", DrivetrainConstants::kOuttakeSpeed);
  frc::SmartDashboard::PutNumber("TopHalf/FeederSpeed", DrivetrainConstants::kFeederSpeed);
  frc::SmartDashboard::PutBoolean("Diagnostics/ReverseTopFeeder", false);
  frc::SmartDashboard::PutBoolean("Diagnostics/ReverseBottomFeeder", false);
  frc::SmartDashboard::PutBoolean("Diagnostics/ReverseOuttake", false);
  frc::SmartDashboard::PutBoolean("Diagnostics/ManualRunTopFeeder", false);
  frc::SmartDashboard::PutBoolean("Diagnostics/ManualRunBottomFeeder", false);
  frc::SmartDashboard::PutBoolean("Diagnostics/ManualRunOuttake", false);
  frc::SmartDashboard::PutNumber("Diagnostics/ManualTestSpeed", 0.50);

  // Swerve module testing buttons
  frc::SmartDashboard::PutBoolean("Diagnostics/ZeroSwerveModules", false);
  frc::SmartDashboard::PutBoolean("Diagnostics/RotateSwerveModules", false);

  // Drive mode chooser - defaults to RobotCentric
  m_driveModeChooser.SetDefaultOption("RobotCentric", "RobotCentric");
  m_driveModeChooser.AddOption("FieldCentric", "FieldCentric");
  frc::SmartDashboard::PutData("Drive Mode", &m_driveModeChooser);
  frc::SmartDashboard::PutBoolean("Drive/FieldCentric", false);
  frc::SmartDashboard::PutBoolean("Drive/IsFieldCentric", false);

  // Robot orientation chooser - reverses forward and backward facing direction
  m_orientationChooser.SetDefaultOption("Forward (Normal)", "Forward");
  m_orientationChooser.AddOption("Reversed (180 deg)", "Reversed");
  frc::SmartDashboard::PutData("Robot Orientation", &m_orientationChooser);
  frc::SmartDashboard::PutBoolean("Drive/ReverseDirection", false);
  frc::SmartDashboard::PutBoolean("Drive/IsReversed", false);

  // Buttons to reset gyro heading and odometry from SmartDashboard / Reflect
  frc::SmartDashboard::PutBoolean("Reset Gyro", false);
  frc::SmartDashboard::PutBoolean("Drive/ResetGyro", false);
  frc::SmartDashboard::PutBoolean("Reset Field Centric Heading", false);
  frc::SmartDashboard::PutBoolean("Reset Odometry", false);
  frc::SmartDashboard::PutBoolean("Drive/ResetOdometry", false);

  // REV Servo Hub channels 2 & 3 setup
  m_servoChannel2.SetPowered(true);
  m_servoChannel2.SetEnabled(true);
  m_servoChannel3.SetPowered(true);
  m_servoChannel3.SetEnabled(true);
  frc::SmartDashboard::PutNumber("Servo/Channel2_CmdPulseWidth", 1500);
  frc::SmartDashboard::PutNumber("Servo/Channel3_CmdPulseWidth", 1500);
  frc::SmartDashboard::PutNumber("Servo/Channel2_Angle_Deg", 90.0);
  frc::SmartDashboard::PutNumber("Servo/Channel3_Angle_Deg", 90.0);
  frc::SmartDashboard::PutNumber("Servo/Channel2_PulseWidth_us", 1500);
  frc::SmartDashboard::PutNumber("Servo/Channel3_PulseWidth_us", 1500);

  // Configure swerve drivetrain motors and zero gyro heading on boot
  m_pigeon.Reset();
  m_swerve.ConfigureMotors();
}

void Robot::RobotPeriodic() {
  // Pigeon2 yaw is CCW-positive; ChimiSwerve convention is CLOCKWISE-positive.
  GyroValue = -std::fmod(m_pigeon.GetYaw().GetValueAsDouble(), 360.0);

  // Reset gyro heading: 'Touchpad' (box), 'Square' (box), 'Create', 'Options', or dashboard buttons
  bool dashResetField = frc::SmartDashboard::GetBoolean("Reset Field Centric Heading", false);
  bool dashResetGyro = frc::SmartDashboard::GetBoolean("Reset Gyro", false) ||
                       frc::SmartDashboard::GetBoolean("Drive/ResetGyro", false);
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
      frc::SmartDashboard::PutBoolean("Drive/ResetGyro", false);
    }
  }

  // Dashboard buttons to reset odometry
  if (frc::SmartDashboard::GetBoolean("Reset Odometry", false) ||
      frc::SmartDashboard::GetBoolean("Drive/ResetOdometry", false)) {
    m_swerve.ResetOdometry();
    frc::SmartDashboard::PutBoolean("Reset Odometry", false);
    frc::SmartDashboard::PutBoolean("Drive/ResetOdometry", false);
  }

  frc::SmartDashboard::PutBoolean("Driver/ResetButtonPressed", resetPressed);
  frc::SmartDashboard::PutNumber("Gyro Heading (CW deg)", GyroValue);
  frc::SmartDashboard::PutBoolean("Gyro/Connected", m_pigeon.IsConnected());
  frc::SmartDashboard::PutString("Controller/ActiveType", "PS5 DualSense");

  UpdateServoTelemetry();
  UpdateDiagnostics();
}

void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  wpi::print("Auto selected: {}\n", m_autoSelected);
  m_swerve.ResetHeading(GyroValue);
  m_swerve.ResetOdometry();

  // Reset timer and ensure mechanisms start clean
  m_timer.Restart();
  OuttakeLeft.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
  OuttakeRight.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
  BottomFeederMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
  TopFeederMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
}

void Robot::AutonomousPeriodic() {
  // Hold swerve drivetrain firmly in place (robot is positioned in front of hub)
  m_swerve.Update(0.0, 0.0, 0.0, GyroValue, 0.0, 0.0, false);

  // Check dashboard Auto enable/disable toggle
  bool autoEnabled = frc::SmartDashboard::GetBoolean("Auto/EnableAutoRoutine", true);
  frc::SmartDashboard::PutBoolean("Auto/IsAutoEnabled", autoEnabled);

  if (!autoEnabled || m_autoSelected == kAutoDoNothing) {
    OuttakeLeft.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
    OuttakeRight.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
    BottomFeederMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
    TopFeederMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
    frc::SmartDashboard::PutString("Auto/State", !autoEnabled ? "Disabled by Dashboard Toggle" : "Do Nothing");
    return;
  }

  // Preload shot routine with tunable speeds and feeder/outtake reversals
  const double t = m_timer.Get().value();
  frc::SmartDashboard::PutNumber("Auto/Timer_s", t);

  const double spinupSec = frc::SmartDashboard::GetNumber("Auto/SpinupSeconds", 1.0);
  const double feedDurationSec = frc::SmartDashboard::GetNumber("Auto/FeedDurationSeconds", 2.0);
  const double outtakeSpeed = frc::SmartDashboard::GetNumber("Auto/OuttakeSpeed", DrivetrainConstants::kOuttakeSpeed);
  const double feederSpeed = frc::SmartDashboard::GetNumber("Auto/FeederSpeed", DrivetrainConstants::kFeederSpeed);

  bool revTop = frc::SmartDashboard::GetBoolean("Diagnostics/ReverseTopFeeder", false);
  bool revBottom = frc::SmartDashboard::GetBoolean("Diagnostics/ReverseBottomFeeder", false);
  bool revOuttake = frc::SmartDashboard::GetBoolean("Diagnostics/ReverseOuttake", false);

  const double outtakeSign = revOuttake ? -1.0 : 1.0;
  const double topFeederSign = revTop ? 1.0 : -1.0;       // Normally -feederSpeed
  const double bottomFeederSign = revBottom ? -1.0 : 1.0; // Normally +feederSpeed

  if (t < spinupSec) {
    // Phase 1: Flywheels spin up, feeders hold
    OuttakeLeft.SetControl(ctre::phoenix6::controls::DutyCycleOut{outtakeSpeed * outtakeSign});
    OuttakeRight.SetControl(ctre::phoenix6::controls::DutyCycleOut{-outtakeSpeed * outtakeSign});
    BottomFeederMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
    TopFeederMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});

    frc::SmartDashboard::PutString("Auto/State", "Phase 1: Spinup Flywheels");
    frc::SmartDashboard::PutNumber("Auto/OuttakeLeftDuty", outtakeSpeed * outtakeSign);
    frc::SmartDashboard::PutNumber("Auto/OuttakeRightDuty", -outtakeSpeed * outtakeSign);
    frc::SmartDashboard::PutNumber("Auto/BottomFeederDuty", 0.0);
    frc::SmartDashboard::PutNumber("Auto/TopFeederDuty", 0.0);
  } else if (t < (spinupSec + feedDurationSec)) {
    // Phase 2: Feeders push preload through spinning flywheels
    OuttakeLeft.SetControl(ctre::phoenix6::controls::DutyCycleOut{outtakeSpeed * outtakeSign});
    OuttakeRight.SetControl(ctre::phoenix6::controls::DutyCycleOut{-outtakeSpeed * outtakeSign});
    BottomFeederMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{feederSpeed * bottomFeederSign});
    TopFeederMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{feederSpeed * topFeederSign});

    frc::SmartDashboard::PutString("Auto/State", "Phase 2: Feeding / Launching Preload");
    frc::SmartDashboard::PutNumber("Auto/OuttakeLeftDuty", outtakeSpeed * outtakeSign);
    frc::SmartDashboard::PutNumber("Auto/OuttakeRightDuty", -outtakeSpeed * outtakeSign);
    frc::SmartDashboard::PutNumber("Auto/BottomFeederDuty", feederSpeed * bottomFeederSign);
    frc::SmartDashboard::PutNumber("Auto/TopFeederDuty", feederSpeed * topFeederSign);
  } else {
    // Phase 3: Shot complete, idle mechanisms
    OuttakeLeft.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
    OuttakeRight.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
    BottomFeederMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
    TopFeederMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});

    frc::SmartDashboard::PutString("Auto/State", "Phase 3: Shot Complete (Idle)");
    frc::SmartDashboard::PutNumber("Auto/OuttakeLeftDuty", 0.0);
    frc::SmartDashboard::PutNumber("Auto/OuttakeRightDuty", 0.0);
    frc::SmartDashboard::PutNumber("Auto/BottomFeederDuty", 0.0);
    frc::SmartDashboard::PutNumber("Auto/TopFeederDuty", 0.0);
  }
}

void Robot::TeleopInit() {
  m_timer.Stop();
  m_swerve.ConfigureMotors();
  m_swerve.ResetHeading(GyroValue);

  // Stop mechanisms when entering teleop until commanded by driver
  OuttakeLeft.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
  OuttakeRight.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
  BottomFeederMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
  TopFeederMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
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

  // Reverse orientation toggle from SmartDashboard / Reflect (dropdown chooser or checkbox toggle)
  bool reverseFromChooser = (m_orientationChooser.GetSelected() == "Reversed");
  bool reverseFromToggle = frc::SmartDashboard::GetBoolean("Drive/ReverseDirection", false);
  bool reverseForward = reverseFromChooser || reverseFromToggle;
  if (reverseForward) {
    x = -x;
    y = -y;
  }
  frc::SmartDashboard::PutBoolean("Drive/IsReversed", reverseForward);

  // Drive mode (ChimiSwerve Sec 2.1a & Sec 3.7):
  // Can be toggled via SmartDashboard "Drive Mode" chooser or "Drive/FieldCentric" checkbox.
  // Automatically falls back to RobotCentric if Pigeon2 gyro is disconnected.
  const bool wantFieldFromChooser = (m_driveModeChooser.GetSelected() == "FieldCentric");
  const bool wantFieldFromToggle = frc::SmartDashboard::GetBoolean("Drive/FieldCentric", m_wantFieldCentric);
  if (wantFieldFromChooser != m_lastChooserField) {
    m_wantFieldCentric = wantFieldFromChooser;
    m_lastChooserField = wantFieldFromChooser;
    m_lastToggleField = m_wantFieldCentric;
    frc::SmartDashboard::PutBoolean("Drive/FieldCentric", m_wantFieldCentric);
  } else if (wantFieldFromToggle != m_lastToggleField) {
    m_wantFieldCentric = wantFieldFromToggle;
    m_lastToggleField = wantFieldFromToggle;
  }

  const bool gyroConnected = m_pigeon.IsConnected();
  FieldCentric = m_wantFieldCentric && gyroConnected;
  frc::SmartDashboard::PutBoolean("Drive/IsFieldCentric", FieldCentric);
  frc::SmartDashboard::PutBoolean("Gyro/FaultFallbackToRobotCentric", m_wantFieldCentric && !gyroConnected);

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

  // ========== Top Half Mechanism Controls (Controller Bumper or Dashboard Overrides) ==========
  bool topHalfBumper = GetDriverRightBumper();
  bool revTop = frc::SmartDashboard::GetBoolean("Diagnostics/ReverseTopFeeder", false);
  bool revBottom = frc::SmartDashboard::GetBoolean("Diagnostics/ReverseBottomFeeder", false);
  bool revOuttake = frc::SmartDashboard::GetBoolean("Diagnostics/ReverseOuttake", false);

  bool manualTop = frc::SmartDashboard::GetBoolean("Diagnostics/ManualRunTopFeeder", false);
  bool manualBottom = frc::SmartDashboard::GetBoolean("Diagnostics/ManualRunBottomFeeder", false);
  bool manualOuttake = frc::SmartDashboard::GetBoolean("Diagnostics/ManualRunOuttake", false);
  double manualSpeed = frc::SmartDashboard::GetNumber("Diagnostics/ManualTestSpeed", 0.50);

  double outtakeBaseSpeed = frc::SmartDashboard::GetNumber("TopHalf/OuttakeSpeed", DrivetrainConstants::kOuttakeSpeed);
  double feederBaseSpeed = frc::SmartDashboard::GetNumber("TopHalf/FeederSpeed", DrivetrainConstants::kFeederSpeed);

  double outtakeSign = revOuttake ? -1.0 : 1.0;
  double topFeederSign = revTop ? 1.0 : -1.0;       // Normally -feederSpeed
  double bottomFeederSign = revBottom ? -1.0 : 1.0; // Normally +feederSpeed

  frc::SmartDashboard::PutBoolean("TopHalf/BumperHeld", topHalfBumper);

  if (topHalfBumper) {
    OuttakeLeft.SetControl(ctre::phoenix6::controls::DutyCycleOut{outtakeBaseSpeed * outtakeSign});
    OuttakeRight.SetControl(ctre::phoenix6::controls::DutyCycleOut{-outtakeBaseSpeed * outtakeSign});
    BottomFeederMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{feederBaseSpeed * bottomFeederSign});
    TopFeederMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{feederBaseSpeed * topFeederSign});

    frc::SmartDashboard::PutNumber("TopHalf/OuttakeLeftDuty", outtakeBaseSpeed * outtakeSign);
    frc::SmartDashboard::PutNumber("TopHalf/OuttakeRightDuty", -outtakeBaseSpeed * outtakeSign);
    frc::SmartDashboard::PutNumber("TopHalf/BottomFeederDuty", feederBaseSpeed * bottomFeederSign);
    frc::SmartDashboard::PutNumber("TopHalf/TopFeederDuty", feederBaseSpeed * topFeederSign);
  } else if (manualTop || manualBottom || manualOuttake) {
    // Manual diagnostic overrides triggered from Reflect buttons/toggles
    double outDuty = (manualOuttake ? manualSpeed : 0.0) * outtakeSign;
    double topDuty = (manualTop ? manualSpeed : 0.0) * (revTop ? 1.0 : -1.0);
    double btmDuty = (manualBottom ? manualSpeed : 0.0) * (revBottom ? -1.0 : 1.0);

    OuttakeLeft.SetControl(ctre::phoenix6::controls::DutyCycleOut{outDuty});
    OuttakeRight.SetControl(ctre::phoenix6::controls::DutyCycleOut{-outDuty});
    BottomFeederMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{btmDuty});
    TopFeederMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{topDuty});

    frc::SmartDashboard::PutNumber("TopHalf/OuttakeLeftDuty", outDuty);
    frc::SmartDashboard::PutNumber("TopHalf/OuttakeRightDuty", -outDuty);
    frc::SmartDashboard::PutNumber("TopHalf/BottomFeederDuty", btmDuty);
    frc::SmartDashboard::PutNumber("TopHalf/TopFeederDuty", topDuty);
  } else {
    OuttakeLeft.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
    OuttakeRight.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
    BottomFeederMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
    TopFeederMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});

    frc::SmartDashboard::PutNumber("TopHalf/OuttakeLeftDuty", 0.0);
    frc::SmartDashboard::PutNumber("TopHalf/OuttakeRightDuty", 0.0);
    frc::SmartDashboard::PutNumber("TopHalf/BottomFeederDuty", 0.0);
    frc::SmartDashboard::PutNumber("TopHalf/TopFeederDuty", 0.0);
  }
}

void Robot::DisabledInit() {
  m_timer.Stop();
  m_timer.Reset();
  m_swerve.DisabledInit();
  BottomFeederMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
  TopFeederMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
  OuttakeLeft.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
  OuttakeRight.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
}

void Robot::DisabledPeriodic() {
  // If zero swerve or rotate swerve button clicked from Reflect dashboard, run routines
  if (frc::SmartDashboard::GetBoolean("Diagnostics/ZeroSwerveModules", false)) {
    m_swerve.TestZeroAzimuth();
  } else if (frc::SmartDashboard::GetBoolean("Diagnostics/RotateSwerveModules", false)) {
    m_swerve.TestRotateModules(0.02);
  } else {
    m_swerve.DisabledPeriodic();
  }
}

void Robot::TestInit() {
  if (!m_swerve.AreConfigsApplied()) {
    m_swerve.ConfigureMotors();
  }
}

// ChimiSwerve Section 3.1 (Page 35): Zero Azimuth Position & Pre-Match Module Testing
//   - Hold Cross (X) on PS5 controller or Diagnostics/ZeroSwerveModules on dashboard: command all wheels to 0 deg
//   - Hold Circle (O) on PS5 controller or Diagnostics/RotateSwerveModules on dashboard: slowly rotate all wheels
void Robot::TestPeriodic() {
  bool dashZero = frc::SmartDashboard::GetBoolean("Diagnostics/ZeroSwerveModules", false);
  bool dashRotate = frc::SmartDashboard::GetBoolean("Diagnostics/RotateSwerveModules", false);

  if (GetDriverCrossButton() || dashZero) {
    m_swerve.TestZeroAzimuth();
  } else if (GetDriverCircleButton() || dashRotate) {
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

bool Robot::GetDriverRightBumper() const {
  int port = GetDriverPort();
  if (port == 1) return m_ps5Controller1.GetR1Button();
  return m_ps5Controller0.GetR1Button();
}

bool Robot::GetDriverRightBumperPressed() {
  int port = GetDriverPort();
  if (port == 1) return m_ps5Controller1.GetR1ButtonPressed();
  return m_ps5Controller0.GetR1ButtonPressed();
}

bool Robot::GetDriverLeftBumper() const {
  int port = GetDriverPort();
  if (port == 1) return m_ps5Controller1.GetL1Button();
  return m_ps5Controller0.GetL1Button();
}

bool Robot::GetDriverLeftBumperPressed() {
  int port = GetDriverPort();
  if (port == 1) return m_ps5Controller1.GetL1ButtonPressed();
  return m_ps5Controller0.GetL1ButtonPressed();
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

// ========== Dashboard & Diagnostics Helpers (Reflect compatible) ==========

void Robot::UpdateServoTelemetry() {
  // Read current pulse width in microseconds from REV Servo Hub
  int pw2 = m_servoChannel2.GetPulseWidth();
  int pw3 = m_servoChannel3.GetPulseWidth();

  // Convert pulse width to degrees: standard 1000us (0 deg) to 2000us (180 deg)
  // or clamped across 500-2500us
  double deg2 = (pw2 > 0) ? std::clamp((static_cast<double>(pw2) - 1000.0) * 180.0 / 1000.0, 0.0, 180.0) : 90.0;
  double deg3 = (pw3 > 0) ? std::clamp((static_cast<double>(pw3) - 1000.0) * 180.0 / 1000.0, 0.0, 180.0) : 90.0;

  frc::SmartDashboard::PutNumber("Servo/Channel2_PulseWidth_us", pw2);
  frc::SmartDashboard::PutNumber("Servo/Channel2_Angle_Deg", deg2);
  frc::SmartDashboard::PutNumber("Servo/Channel2_Current_A", m_servoChannel2.GetCurrent());
  frc::SmartDashboard::PutBoolean("Servo/Channel2_Enabled", m_servoChannel2.IsEnabled());

  frc::SmartDashboard::PutNumber("Servo/Channel3_PulseWidth_us", pw3);
  frc::SmartDashboard::PutNumber("Servo/Channel3_Angle_Deg", deg3);
  frc::SmartDashboard::PutNumber("Servo/Channel3_Current_A", m_servoChannel3.GetCurrent());
  frc::SmartDashboard::PutBoolean("Servo/Channel3_Enabled", m_servoChannel3.IsEnabled());

  frc::SmartDashboard::PutNumber("Servo/Hub_Voltage_V", m_servoHub.GetDeviceVoltage());
  frc::SmartDashboard::PutNumber("Servo/Hub_Current_A", m_servoHub.GetDeviceCurrent());

  // Check if operator adjusted target pulse width via Reflect Slider widget
  int cmdPw2 = static_cast<int>(frc::SmartDashboard::GetNumber("Servo/Channel2_CmdPulseWidth", 1500));
  int cmdPw3 = static_cast<int>(frc::SmartDashboard::GetNumber("Servo/Channel3_CmdPulseWidth", 1500));
  if (cmdPw2 >= 500 && cmdPw2 <= 2500 && std::abs(cmdPw2 - pw2) > 5) {
    m_servoChannel2.SetPulseWidth(cmdPw2);
  }
  if (cmdPw3 >= 500 && cmdPw3 <= 2500 && std::abs(cmdPw3 - pw3) > 5) {
    m_servoChannel3.SetPulseWidth(cmdPw3);
  }
}

void Robot::UpdateDiagnostics() {
  // Battery & Power Rails
  frc::SmartDashboard::PutNumber("Diagnostics/BatteryVoltage_V", frc::RobotController::GetBatteryVoltage().value());
  frc::SmartDashboard::PutNumber("Diagnostics/RoboRIO_InputVoltage_V", frc::RobotController::GetInputVoltage());
  frc::SmartDashboard::PutBoolean("Diagnostics/RoboRIO_3V3Rail_OK", frc::RobotController::GetEnabled3V3());
  frc::SmartDashboard::PutBoolean("Diagnostics/RoboRIO_5VRail_OK", frc::RobotController::GetEnabled5V());
  frc::SmartDashboard::PutBoolean("Diagnostics/RoboRIO_6VRail_OK", frc::RobotController::GetEnabled6V());

  // Game & Match Information
  frc::SmartDashboard::PutNumber("Diagnostics/MatchTime_s", frc::DriverStation::GetMatchTime().value());
  frc::SmartDashboard::PutBoolean("Diagnostics/FMS_Connected", frc::DriverStation::IsFMSAttached());
  frc::SmartDashboard::PutBoolean("Diagnostics/DS_Connected", frc::DriverStation::IsDSAttached());

  std::string mode = "Disabled";
  if (IsAutonomousEnabled()) mode = "Autonomous";
  else if (IsTeleopEnabled()) mode = "Teleop";
  else if (IsTest()) mode = "Test";
  frc::SmartDashboard::PutString("Diagnostics/RobotState", mode);

  // Pigeon2 & Orientation
  frc::SmartDashboard::PutBoolean("Diagnostics/Pigeon2_Connected", m_pigeon.IsConnected());
  frc::SmartDashboard::PutNumber("Diagnostics/Pigeon2_Yaw_deg", GyroValue);
  frc::SmartDashboard::PutNumber("Diagnostics/Pigeon2_Pitch_deg", m_pigeon.GetPitch().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Diagnostics/Pigeon2_Roll_deg", m_pigeon.GetRoll().GetValueAsDouble());

  // CANcoder Connection Statuses
  frc::SmartDashboard::PutBoolean("Diagnostics/CANcoder_FL_OK", m_swerve.IsCANcoderFLConnected());
  frc::SmartDashboard::PutBoolean("Diagnostics/CANcoder_FR_OK", m_swerve.CANcoderFR.IsConnected());
  frc::SmartDashboard::PutBoolean("Diagnostics/CANcoder_BL_OK", m_swerve.CANcoderBL.IsConnected());
  frc::SmartDashboard::PutBoolean("Diagnostics/CANcoder_BR_OK", m_swerve.CANcoderBR.IsConnected());

  // Top Half Mechanism Motors (CAN bus: "Tophalf")
  frc::SmartDashboard::PutNumber("Diagnostics/TopFeeder_Current_A", TopFeederMotor.GetStatorCurrent().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Diagnostics/TopFeeder_Temp_C", TopFeederMotor.GetDeviceTemp().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Diagnostics/TopFeeder_DutyCycle", TopFeederMotor.GetDutyCycle().GetValueAsDouble());

  frc::SmartDashboard::PutNumber("Diagnostics/BottomFeeder_Current_A", BottomFeederMotor.GetStatorCurrent().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Diagnostics/BottomFeeder_Temp_C", BottomFeederMotor.GetDeviceTemp().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Diagnostics/BottomFeeder_DutyCycle", BottomFeederMotor.GetDutyCycle().GetValueAsDouble());

  frc::SmartDashboard::PutNumber("Diagnostics/OuttakeLeft_Current_A", OuttakeLeft.GetStatorCurrent().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Diagnostics/OuttakeLeft_Temp_C", OuttakeLeft.GetDeviceTemp().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Diagnostics/OuttakeLeft_DutyCycle", OuttakeLeft.GetDutyCycle().GetValueAsDouble());

  frc::SmartDashboard::PutNumber("Diagnostics/OuttakeRight_Current_A", OuttakeRight.GetStatorCurrent().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Diagnostics/OuttakeRight_Temp_C", OuttakeRight.GetDeviceTemp().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Diagnostics/OuttakeRight_DutyCycle", OuttakeRight.GetDutyCycle().GetValueAsDouble());

  // Drive Mode & Modules Summary
  frc::SmartDashboard::PutString("Diagnostics/Drive_Mode", FieldCentric ? "FieldCentric" : "RobotCentric");
  frc::SmartDashboard::PutString("Diagnostics/Active_Modules", "FL (30), FR (20), BL (40), BR (10)");
  frc::SmartDashboard::PutString("Diagnostics/FL_Status", "Active (Swerve 3 Connected)");

  // ==================== Alert Updates (Reflect & Driver Station) ====================
  // Battery Alerts
  double batteryVolts = frc::RobotController::GetBatteryVoltage().value();
  m_alertBatteryCritical.Set(batteryVolts > 0.0 && batteryVolts < 10.5);
  m_alertBatteryLow.Set(batteryVolts >= 10.5 && batteryVolts < 12.0);

  // Hardware Connection Errors
  bool pigeonConnected = m_pigeon.IsConnected();
  m_alertPigeonDisconnected.Set(!pigeonConnected);
  m_alertCANcoderFL.Set(!m_swerve.IsCANcoderFLConnected());
  m_alertCANcoderFR.Set(!m_swerve.CANcoderFR.IsConnected());
  m_alertCANcoderBL.Set(!m_swerve.CANcoderBL.IsConnected());
  m_alertCANcoderBR.Set(!m_swerve.CANcoderBR.IsConnected());

  // Operational Warnings
  bool autoEnabled = frc::SmartDashboard::GetBoolean("Auto/EnableAutoRoutine", true);
  m_alertAutoDisabled.Set(!autoEnabled);

  bool reverseDrive = frc::SmartDashboard::GetBoolean("Drive/ReverseDirection", false);
  m_alertDriveReversed.Set(reverseDrive);

  bool revTop = frc::SmartDashboard::GetBoolean("Diagnostics/ReverseTopFeeder", false);
  bool revBottom = frc::SmartDashboard::GetBoolean("Diagnostics/ReverseBottomFeeder", false);
  bool revOuttake = frc::SmartDashboard::GetBoolean("Diagnostics/ReverseOuttake", false);
  m_alertTopFeederReversed.Set(revTop);
  m_alertBottomFeederReversed.Set(revBottom);
  m_alertOuttakeReversed.Set(revOuttake);

  bool manualRunTop = frc::SmartDashboard::GetBoolean("Diagnostics/ManualRunTopFeeder", false);
  bool manualRunBottom = frc::SmartDashboard::GetBoolean("Diagnostics/ManualRunBottomFeeder", false);
  bool manualRunOuttake = frc::SmartDashboard::GetBoolean("Diagnostics/ManualRunOuttake", false);
  m_alertManualTestingActive.Set(manualRunTop || manualRunBottom || manualRunOuttake);

  // Status Infos
  m_alertFieldCentricActive.Set(FieldCentric);
  m_alertRobotCentricActive.Set(!FieldCentric);
  m_alertSwerveAligned.Set(m_swerve.AreWheelsAligned());
  m_alertFMSConnected.Set(frc::DriverStation::IsFMSAttached());
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
