// ============================================================================
// FRC Team 8753 - 2026 Swerve Drivetrain (Restored from swerve-base / 2026_Bot)
// Runs Swerve 1 (BR 10/11/12), Swerve 2 (FR 20/21/22), Swerve 3 (FL 30/31/32),
// and Swerve 4 (BL 40/41/42) - all 4 swerve modules active
// ============================================================================

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <algorithm>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/time.h>

#include "Drivetrain.h"
#include "DrivetrainConstants.h"

using namespace ctre::phoenix6;
using namespace units::literals;
namespace DC = DrivetrainConstants;

// CANcoders unwired and disabled
double Drivetrain::GetCANcoderFLTurns() {
  return 0.0;
}

bool Drivetrain::IsCANcoderFLConnected() {
  return false;
}

Drivetrain::Drivetrain() {
  positionFWDField = units::length::meter_t(0);
  positionSTRField = units::length::meter_t(0);

  ConfigureMotors();
}

void Drivetrain::ConfigureMotors() {
  // Drive Krakens configurations: all 4 modules (BR, FR, FL, BL) active in Brake mode
  // ChimiSwerve Section 3.6 (Pages 40-41): Ramp rate + current limiting prevent brownouts
  driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = units::time::second_t(0.5);
  driveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = units::time::second_t(DC::DriveOpenLoopRamp);
  driveConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
  driveConfig.CurrentLimits.StatorCurrentLimit = units::current::ampere_t{DC::kDriveCurrentLimit};
  driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;

  bool swapM2 = frc::SmartDashboard::GetBoolean("Drive/Swap_Module2_IDs", false);
  hardware::TalonFX& frDrive = swapM2 ? m_FR_Steer : m_FR_Drive;
  hardware::TalonFX& frSteer = swapM2 ? m_FR_Drive : m_FR_Steer;

  m_FL_Drive.GetConfigurator().Apply(driveConfig, 100_ms);
  frDrive.GetConfigurator().Apply(driveConfig, 100_ms);
  m_BL_Drive.GetConfigurator().Apply(driveConfig, 100_ms);
  m_BR_Drive.GetConfigurator().Apply(driveConfig, 100_ms);

  // Steer motors fully disabled: pure open-loop RotorSensor mode, Brake mode, 0 output
  // Steer modules are manually aligned straight and physically locked via Brake mode
  ctre::phoenix6::configs::TalonFXConfiguration steerBrakeConfig{};
  steerBrakeConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
  steerBrakeConfig.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor;
  steerBrakeConfig.CurrentLimits.StatorCurrentLimit = units::current::ampere_t{DC::kSteerCurrentLimit};
  steerBrakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;

  auto flStatus = m_FL_Steer.GetConfigurator().Apply(steerBrakeConfig, 100_ms);
  frSteer.GetConfigurator().Apply(steerBrakeConfig, 100_ms);
  m_BL_Steer.GetConfigurator().Apply(steerBrakeConfig, 100_ms);
  m_BR_Steer.GetConfigurator().Apply(steerBrakeConfig, 100_ms);

  m_FL_Steer.SetControl(ctre::phoenix6::controls::NeutralOut{});
  frSteer.SetControl(ctre::phoenix6::controls::NeutralOut{});
  m_BL_Steer.SetControl(ctre::phoenix6::controls::NeutralOut{});
  m_BR_Steer.SetControl(ctre::phoenix6::controls::NeutralOut{});

  frc::SmartDashboard::PutString("Diagnostics/FL_Steer_ConfigStatus", flStatus.GetName());

  // Publish default dashboard properties
  static bool s_dashInitialized = false;
  if (!s_dashInitialized) {
    frc::SmartDashboard::PutBoolean("Drive/Swap_Module2_IDs", false);
    frc::SmartDashboard::PutBoolean("Drive/Invert_BR_Drive", DC::kBRDriveInverted);
    frc::SmartDashboard::PutBoolean("Drive/Invert_FR_Drive", false);
    frc::SmartDashboard::PutNumber("Tank/TurnSensitivity", 0.75);
    frc::SmartDashboard::PutNumber("Tank/DriveBaseSpeed", 0.50);
    frc::SmartDashboard::PutNumber("Swerve/Steer_kP", ModuleP);
    frc::SmartDashboard::PutNumber("Swerve/Steer_kI", ModuleI);
    frc::SmartDashboard::PutNumber("Swerve/Steer_kD", ModuleD);
    frc::SmartDashboard::PutNumber("Swerve/Steer_kS", ModuleS);
    frc::SmartDashboard::PutNumber("Swerve/Steer_RampSeconds", ModuleRampSeconds);
    frc::SmartDashboard::PutNumber("Swerve/Steer_PeakOutput", ModulePeakOutput);
    frc::SmartDashboard::PutNumber("Swerve/Steer_DeadbandTurns", DC::kSteerDeadbandTurns);
    frc::SmartDashboard::PutBoolean("Swerve/EnableInversionAwareness", true);
    frc::SmartDashboard::PutBoolean("Swerve/EnableHeadingLock", DC::kEnableHeadingLock);
    frc::SmartDashboard::PutNumber("Swerve/HeadingLock_kP", straightP);
    s_dashInitialized = true;
  }
}


void Drivetrain::SyncSmartDashboardPID() {
  const float dashP = static_cast<float>(frc::SmartDashboard::GetNumber("Swerve/Steer_kP", ModuleP));
  const float dashI = static_cast<float>(frc::SmartDashboard::GetNumber("Swerve/Steer_kI", ModuleI));
  const float dashD = static_cast<float>(frc::SmartDashboard::GetNumber("Swerve/Steer_kD", ModuleD));
  const float dashS = static_cast<float>(std::clamp(frc::SmartDashboard::GetNumber("Swerve/Steer_kS", ModuleS), 0.0, 0.20));
  const float dashRamp = static_cast<float>(std::clamp(frc::SmartDashboard::GetNumber("Swerve/Steer_RampSeconds", ModuleRampSeconds), 0.0, 0.5));
  const float dashPeak = static_cast<float>(std::clamp(frc::SmartDashboard::GetNumber("Swerve/Steer_PeakOutput", ModulePeakOutput), 0.05, 1.0));
  const float dashStraightP = static_cast<float>(frc::SmartDashboard::GetNumber("Swerve/HeadingLock_kP", straightP));

  if (std::abs(dashStraightP - straightP) > 1e-5f) {
    straightP = dashStraightP;
  }

  if (std::abs(dashP - ModuleP) > 1e-4f ||
      std::abs(dashI - ModuleI) > 1e-4f ||
      std::abs(dashD - ModuleD) > 1e-4f ||
      std::abs(dashS - ModuleS) > 1e-4f) {
    ModuleP = dashP;
    ModuleI = dashI;
    ModuleD = dashD;
    ModuleS = dashS;
    Posconfig.kP = ModuleP;
    Posconfig.kI = ModuleI;
    Posconfig.kD = ModuleD;
    Posconfig.kS = ModuleS;
    // When swerve is fully disabled, steer motors remain locked in open-loop RotorSensor Brake mode
  }

  if (std::abs(dashRamp - ModuleRampSeconds) > 1e-4f ||
      std::abs(dashPeak - ModulePeakOutput) > 1e-4f) {
    ModuleRampSeconds = dashRamp;
    ModulePeakOutput = dashPeak;
    // When swerve is fully disabled, steer motors remain locked in open-loop RotorSensor Brake mode
  }
}

void Drivetrain::DisabledInit() {
  bool swapM2 = frc::SmartDashboard::GetBoolean("Drive/Swap_Module2_IDs", false);
  hardware::TalonFX& frDrive = swapM2 ? m_FR_Steer : m_FR_Drive;
  hardware::TalonFX& frSteer = swapM2 ? m_FR_Drive : m_FR_Steer;

  m_FL_Drive.Set(0.0);
  frDrive.Set(0.0);
  m_BL_Drive.Set(0.0);
  m_BR_Drive.Set(0.0);
  m_FL_Steer.SetControl(ctre::phoenix6::controls::NeutralOut{});
  frSteer.SetControl(ctre::phoenix6::controls::NeutralOut{});
  m_BL_Steer.SetControl(ctre::phoenix6::controls::NeutralOut{});
  m_BR_Steer.SetControl(ctre::phoenix6::controls::NeutralOut{});
  oldspeedFL = 0; oldspeedFR = 0; oldspeedBL = 0; oldspeedBR = 0;
  m_flipFL = false; m_flipFR = false; m_flipBL = false; m_flipBR = false;
  m_wasDriving = false;
  m_steerActiveFL = false; m_steerActiveFR = false; m_steerActiveBL = false; m_steerActiveBR = false;
  iError = 0.0;

  SyncSmartDashboardPID();
}

void Drivetrain::DisabledPeriodic() {
  bool swapM2 = frc::SmartDashboard::GetBoolean("Drive/Swap_Module2_IDs", false);
  hardware::TalonFX& frDrive = swapM2 ? m_FR_Steer : m_FR_Drive;
  hardware::TalonFX& frSteer = swapM2 ? m_FR_Drive : m_FR_Steer;

  m_FL_Drive.Set(0.0);
  frDrive.Set(0.0);
  m_BL_Drive.Set(0.0);
  m_BR_Drive.Set(0.0);
  m_FL_Steer.SetControl(ctre::phoenix6::controls::NeutralOut{});
  frSteer.SetControl(ctre::phoenix6::controls::NeutralOut{});
  m_BL_Steer.SetControl(ctre::phoenix6::controls::NeutralOut{});
  m_BR_Steer.SetControl(ctre::phoenix6::controls::NeutralOut{});
  m_wasDriving = false;

  SyncSmartDashboardPID();

  // CANcoders are unwired; modules are manually aligned straight and locked in Brake mode
  const double curFLTurns = 0.0;
  const double curFRTurns = 0.0;
  const double curBLTurns = 0.0;
  const double curBRTurns = 0.0;

  const double curFLDeg = 0.0;
  const double curFRDeg = 0.0;
  const double curBLDeg = 0.0;
  const double curBRDeg = 0.0;

  frc::SmartDashboard::PutNumber("Swerve/FL_Angle_Deg", curFLDeg);
  frc::SmartDashboard::PutNumber("Swerve/FR_Angle_Deg", curFRDeg);
  frc::SmartDashboard::PutNumber("Swerve/BL_Angle_Deg", curBLDeg);
  frc::SmartDashboard::PutNumber("Swerve/BR_Angle_Deg", curBRDeg);

  frc::SmartDashboard::PutNumber("Swerve/FL_ZeroedTurns", curFLTurns);
  frc::SmartDashboard::PutNumber("Swerve/FR_ZeroedTurns", curFRTurns);
  frc::SmartDashboard::PutNumber("Swerve/BL_ZeroedTurns", curBLTurns);
  frc::SmartDashboard::PutNumber("Swerve/BR_ZeroedTurns", curBRTurns);

  // Raw CANcoder absolute readings (unwired)
  frc::SmartDashboard::PutNumber("Swerve/CANcoder_FL_RawTurns", 0.0);
  frc::SmartDashboard::PutNumber("Swerve/CANcoder_FR_RawTurns", 0.0);
  frc::SmartDashboard::PutNumber("Swerve/CANcoder_BL_RawTurns", 0.0);
  frc::SmartDashboard::PutNumber("Swerve/CANcoder_BR_RawTurns", 0.0);

  // Swerve 3 (CAN 31) hardware diagnostics
  frc::SmartDashboard::PutNumber("Diagnostics/FL_Steer_Temp_C", m_FL_Steer.GetDeviceTemp().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Diagnostics/FL_Steer_Current_A", m_FL_Steer.GetStatorCurrent().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Diagnostics/FL_Steer_DutyCycle", m_FL_Steer.GetDutyCycle().GetValueAsDouble());

  // Wheel straight alignment indicator (|angle| < 3 degrees)
  bool wheelsStraight = (std::abs(curFLDeg) < 3.0) &&
                        (std::abs(curFRDeg) < 3.0) &&
                        (std::abs(curBLDeg) < 3.0) &&
                        (std::abs(curBRDeg) < 3.0);
  frc::SmartDashboard::PutBoolean("Swerve/All_Wheels_Straight", wheelsStraight);

  // Swerve module state arrays formatted for Reflect / AdvantageScope / Shuffleboard
  std::vector<double> measuredStates = {
      curFLTurns * (2.0 * M_PI), 0.0,
      curFRTurns * (2.0 * M_PI), 0.0,
      curBLTurns * (2.0 * M_PI), 0.0,
      curBRTurns * (2.0 * M_PI), 0.0
  };
  std::vector<double> desiredStates = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> moduleAnglesDeg = {curFLDeg, curFRDeg, curBLDeg, curBRDeg};
  std::vector<double> moduleSpeedsMps = {0.0, 0.0, 0.0, 0.0};

  frc::SmartDashboard::PutNumberArray("Swerve/MeasuredStates", measuredStates);
  frc::SmartDashboard::PutNumberArray("Swerve/DesiredStates", desiredStates);
  frc::SmartDashboard::PutNumberArray("Swerve/ModuleAngles_deg", moduleAnglesDeg);
  frc::SmartDashboard::PutNumberArray("Swerve/ModuleSpeeds_mps", moduleSpeedsMps);
}

void Drivetrain::TestZeroAzimuth() {
  bool swapM2 = frc::SmartDashboard::GetBoolean("Drive/Swap_Module2_IDs", false);
  hardware::TalonFX& frDrive = swapM2 ? m_FR_Steer : m_FR_Drive;
  hardware::TalonFX& frSteer = swapM2 ? m_FR_Drive : m_FR_Steer;

  m_FL_Steer.SetControl(ctre::phoenix6::controls::NeutralOut{});
  frSteer.SetControl(ctre::phoenix6::controls::NeutralOut{});
  m_BL_Steer.SetControl(ctre::phoenix6::controls::NeutralOut{});
  m_BR_Steer.SetControl(ctre::phoenix6::controls::NeutralOut{});
  m_FL_Drive.Set(0.0);
  frDrive.Set(0.0);
  m_BL_Drive.Set(0.0);
  m_BR_Drive.Set(0.0);
}

void Drivetrain::TestRotateModules(double speed) {
  bool swapM2 = frc::SmartDashboard::GetBoolean("Drive/Swap_Module2_IDs", false);
  hardware::TalonFX& frDrive = swapM2 ? m_FR_Steer : m_FR_Drive;
  hardware::TalonFX& frSteer = swapM2 ? m_FR_Drive : m_FR_Steer;

  // Swerve is disabled: Steer motors never spin, strictly held in Brake mode
  m_FL_Steer.SetControl(ctre::phoenix6::controls::NeutralOut{});
  frSteer.SetControl(ctre::phoenix6::controls::NeutralOut{});
  m_BL_Steer.SetControl(ctre::phoenix6::controls::NeutralOut{});
  m_BR_Steer.SetControl(ctre::phoenix6::controls::NeutralOut{});
  m_FL_Drive.Set(0.0);
  frDrive.Set(0.0);
  m_BL_Drive.Set(0.0);
  m_BR_Drive.Set(0.0);
}

void Drivetrain::Update(double x, double y, double x2, double GyroValue,
                        double triggerL, double triggerR, bool FieldCentric,
                        bool isTankDrive, bool holdSteer) {
  bool swapM2 = frc::SmartDashboard::GetBoolean("Drive/Swap_Module2_IDs", false);
  hardware::TalonFX& frDrive = swapM2 ? m_FR_Steer : m_FR_Drive;
  hardware::TalonFX& frSteer = swapM2 ? m_FR_Drive : m_FR_Steer;

  // Steer motors permanently commanded 0.0 power in Brake mode
  // Modules stay locked mechanically where manually aligned before match start
  m_FL_Steer.SetControl(ctre::phoenix6::controls::NeutralOut{});
  frSteer.SetControl(ctre::phoenix6::controls::NeutralOut{});
  m_BL_Steer.SetControl(ctre::phoenix6::controls::NeutralOut{});
  m_BR_Steer.SetControl(ctre::phoenix6::controls::NeutralOut{});

  // Live Swerve 3 (CAN 31) & Swerve 2 (CAN 20/21) diagnostics while driving
  frc::SmartDashboard::PutNumber("Diagnostics/FL_Steer_Temp_C", m_FL_Steer.GetDeviceTemp().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Diagnostics/FL_Steer_Current_A", m_FL_Steer.GetStatorCurrent().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Diagnostics/FL_Steer_DutyCycle", m_FL_Steer.GetDutyCycle().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Diagnostics/FR_Steer_Current_A", frSteer.GetStatorCurrent().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Diagnostics/FR_Drive_Current_A", frDrive.GetStatorCurrent().GetValueAsDouble());

  // ========== TANK DRIVE ONLY (Swerve Fully Disabled) ==========
  // Forward/backward throttle from Left Stick Y (y)
  // Turn: Right Stick X (x2) with priority; Left Stick X (x) with intentional deflection (> 0.15)
  double throttle = y;
  const double turnSensitivity = frc::SmartDashboard::GetNumber("Tank/TurnSensitivity", 0.75);
  double turn = 0.0;
  if (std::abs(x2) > 0.05) {
    turn = x2 * turnSensitivity;
  } else if (std::abs(x) > 0.15) {
    turn = std::copysign((std::abs(x) - 0.15) / 0.85, x) * (turnSensitivity * 0.85);
  }

  // Smooth target calculation: when sticks are released, targets are 0.0,
  // allowing the rate-limiter to ramp down smoothly without jarring duty cycle steps or motor pulsing
  double leftTarget = 0.0;
  double rightTarget = 0.0;
  if (std::abs(throttle) > 1e-4 || std::abs(turn) > 1e-4) {
    auto [l, r] = TankDriveKinematics(throttle, turn);
    leftTarget = l;
    rightTarget = r;
  }

  speedFL = leftTarget;
  speedBL = leftTarget;
  speedFR = rightTarget;
  speedBR = rightTarget;

  frc::SmartDashboard::PutNumber("Tank/LeftSpeed", leftTarget);
  frc::SmartDashboard::PutNumber("Tank/RightSpeed", rightTarget);
  frc::SmartDashboard::PutBoolean("Drive/IsTankDrive", true);

  // Smooth acceleration & deceleration ramps (prevents aggressive pulsation / stuttering)
  constexpr double kRampStep = 0.06;
  auto updateRamp = [kRampStep](double target, double& current) {
    double err = target - current;
    if (err >= kRampStep) {
      current += kRampStep;
    } else if (err <= -kRampStep) {
      current -= kRampStep;
    } else {
      current = target;
    }
    // Snap cleanly to zero when stopping to eliminate micro-creeping
    if (std::abs(target) < 1e-4 && std::abs(current) < 0.02) {
      current = 0.0;
    }
  };

  updateRamp(speedFL, oldspeedFL);
  updateRamp(speedFR, oldspeedFR);
  updateRamp(speedBL, oldspeedBL);
  updateRamp(speedBR, oldspeedBR);

  // Speed cap with triggers:
  // Base speed = 50%, R2 boosts up to 75% (DriveMotorsHardLimit), L2 slows down to 20%
  const double baseSpeed = frc::SmartDashboard::GetNumber("Tank/DriveBaseSpeed", 0.50);
  const double cap = std::clamp(baseSpeed + 0.25 * triggerR - 0.30 * triggerL, 0.20, DC::DriveMotorsHardLimit);

  // BR motor inversion:
  // On CLEO chassis, BR drive motor is mechanically inverted (kBRDriveInverted = true).
  // Left side: FL and BL are negated (-oldspeedFL, -oldspeedBL).
  // Right side: FR is positive (+oldspeedFR), BR is negated (-oldspeedBR) to match FR forward direction.
  bool invertBR = frc::SmartDashboard::GetBoolean("Drive/Invert_BR_Drive", DC::kBRDriveInverted);
  bool invertFR = frc::SmartDashboard::GetBoolean("Drive/Invert_FR_Drive", false);

  double cmdFL = std::clamp(-oldspeedFL * cap, -cap, cap);
  double cmdFR = std::clamp((invertFR ? -oldspeedFR : oldspeedFR) * cap, -cap, cap);
  double cmdBL = std::clamp(-oldspeedBL * cap, -cap, cap);
  double cmdBR = std::clamp((invertBR ? -oldspeedBR : oldspeedBR) * cap, -cap, cap);

  // When stopped, enforce true 0.0
  if (std::abs(oldspeedFL) < 1e-4) cmdFL = 0.0;
  if (std::abs(oldspeedFR) < 1e-4) cmdFR = 0.0;
  if (std::abs(oldspeedBL) < 1e-4) cmdBL = 0.0;
  if (std::abs(oldspeedBR) < 1e-4) cmdBR = 0.0;

  m_FL_Drive.Set(cmdFL);
  frDrive.Set(cmdFR);
  m_BL_Drive.Set(cmdBL);
  m_BR_Drive.Set(cmdBR);

  frc::SmartDashboard::PutNumber("Tank/Cmd_FL", cmdFL);
  frc::SmartDashboard::PutNumber("Tank/Cmd_FR", cmdFR);
  frc::SmartDashboard::PutNumber("Tank/Cmd_BL", cmdBL);
  frc::SmartDashboard::PutNumber("Tank/Cmd_BR", cmdBR);

  double wheelSpeedFL = (m_FL_Drive.GetVelocity().GetValueAsDouble() / 6.0375) * (M_PI * 0.0889);
  double wheelSpeedFR = (frDrive.GetVelocity().GetValueAsDouble() / 6.0375) * (M_PI * 0.0889);
  double wheelSpeedBL = (m_BL_Drive.GetVelocity().GetValueAsDouble() / 6.0375) * (M_PI * 0.0889);
  double wheelSpeedBR = (m_BR_Drive.GetVelocity().GetValueAsDouble() / 6.0375) * (M_PI * 0.0889);

  odometryUpdate(0.0, 0.0, 0.0, 0.0, wheelSpeedFL, wheelSpeedFR, wheelSpeedBL, wheelSpeedBR, GyroValue);

  // Swerve module state arrays formatted for Reflect / AdvantageScope
  std::vector<double> measuredStates = {
      0.0, wheelSpeedFL,
      0.0, wheelSpeedFR,
      0.0, wheelSpeedBL,
      0.0, wheelSpeedBR
  };
  std::vector<double> desiredStates = {
      0.0, speedFL * 4.0,
      0.0, speedFR * 4.0,
      0.0, speedBL * 4.0,
      0.0, speedBR * 4.0
  };
  std::vector<double> moduleAnglesDeg = {0.0, 0.0, 0.0, 0.0};
  std::vector<double> moduleSpeedsMps = {wheelSpeedFL, wheelSpeedFR, wheelSpeedBL, wheelSpeedBR};

  frc::SmartDashboard::PutNumberArray("Swerve/MeasuredStates", measuredStates);
  frc::SmartDashboard::PutNumberArray("Swerve/DesiredStates", desiredStates);
  frc::SmartDashboard::PutNumberArray("Swerve/ModuleAngles_deg", moduleAnglesDeg);
  frc::SmartDashboard::PutNumberArray("Swerve/ModuleSpeeds_mps", moduleSpeedsMps);
}

// Function to minimize rotation within +-PI/2
double Drivetrain::MinimizeRotation(double targetAngle, double currentAngle) {
  double error = targetAngle - currentAngle;
  while (error > M_PI) error -= 2.0 * M_PI;
  while (error < -M_PI) error += 2.0 * M_PI;

  if (std::abs(error) > M_PI / 2.0) {
    targetAngle -= M_PI;
  }
  return targetAngle;
}

double Drivetrain::MinimizeRotation(double targetAngleRad, double currentAngleRad, double& speedPercent) {
  double errorRad = std::remainder(targetAngleRad - currentAngleRad, 2.0 * M_PI);
  if (std::abs(errorRad) > M_PI / 2.0) {
    errorRad -= std::copysign(M_PI, errorRad);
    speedPercent = -speedPercent;
  }
  return std::remainder(currentAngleRad + errorRad, 2.0 * M_PI);
}

double Drivetrain::WrapError(double targetAngle, double currentAngle) {
  double error = targetAngle + M_PI - currentAngle;
  while (error > M_PI) error -= 2.0 * M_PI;
  while (error < -M_PI) error += 2.0 * M_PI;
  return error;
}

double Drivetrain::ChimiOptimizeAzimuth(double waRad, double currentTurns, double& ws) {
  const double targetTurns = waRad / (2.0 * M_PI);
  double errorTurns = std::remainder(targetTurns - currentTurns, 1.0);
  if (std::abs(errorTurns) > 0.25) {
    errorTurns -= std::copysign(0.5, errorTurns);
    ws = -ws;
  }
  if (std::abs(errorTurns) < 0.003) {
    errorTurns = 0.0;
  }
  return currentTurns + errorTurns;
}

ModuleStates Drivetrain::InverseKinematics(double FWD, double STR, double ROT) {
  const double A = STR - ROT * (DC::L / DC::R);
  const double B = STR + ROT * (DC::L / DC::R);
  const double C = FWD - ROT * (DC::W / DC::R);
  const double D = FWD + ROT * (DC::W / DC::R);

  ModuleStates s;
  s.wsFR = std::hypot(B, C);  s.waFR = std::atan2(B, C);
  s.wsFL = std::hypot(B, D);  s.waFL = std::atan2(B, D);
  s.wsBL = std::hypot(A, D);  s.waBL = std::atan2(A, D);
  s.wsBR = std::hypot(A, C);  s.waBR = std::atan2(A, C);

  const double maxSpeed = std::max({s.wsFL, s.wsFR, s.wsBL, s.wsBR});
  if (maxSpeed > 1.0) {
    s.wsFL /= maxSpeed; s.wsFR /= maxSpeed; s.wsBL /= maxSpeed; s.wsBR /= maxSpeed;
  }
  return s;
}

double Drivetrain::calcYawStraight(double targetAngle, double currentAngle, double kP) {
  return std::remainder(targetAngle - currentAngle, 360.0) * kP;
}

void Drivetrain::odometryUpdate(double angleFL, double angleFR, double angleBL, double angleBR,
                                double wheelSpeedFL, double wheelSpeedFR, double wheelSpeedBL,
                                double wheelSpeedBR, double GyroValue) {
  units::time::second_t odoCurrentTime = frc::Timer::GetFPGATimestamp();
  odoDeltaTime = (odoCurrentTime - odoLastTime).value();
  odoLastTime = odoCurrentTime;

  double B_FL = std::sin(angleFL) * wheelSpeedFL;
  double B_FR = std::sin(angleFR) * wheelSpeedFR;
  double A_BL = std::sin(angleBL) * wheelSpeedBL;
  double A_BR = std::sin(angleBR) * wheelSpeedBR;

  double D_FL = std::cos(angleFL) * wheelSpeedFL;
  double C_FR = std::cos(angleFR) * wheelSpeedFR;
  double D_BL = std::cos(angleBL) * wheelSpeedBL;
  double C_BR = std::cos(angleBR) * wheelSpeedBR;

  double A = (A_BL + A_BR) / 2.0;
  double B = (B_FL + B_FR) / 2.0;
  double C = (C_FR + C_BR) / 2.0;
  double D = (D_FL + D_BL) / 2.0;

  odoROT = GyroValue * M_PI / 180.0;

  double FWD1 = odoROT * (L / 2.0) + A;
  double FWD2 = -odoROT * (L / 2.0) + B;
  odoFWD = (FWD1 + FWD2) / 2.0;

  double STR1 = odoROT * (W / 2.0) + C;
  double STR2 = -odoROT * (W / 2.0) + D;
  odoSTR = (STR1 + STR2) / 2.0;

  double temp = odoFWD * std::cos(GyroValue * M_PI / 180.0) + odoSTR * std::sin(GyroValue * M_PI / 180.0);
  odoSTR = odoSTR * std::cos(GyroValue * M_PI / 180.0) - odoFWD * std::sin(GyroValue * M_PI / 180.0);
  odoFWD = temp;

  if (frc::DriverStation::IsEnabled()) {
    positionFWDField -= units::length::meter_t(odoFWD * odoDeltaTime);
    positionSTRField += units::length::meter_t(odoSTR * odoDeltaTime);
    ROTField = frc::Rotation2d(units::angle::radian_t(odoROT));
  } else {
    odoFWD = 0; odoSTR = 0; odoROT = 0; odoDeltaTime = 0;
    odoLastTime = units::time::second_t(0);
    positionFWDField = 0_m; positionSTRField = 0_m;
    ROTField = frc::Rotation2d(0_rad);
  }

  frc::SmartDashboard::PutNumber("positionFWDField", positionFWDField.value());
  frc::SmartDashboard::PutNumber("positionSTRField", positionSTRField.value());
  frc::SmartDashboard::PutNumber("ROTField_deg", ROTField.Degrees().value());
}

void Drivetrain::UpdateSim(units::time::second_t dt) {
  constexpr double kMaxRps = 100.0;
  auto simDrive = [&](hardware::TalonFX& motor, double cmd) {
    auto& sim = motor.GetSimState();
    const double rps = cmd * kMaxRps;
    sim.SetRotorVelocity(units::turns_per_second_t{rps});
    sim.AddRotorPosition(units::turn_t{rps * dt.value()});
  };
  bool swapM2 = frc::SmartDashboard::GetBoolean("Drive/Swap_Module2_IDs", false);
  hardware::TalonFX& frDrive = swapM2 ? m_FR_Steer : m_FR_Drive;
  simDrive(m_FL_Drive, -oldspeedFL / 2.0);
  simDrive(frDrive, oldspeedFR / 2.0);
  simDrive(m_BL_Drive, -oldspeedBL / 2.0);
  simDrive(m_BR_Drive, -oldspeedBR / 2.0);

  const double tFL = (angleFL / M_PI) / 2.0;
  const double tFR = (angleFR / M_PI) / 2.0;
  const double tBL = (angleBL / M_PI) / 2.0;
  const double tBR = (angleBR / M_PI) / 2.0;

  m_FL_Steer.GetSimState().SetRawRotorPosition(units::turn_t{tFL * DC::rotorToSensorRatio});
  m_FR_Steer.GetSimState().SetRawRotorPosition(units::turn_t{tFR * DC::rotorToSensorRatio});
  m_BL_Steer.GetSimState().SetRawRotorPosition(units::turn_t{tBL * DC::rotorToSensorRatio});
  m_BR_Steer.GetSimState().SetRawRotorPosition(units::turn_t{tBR * DC::rotorToSensorRatio});
}

std::pair<double, double> Drivetrain::TankDriveKinematics(double throttle, double turn) {
  double left = throttle + turn;
  double right = throttle - turn;
  double maxMag = std::max(std::abs(left), std::abs(right));
  if (maxMag > 1.0) {
    left /= maxMag;
    right /= maxMag;
  }
  return {left, right};
}
