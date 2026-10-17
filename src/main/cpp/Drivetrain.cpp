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

namespace {
// Straight-ahead AbsolutePosition readings from Phoenix Tuner X screenshots:
//   Swerve 1 (BR, CANcoder 12 on "rio"):       -0.297363 turns
//   Swerve 2 (FR, CANcoder 22 on "rio"):       -0.173584 turns
//   Swerve 3 (FL, CANcoder 32 on "intakebus"): -0.172119 turns
//   Swerve 4 (BL, CANcoder 42 on "rio"):        0.158203 turns
constexpr double kSwerve1BRZeroTurns = -0.297363;
constexpr double kSwerve2FRZeroTurns = -0.173584;
constexpr double kSwerve3FLZeroTurns = -0.172119;
constexpr double kSwerve4BLZeroTurns =  0.158203;
}  // namespace

double Drivetrain::GetCANcoderFLTurns() {
  auto pos = CANcoderFL.GetAbsolutePosition();
  if (pos.GetStatus().IsOK()) {
    return pos.GetValueAsDouble();
  }
  static ctre::phoenix6::hardware::CANcoder CANcoderFL_rio{DC::kFLEncoderId, ctre::phoenix6::CANBus{DC::kCanBus}};
  auto rioPos = CANcoderFL_rio.GetAbsolutePosition();
  if (rioPos.GetStatus().IsOK()) {
    return rioPos.GetValueAsDouble();
  }
  return pos.GetValueAsDouble();
}

bool Drivetrain::IsCANcoderFLConnected() {
  if (CANcoderFL.IsConnected()) {
    return true;
  }
  static ctre::phoenix6::hardware::CANcoder CANcoderFL_rio{DC::kFLEncoderId, ctre::phoenix6::CANBus{DC::kCanBus}};
  return CANcoderFL_rio.IsConnected();
}

Drivetrain::Drivetrain() {
  positionFWDField = units::length::meter_t(0);
  positionSTRField = units::length::meter_t(0);

  ConfigureMotors();
}

void Drivetrain::ConfigureMotors() {
  // Drive Krakens configurations: all 4 modules (BR, FR, FL, BL) active in Brake mode
  driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = units::time::second_t(0.5);
  driveConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

  m_FL_Drive.GetConfigurator().Apply(driveConfig, 100_ms);
  m_FR_Drive.GetConfigurator().Apply(driveConfig, 100_ms);
  m_BL_Drive.GetConfigurator().Apply(driveConfig, 100_ms);
  m_BR_Drive.GetConfigurator().Apply(driveConfig, 100_ms);

  Posconfig.kP = ModuleP;
  Posconfig.kI = ModuleI;
  Posconfig.kD = ModuleD;

  // Swerve 3 (FL, Steer 31 on "rio", CANcoder 32 on "intakebus"):
  // Because Steer 31 ("rio") and CANcoder 32 ("intakebus") are on different CAN buses,
  // setting RemoteCANcoder on Steer 31 causes a missing-remote-sensor fault (solid orange, no movement).
  // Instead, configure Steer 31 with RotorSensor + Brake mode and run the closed-loop PID on the roboRIO
  // directly from GetCANcoderFLTurns() via m_FL_Steer.Set(dutyCycle).
  configFL.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor;
  configFL.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
  configFL.MotorOutput.PeakForwardDutyCycle = 1.0;
  configFL.MotorOutput.PeakReverseDutyCycle = -1.0;

  // Steer Krakens (Swerve 2 FR, Swerve 4 BL, Swerve 1 BR):
  // Brake mode + reduced PeakDutyCycle (0.35) + 0.08s ramp + yesterday's tuned PID (0.75, 0.0001, 1.0)
  configFR.Feedback.FeedbackRemoteSensorID = CANcoderFR.GetDeviceID();
  configFR.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
  configFR.Feedback.SensorToMechanismRatio = 1.0;
  configFR.Feedback.RotorToSensorRatio = 18;
  configFR.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = units::time::second_t(0.08);
  configFR.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
  configFR.MotorOutput.PeakForwardDutyCycle = 0.35;
  configFR.MotorOutput.PeakReverseDutyCycle = -0.35;
  configFR.ClosedLoopGeneral.ContinuousWrap = true;
  configFR.Slot0 = Posconfig;

  configBL.Feedback.FeedbackRemoteSensorID = CANcoderBL.GetDeviceID();
  configBL.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
  configBL.Feedback.SensorToMechanismRatio = 1.0;
  configBL.Feedback.RotorToSensorRatio = 18;
  configBL.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = units::time::second_t(0.08);
  configBL.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
  configBL.MotorOutput.PeakForwardDutyCycle = 0.35;
  configBL.MotorOutput.PeakReverseDutyCycle = -0.35;
  configBL.ClosedLoopGeneral.ContinuousWrap = true;
  configBL.Slot0 = Posconfig;

  configBR.Feedback.FeedbackRemoteSensorID = CANcoderBR.GetDeviceID();
  configBR.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
  configBR.Feedback.SensorToMechanismRatio = 1.0;
  configBR.Feedback.RotorToSensorRatio = 18;
  configBR.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = units::time::second_t(0.08);
  configBR.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
  configBR.MotorOutput.PeakForwardDutyCycle = 0.35;
  configBR.MotorOutput.PeakReverseDutyCycle = -0.35;
  configBR.ClosedLoopGeneral.ContinuousWrap = true;
  configBR.Slot0 = Posconfig;

  auto flStatus = m_FL_Steer.GetConfigurator().Apply(configFL, 100_ms);
  m_FR_Steer.GetConfigurator().Apply(configFR, 100_ms);
  m_BL_Steer.GetConfigurator().Apply(configBL, 100_ms);
  m_BR_Steer.GetConfigurator().Apply(configBR, 100_ms);

  frc::SmartDashboard::PutString("Diagnostics/FL_Steer_ConfigStatus", flStatus.GetName());

  // Publish editable PID & deadband fields to SmartDashboard for live tuning
  frc::SmartDashboard::PutNumber("Swerve/Steer_kP", ModuleP);
  frc::SmartDashboard::PutNumber("Swerve/Steer_kI", ModuleI);
  frc::SmartDashboard::PutNumber("Swerve/Steer_kD", ModuleD);
  frc::SmartDashboard::PutNumber("Swerve/FL_Steer_kP", 1.5);
  frc::SmartDashboard::PutNumber("Swerve/Steer_DeadbandTurns", DC::kSteerDeadbandTurns);
}

void Drivetrain::DisabledInit() {
  m_FL_Drive.Set(0.0);
  m_FR_Drive.Set(0.0);
  m_BL_Drive.Set(0.0);
  m_BR_Drive.Set(0.0);
  m_FL_Steer.Set(0.0);
  m_FR_Steer.Set(0.0);
  m_BL_Steer.Set(0.0);
  m_BR_Steer.Set(0.0);
  oldspeedFL = 0; oldspeedFR = 0; oldspeedBL = 0; oldspeedBR = 0;
}

void Drivetrain::DisabledPeriodic() {
  m_FL_Drive.Set(0.0);
  m_FR_Drive.Set(0.0);
  m_BL_Drive.Set(0.0);
  m_BR_Drive.Set(0.0);
  m_FL_Steer.Set(0.0);
  m_FR_Steer.Set(0.0);
  m_BL_Steer.Set(0.0);
  m_BR_Steer.Set(0.0);

  // Check for live PID edits on SmartDashboard while disabled
  const double newP = frc::SmartDashboard::GetNumber("Swerve/Steer_kP", ModuleP);
  const double newI = frc::SmartDashboard::GetNumber("Swerve/Steer_kI", ModuleI);
  const double newD = frc::SmartDashboard::GetNumber("Swerve/Steer_kD", ModuleD);
  if (std::abs(newP - ModuleP) > 1e-5 ||
      std::abs(newI - ModuleI) > 1e-5 ||
      std::abs(newD - ModuleD) > 1e-5) {
    ModuleP = static_cast<float>(newP);
    ModuleI = static_cast<float>(newI);
    ModuleD = static_cast<float>(newD);
    Posconfig.kP = ModuleP;
    Posconfig.kI = ModuleI;
    Posconfig.kD = ModuleD;
    m_FR_Steer.GetConfigurator().Apply(Posconfig);
    m_BL_Steer.GetConfigurator().Apply(Posconfig);
    m_BR_Steer.GetConfigurator().Apply(Posconfig);
  }

  const double curFLTurns = std::remainder(GetCANcoderFLTurns() - kSwerve3FLZeroTurns, 1.0);
  const double curFRTurns = std::remainder(m_FR_Steer.GetPosition().GetValueAsDouble() - kSwerve2FRZeroTurns, 1.0);
  const double curBLTurns = std::remainder(m_BL_Steer.GetPosition().GetValueAsDouble() - kSwerve4BLZeroTurns, 1.0);
  const double curBRTurns = std::remainder(m_BR_Steer.GetPosition().GetValueAsDouble() - kSwerve1BRZeroTurns, 1.0);

  const double curFLDeg = curFLTurns * 360.0;
  const double curFRDeg = curFRTurns * 360.0;
  const double curBLDeg = curBLTurns * 360.0;
  const double curBRDeg = curBRTurns * 360.0;

  frc::SmartDashboard::PutNumber("Swerve/FL_Angle_Deg", curFLDeg);
  frc::SmartDashboard::PutNumber("Swerve/FR_Angle_Deg", curFRDeg);
  frc::SmartDashboard::PutNumber("Swerve/BL_Angle_Deg", curBLDeg);
  frc::SmartDashboard::PutNumber("Swerve/BR_Angle_Deg", curBRDeg);

  frc::SmartDashboard::PutNumber("Swerve/FL_ZeroedTurns", curFLTurns);
  frc::SmartDashboard::PutNumber("Swerve/FR_ZeroedTurns", curFRTurns);
  frc::SmartDashboard::PutNumber("Swerve/BL_ZeroedTurns", curBLTurns);
  frc::SmartDashboard::PutNumber("Swerve/BR_ZeroedTurns", curBRTurns);

  // Raw CANcoder absolute readings
  frc::SmartDashboard::PutNumber("Swerve/CANcoder_FL_RawTurns", GetCANcoderFLTurns());
  frc::SmartDashboard::PutNumber("Swerve/CANcoder_FR_RawTurns", CANcoderFR.GetAbsolutePosition().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Swerve/CANcoder_BL_RawTurns", CANcoderBL.GetAbsolutePosition().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Swerve/CANcoder_BR_RawTurns", CANcoderBR.GetAbsolutePosition().GetValueAsDouble());

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
}

void Drivetrain::TestZeroAzimuth() {
  ctre::phoenix6::controls::PositionDutyCycle steerPos{0_tr};
  const double curZeroedFL = std::remainder(GetCANcoderFLTurns() - kSwerve3FLZeroTurns, 1.0);
  const double errFL = std::remainder(0.0 - curZeroedFL, 1.0);
  if (std::abs(errFL) > DC::kSteerDeadbandTurns) {
    m_FL_Steer.Set(std::clamp(errFL * 1.5, -0.45, 0.45));
  } else {
    m_FL_Steer.Set(0.0);
  }
  m_FR_Steer.SetControl(steerPos.WithPosition(units::angle::turn_t{kSwerve2FRZeroTurns}));
  m_BL_Steer.SetControl(steerPos.WithPosition(units::angle::turn_t{kSwerve4BLZeroTurns}));
  m_BR_Steer.SetControl(steerPos.WithPosition(units::angle::turn_t{kSwerve1BRZeroTurns}));
  m_FL_Drive.Set(0.0);
  m_FR_Drive.Set(0.0);
  m_BL_Drive.Set(0.0);
  m_BR_Drive.Set(0.0);
}

void Drivetrain::TestRotateModules(double speed) {
  ctre::phoenix6::controls::DutyCycleOut duty{speed};
  m_FL_Steer.SetControl(duty);
  m_FR_Steer.SetControl(duty);
  m_BL_Steer.SetControl(duty);
  m_BR_Steer.SetControl(duty);
  m_FL_Drive.Set(0.0);
  m_FR_Drive.Set(0.0);
  m_BL_Drive.Set(0.0);
  m_BR_Drive.Set(0.0);
}

void Drivetrain::Update(double x, double y, double x2, double GyroValue,
                        double triggerL, double triggerR, bool FieldCentric) {
  // Apply live PID edits from SmartDashboard if changed
  const double newP = frc::SmartDashboard::GetNumber("Swerve/Steer_kP", ModuleP);
  const double newI = frc::SmartDashboard::GetNumber("Swerve/Steer_kI", ModuleI);
  const double newD = frc::SmartDashboard::GetNumber("Swerve/Steer_kD", ModuleD);
  const double deadbandTurns = frc::SmartDashboard::GetNumber("Swerve/Steer_DeadbandTurns", DC::kSteerDeadbandTurns);
  if (std::abs(newP - ModuleP) > 1e-5 ||
      std::abs(newI - ModuleI) > 1e-5 ||
      std::abs(newD - ModuleD) > 1e-5) {
    ModuleP = static_cast<float>(newP);
    ModuleI = static_cast<float>(newI);
    ModuleD = static_cast<float>(newD);
    Posconfig.kP = ModuleP;
    Posconfig.kI = ModuleI;
    Posconfig.kD = ModuleD;
    m_FR_Steer.GetConfigurator().Apply(Posconfig);
    m_BL_Steer.GetConfigurator().Apply(Posconfig);
    m_BR_Steer.GetConfigurator().Apply(Posconfig);
  }

  // Live Swerve 3 (CAN 31) diagnostics while driving
  frc::SmartDashboard::PutNumber("Diagnostics/FL_Steer_Temp_C", m_FL_Steer.GetDeviceTemp().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Diagnostics/FL_Steer_Current_A", m_FL_Steer.GetStatorCurrent().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Diagnostics/FL_Steer_DutyCycle", m_FL_Steer.GetDutyCycle().GetValueAsDouble());

  // Rotation disabled: only forwards/backwards (y) and side-to-side (x)
  (void)x2;
  ROT = 0.0;

  // When stick is centered, stop all drive and steer motors so they never hum or twitch
  if (x == 0.0 && y == 0.0) {
    oldspeedFL = 0; oldspeedFR = 0; oldspeedBL = 0; oldspeedBR = 0;
    m_FL_Drive.Set(0.0);
    m_FR_Drive.Set(0.0);
    m_BL_Drive.Set(0.0);
    m_BR_Drive.Set(0.0);
    m_FL_Steer.Set(0.0);
    m_FR_Steer.Set(0.0);
    m_BL_Steer.Set(0.0);
    m_BR_Steer.Set(0.0);
    return;
  }

  // Get zeroed wheel angles in radians for all 4 swerve modules (FL from CANcoder 32, FR/BL/BR from RemoteCANcoder)
  const double curZeroedFL = std::remainder(GetCANcoderFLTurns() - kSwerve3FLZeroTurns, 1.0);
  const double curZeroedFR = std::remainder(m_FR_Steer.GetPosition().GetValueAsDouble() - kSwerve2FRZeroTurns, 1.0);
  const double curZeroedBL = std::remainder(m_BL_Steer.GetPosition().GetValueAsDouble() - kSwerve4BLZeroTurns, 1.0);
  const double curZeroedBR = std::remainder(m_BR_Steer.GetPosition().GetValueAsDouble() - kSwerve1BRZeroTurns, 1.0);

  double FL_pos = curZeroedFL * (2.0 * M_PI);
  double FR_pos = curZeroedFR * (2.0 * M_PI);
  double BL_pos = curZeroedBL * (2.0 * M_PI);
  double BR_pos = curZeroedBR * (2.0 * M_PI);

  double STR = x;
  double FWD = y;

  // Field Centric // offsetting vectors by the gyro angle
  if (FieldCentric == true) {
    double temp = FWD * std::cos(GyroValue * M_PI / 180.0) + STR * std::sin(GyroValue * M_PI / 180.0);
    STR = -FWD * std::sin(GyroValue * M_PI / 180.0) + STR * std::cos(GyroValue * M_PI / 180.0);
    FWD = temp;
  }

  // Pure translation (exactly as in 3d1bb52 / 772febb): all 4 wheels turn to the EXACT SAME angle
  const double targetAngleRad = std::atan2(STR, FWD);
  const double targetTurnsOffset = targetAngleRad / (2.0 * M_PI);
  const double targetSpeed = std::clamp(std::hypot(STR, FWD), 0.0, 1.0);

  angleFL = targetAngleRad;
  angleFR = targetAngleRad;
  angleBL = targetAngleRad;
  angleBR = targetAngleRad;

  speedFL = targetSpeed;
  speedFR = targetSpeed;
  speedBL = targetSpeed;
  speedBR = targetSpeed;

  ctre::phoenix6::controls::PositionDutyCycle steerm_speedFR{0_tr};
  ctre::phoenix6::controls::PositionDutyCycle steerm_speedBL{0_tr};
  ctre::phoenix6::controls::PositionDutyCycle steerm_speedBR{0_tr};

  steerm_speedFR.Slot = 0;
  steerm_speedBL.Slot = 0;
  steerm_speedBR.Slot = 0;

  const double errFL = std::remainder(targetTurnsOffset - curZeroedFL, 1.0);
  const double errFR = std::remainder(targetTurnsOffset - curZeroedFR, 1.0);
  const double errBL = std::remainder(targetTurnsOffset - curZeroedBL, 1.0);
  const double errBR = std::remainder(targetTurnsOffset - curZeroedBR, 1.0);

  const double curFLDeg = curZeroedFL * 360.0;
  const double curFRDeg = curZeroedFR * 360.0;
  const double curBLDeg = curZeroedBL * 360.0;
  const double curBRDeg = curZeroedBR * 360.0;

  const double targetDeg = targetTurnsOffset * 360.0;

  const double errFLDeg = errFL * 360.0;
  const double errFRDeg = errFR * 360.0;
  const double errBLDeg = errBL * 360.0;
  const double errBRDeg = errBR * 360.0;

  // Live wheel angles in degrees (0 deg = straight ahead)
  frc::SmartDashboard::PutNumber("Swerve/FL_Angle_Deg", curFLDeg);
  frc::SmartDashboard::PutNumber("Swerve/FR_Angle_Deg", curFRDeg);
  frc::SmartDashboard::PutNumber("Swerve/BL_Angle_Deg", curBLDeg);
  frc::SmartDashboard::PutNumber("Swerve/BR_Angle_Deg", curBRDeg);

  // Target angles in degrees
  frc::SmartDashboard::PutNumber("Swerve/FL_Target_Deg", targetDeg);
  frc::SmartDashboard::PutNumber("Swerve/FR_Target_Deg", targetDeg);
  frc::SmartDashboard::PutNumber("Swerve/BL_Target_Deg", targetDeg);
  frc::SmartDashboard::PutNumber("Swerve/BR_Target_Deg", targetDeg);

  // Alignment errors in degrees
  frc::SmartDashboard::PutNumber("Swerve/FL_Error_Deg", errFLDeg);
  frc::SmartDashboard::PutNumber("Swerve/FR_Error_Deg", errFRDeg);
  frc::SmartDashboard::PutNumber("Swerve/BL_Error_Deg", errBLDeg);
  frc::SmartDashboard::PutNumber("Swerve/BR_Error_Deg", errBRDeg);

  // Wheel straight alignment indicator (|angle| < 3 degrees)
  bool wheelsStraight = (std::abs(curFLDeg) < 3.0) &&
                        (std::abs(curFRDeg) < 3.0) &&
                        (std::abs(curBLDeg) < 3.0) &&
                        (std::abs(curBRDeg) < 3.0);
  frc::SmartDashboard::PutBoolean("Swerve/All_Wheels_Straight", wheelsStraight);

  // Raw CANcoder absolute readings (turns)
  frc::SmartDashboard::PutNumber("Swerve/CANcoder_FL_RawTurns", GetCANcoderFLTurns());
  frc::SmartDashboard::PutNumber("Swerve/CANcoder_FR_RawTurns", CANcoderFR.GetAbsolutePosition().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Swerve/CANcoder_BL_RawTurns", CANcoderBL.GetAbsolutePosition().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Swerve/CANcoder_BR_RawTurns", CANcoderBR.GetAbsolutePosition().GetValueAsDouble());

  // Turns telemetry
  frc::SmartDashboard::PutNumber("Swerve/TargetTurns", targetTurnsOffset);
  frc::SmartDashboard::PutNumber("Swerve/FL_ErrorTurns", errFL);
  frc::SmartDashboard::PutNumber("Swerve/FR_ErrorTurns", errFR);
  frc::SmartDashboard::PutNumber("Swerve/BL_ErrorTurns", errBL);
  frc::SmartDashboard::PutNumber("Swerve/BR_ErrorTurns", errBR);
  frc::SmartDashboard::PutNumber("Swerve/FL_ZeroedTurns", curZeroedFL);
  frc::SmartDashboard::PutNumber("Swerve/FR_ZeroedTurns", curZeroedFR);
  frc::SmartDashboard::PutNumber("Swerve/BL_ZeroedTurns", curZeroedBL);
  frc::SmartDashboard::PutNumber("Swerve/BR_ZeroedTurns", curZeroedBR);

  if (std::abs(errFL) > deadbandTurns) {
    const double flKp = frc::SmartDashboard::GetNumber("Swerve/FL_Steer_kP", 1.5);
    const double flDuty = std::clamp(errFL * flKp, -0.45, 0.45);
    m_FL_Steer.Set(flDuty);
  } else {
    m_FL_Steer.Set(0.0);
  }

  if (std::abs(errFR) > deadbandTurns) {
    m_FR_Steer.SetControl(steerm_speedFR.WithPosition(units::angle::turn_t{kSwerve2FRZeroTurns + targetTurnsOffset}));
  } else {
    m_FR_Steer.Set(0.0);
  }

  if (std::abs(errBL) > deadbandTurns) {
    m_BL_Steer.SetControl(steerm_speedBL.WithPosition(units::angle::turn_t{kSwerve4BLZeroTurns + targetTurnsOffset}));
  } else {
    m_BL_Steer.Set(0.0);
  }

  if (std::abs(errBR) > deadbandTurns) {
    m_BR_Steer.SetControl(steerm_speedBR.WithPosition(units::angle::turn_t{kSwerve1BRZeroTurns + targetTurnsOffset}));
  } else {
    m_BR_Steer.Set(0.0);
  }

  double speedFLEr = speedFL - oldspeedFL;
  oldspeedFL += (speedFLEr >= 0.07) ? 0.07 : (speedFLEr <= -0.07) ? -0.07 : speedFLEr;
  double speedFREr = speedFR - oldspeedFR;
  oldspeedFR += (speedFREr >= 0.07) ? 0.07 : (speedFREr <= -0.07) ? -0.07 : speedFREr;
  double speedBLEr = speedBL - oldspeedBL;
  oldspeedBL += (speedBLEr >= 0.07) ? 0.07 : (speedBLEr <= -0.07) ? -0.07 : speedBLEr;
  double speedBREr = speedBR - oldspeedBR;
  oldspeedBR += (speedBREr >= 0.07) ? 0.07 : (speedBREr <= -0.07) ? -0.07 : speedBREr;

  // Control Driving motors: FL and BL are negated (-oldspeed), FR and BR are positive
  const double cap = std::clamp(0.5 + 0.4 * triggerR - 0.35 * triggerL, 0.1, 1.0);
  m_FL_Drive.Set(std::clamp(-oldspeedFL / 2.0, -cap, cap));
  m_FR_Drive.Set(std::clamp(oldspeedFR / 2.0, -cap, cap));
  m_BL_Drive.Set(std::clamp(-oldspeedBL / 2.0, -cap, cap));
  m_BR_Drive.Set(std::clamp(oldspeedBR / 2.0, -cap, cap));

  double wheelSpeedFL = (m_FL_Drive.GetVelocity().GetValueAsDouble() / 6.0375) * (M_PI * 0.0889);
  double wheelSpeedFR = (m_FR_Drive.GetVelocity().GetValueAsDouble() / 6.0375) * (M_PI * 0.0889);
  double wheelSpeedBL = (m_BL_Drive.GetVelocity().GetValueAsDouble() / 6.0375) * (M_PI * 0.0889);
  double wheelSpeedBR = (m_BR_Drive.GetVelocity().GetValueAsDouble() / 6.0375) * (M_PI * 0.0889);

  odometryUpdate(FL_pos, FR_pos, BL_pos, BR_pos, wheelSpeedFL, wheelSpeedFR, wheelSpeedBL, wheelSpeedBR, GyroValue);
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
  simDrive(m_FL_Drive, -oldspeedFL / 2.0);
  simDrive(m_FR_Drive, oldspeedFR / 2.0);
  simDrive(m_BL_Drive, -oldspeedBL / 2.0);
  simDrive(m_BR_Drive, oldspeedBR / 2.0);

  const double tFL = (angleFL / M_PI) / 2.0;
  const double tFR = (angleFR / M_PI) / 2.0;
  const double tBL = (angleBL / M_PI) / 2.0;
  const double tBR = (angleBR / M_PI) / 2.0;

  m_FL_Steer.GetSimState().SetRawRotorPosition(units::turn_t{tFL * DC::rotorToSensorRatio});
  m_FR_Steer.GetSimState().SetRawRotorPosition(units::turn_t{tFR * DC::rotorToSensorRatio});
  m_BL_Steer.GetSimState().SetRawRotorPosition(units::turn_t{tBL * DC::rotorToSensorRatio});
  m_BR_Steer.GetSimState().SetRawRotorPosition(units::turn_t{tBR * DC::rotorToSensorRatio});

  CANcoderFL.GetSimState().SetRawPosition(units::turn_t{tFL});
  CANcoderFR.GetSimState().SetRawPosition(units::turn_t{tFR});
  CANcoderBL.GetSimState().SetRawPosition(units::turn_t{tBL});
  CANcoderBR.GetSimState().SetRawPosition(units::turn_t{tBR});
}
