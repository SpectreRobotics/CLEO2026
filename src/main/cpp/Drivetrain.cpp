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
  // ChimiSwerve Section 3.6 (Pages 40-41): Ramp rate + current limiting prevent brownouts
  driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = units::time::second_t(0.5);
  driveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = units::time::second_t(DC::DriveOpenLoopRamp);
  driveConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
  driveConfig.CurrentLimits.StatorCurrentLimit = units::current::ampere_t{DC::kDriveCurrentLimit};
  driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;

  m_FL_Drive.GetConfigurator().Apply(driveConfig, 100_ms);
  m_FR_Drive.GetConfigurator().Apply(driveConfig, 100_ms);
  m_BL_Drive.GetConfigurator().Apply(driveConfig, 100_ms);
  m_BR_Drive.GetConfigurator().Apply(driveConfig, 100_ms);

  Posconfig.kP = ModuleP;
  Posconfig.kI = ModuleI;
  Posconfig.kD = ModuleD;
  Posconfig.kS = ModuleS;

  // Swerve 3 (FL, Steer 31 on "rio", CANcoder 32 moved to "rio"):
  // Identical 1-to-1 hardware RemoteCANcoder config as FR, BL, BR.
  // ContinuousWrap = false because C++ computes continuous shortest-path target (kZeroTurns + m_contTarget),
  // which prevents the TalonFX D term from ever seeing a +-0.5 wrap jump and spinning 360 deg!
  configFL.Feedback.FeedbackRemoteSensorID = CANcoderFL.GetDeviceID();
  configFL.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
  configFL.Feedback.SensorToMechanismRatio = 1.0;
  configFL.Feedback.RotorToSensorRatio = 18;
  configFL.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = units::time::second_t(ModuleRampSeconds);
  configFL.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
  configFL.MotorOutput.PeakForwardDutyCycle = ModulePeakOutput;
  configFL.MotorOutput.PeakReverseDutyCycle = -ModulePeakOutput;
  configFL.CurrentLimits.StatorCurrentLimit = units::current::ampere_t{DC::kSteerCurrentLimit};
  configFL.CurrentLimits.StatorCurrentLimitEnable = true;
  configFL.ClosedLoopGeneral.ContinuousWrap = false;
  configFL.Slot0 = Posconfig;

  // --- Commented out: Old configFL when CANcoder 32 was on "intakebus" ---
  // configFL.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor;
  // configFL.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = units::time::second_t(0.0);
  // configFL.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = units::time::second_t(0.0);
  // configFL.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
  // configFL.MotorOutput.PeakForwardDutyCycle = 0.45;
  // configFL.MotorOutput.PeakReverseDutyCycle = -0.45;
  // configFL.ClosedLoopGeneral.ContinuousWrap = false;
  // configFL.Slot0 = Posconfig;

  configFR.Feedback.FeedbackRemoteSensorID = CANcoderFR.GetDeviceID();
  configFR.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
  configFR.Feedback.SensorToMechanismRatio = 1.0;
  configFR.Feedback.RotorToSensorRatio = 18;
  configFR.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = units::time::second_t(ModuleRampSeconds);
  configFR.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
  configFR.MotorOutput.PeakForwardDutyCycle = ModulePeakOutput;
  configFR.MotorOutput.PeakReverseDutyCycle = -ModulePeakOutput;
  configFR.CurrentLimits.StatorCurrentLimit = units::current::ampere_t{DC::kSteerCurrentLimit};
  configFR.CurrentLimits.StatorCurrentLimitEnable = true;
  configFR.ClosedLoopGeneral.ContinuousWrap = false;
  configFR.Slot0 = Posconfig;

  configBL.Feedback.FeedbackRemoteSensorID = CANcoderBL.GetDeviceID();
  configBL.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
  configBL.Feedback.SensorToMechanismRatio = 1.0;
  configBL.Feedback.RotorToSensorRatio = 18;
  configBL.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = units::time::second_t(ModuleRampSeconds);
  configBL.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
  configBL.MotorOutput.PeakForwardDutyCycle = ModulePeakOutput;
  configBL.MotorOutput.PeakReverseDutyCycle = -ModulePeakOutput;
  configBL.CurrentLimits.StatorCurrentLimit = units::current::ampere_t{DC::kSteerCurrentLimit};
  configBL.CurrentLimits.StatorCurrentLimitEnable = true;
  configBL.ClosedLoopGeneral.ContinuousWrap = false;
  configBL.Slot0 = Posconfig;

  configBR.Feedback.FeedbackRemoteSensorID = CANcoderBR.GetDeviceID();
  configBR.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
  configBR.Feedback.SensorToMechanismRatio = 1.0;
  configBR.Feedback.RotorToSensorRatio = 18;
  configBR.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = units::time::second_t(ModuleRampSeconds);
  configBR.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
  configBR.MotorOutput.PeakForwardDutyCycle = ModulePeakOutput;
  configBR.MotorOutput.PeakReverseDutyCycle = -ModulePeakOutput;
  configBR.CurrentLimits.StatorCurrentLimit = units::current::ampere_t{DC::kSteerCurrentLimit};
  configBR.CurrentLimits.StatorCurrentLimitEnable = true;
  configBR.ClosedLoopGeneral.ContinuousWrap = false;
  configBR.Slot0 = Posconfig;

  auto flStatus = m_FL_Steer.GetConfigurator().Apply(configFL);
  m_FL_Steer.GetConfigurator().Apply(Posconfig);
  m_FR_Steer.GetConfigurator().Apply(configFR);
  m_FR_Steer.GetConfigurator().Apply(Posconfig);
  m_BL_Steer.GetConfigurator().Apply(configBL);
  m_BL_Steer.GetConfigurator().Apply(Posconfig);
  m_BR_Steer.GetConfigurator().Apply(configBR);
  m_BR_Steer.GetConfigurator().Apply(Posconfig);

  frc::SmartDashboard::PutString("Diagnostics/FL_Steer_ConfigStatus", flStatus.GetName());

  // Publish editable PID, kS, ramp, peak output & deadband fields on SmartDashboard on boot
  static bool s_dashInitialized = false;
  if (!s_dashInitialized) {
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
  SyncSmartDashboardPID();
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
    m_FL_Steer.GetConfigurator().Apply(Posconfig);
    m_FR_Steer.GetConfigurator().Apply(Posconfig);
    m_BL_Steer.GetConfigurator().Apply(Posconfig);
    m_BR_Steer.GetConfigurator().Apply(Posconfig);
  }

  if (std::abs(dashRamp - ModuleRampSeconds) > 1e-4f ||
      std::abs(dashPeak - ModulePeakOutput) > 1e-4f) {
    ModuleRampSeconds = dashRamp;
    ModulePeakOutput = dashPeak;
    configFL.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = units::time::second_t(ModuleRampSeconds);
    configFR.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = units::time::second_t(ModuleRampSeconds);
    configBL.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = units::time::second_t(ModuleRampSeconds);
    configBR.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = units::time::second_t(ModuleRampSeconds);
    configFL.MotorOutput.PeakForwardDutyCycle = ModulePeakOutput;
    configFL.MotorOutput.PeakReverseDutyCycle = -ModulePeakOutput;
    configFR.MotorOutput.PeakForwardDutyCycle = ModulePeakOutput;
    configFR.MotorOutput.PeakReverseDutyCycle = -ModulePeakOutput;
    configBL.MotorOutput.PeakForwardDutyCycle = ModulePeakOutput;
    configBL.MotorOutput.PeakReverseDutyCycle = -ModulePeakOutput;
    configBR.MotorOutput.PeakForwardDutyCycle = ModulePeakOutput;
    configBR.MotorOutput.PeakReverseDutyCycle = -ModulePeakOutput;
    m_FL_Steer.GetConfigurator().Apply(configFL.ClosedLoopRamps);
    m_FR_Steer.GetConfigurator().Apply(configFR.ClosedLoopRamps);
    m_BL_Steer.GetConfigurator().Apply(configBL.ClosedLoopRamps);
    m_BR_Steer.GetConfigurator().Apply(configBR.ClosedLoopRamps);
    m_FL_Steer.GetConfigurator().Apply(configFL.MotorOutput);
    m_FR_Steer.GetConfigurator().Apply(configFR.MotorOutput);
    m_BL_Steer.GetConfigurator().Apply(configBL.MotorOutput);
    m_BR_Steer.GetConfigurator().Apply(configBR.MotorOutput);
  }
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
  m_flipFL = false; m_flipFR = false; m_flipBL = false; m_flipBR = false;
  m_wasDriving = false;
  m_steerActiveFL = false; m_steerActiveFR = false; m_steerActiveBL = false; m_steerActiveBR = false;
  iError = 0.0;

  SyncSmartDashboardPID();
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
  m_wasDriving = false;

  SyncSmartDashboardPID();

  // const double curFLTurns = std::remainder(GetCANcoderFLTurns() - kSwerve3FLZeroTurns, 1.0);
  const double curFLTurns = std::remainder(m_FL_Steer.GetPosition().GetValueAsDouble() - kSwerve3FLZeroTurns, 1.0);
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
  frc::SmartDashboard::PutNumber("Swerve/CANcoder_FL_RawTurns", CANcoderFL.GetAbsolutePosition().GetValueAsDouble());
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
  SyncSmartDashboardPID();
  ctre::phoenix6::controls::PositionDutyCycle steerPos{0_tr};
  const double curRawFL = m_FL_Steer.GetPosition().GetValueAsDouble();
  const double curRawFR = m_FR_Steer.GetPosition().GetValueAsDouble();
  const double curRawBL = m_BL_Steer.GetPosition().GetValueAsDouble();
  const double curRawBR = m_BR_Steer.GetPosition().GetValueAsDouble();

  m_FL_Steer.SetControl(steerPos.WithPosition(units::angle::turn_t{curRawFL + std::remainder(kSwerve3FLZeroTurns - curRawFL, 1.0)}));
  m_FR_Steer.SetControl(steerPos.WithPosition(units::angle::turn_t{curRawFR + std::remainder(kSwerve2FRZeroTurns - curRawFR, 1.0)}));
  m_BL_Steer.SetControl(steerPos.WithPosition(units::angle::turn_t{curRawBL + std::remainder(kSwerve4BLZeroTurns - curRawBL, 1.0)}));
  m_BR_Steer.SetControl(steerPos.WithPosition(units::angle::turn_t{curRawBR + std::remainder(kSwerve1BRZeroTurns - curRawBR, 1.0)}));

  // --- Commented out: Old TestZeroAzimuth when CANcoder 32 was on "intakebus" ---
  // const double curZeroedFL = std::remainder(GetCANcoderFLTurns() - kSwerve3FLZeroTurns, 1.0);
  // const double errFL = std::remainder(0.0 - curZeroedFL, 1.0);
  // if (std::abs(errFL) > DC::kSteerDeadbandTurns) {
  //   const double flWheelVel = m_FL_Steer.GetVelocity().GetValueAsDouble() / 18.0;
  //   const double pTerm = errFL * (static_cast<double>(FL_P) * 2.0);
  //   const double dTerm = -flWheelVel * (0.10 * static_cast<double>(FL_D));
  //   double flDuty = pTerm + dTerm;
  //   if ((errFL > 0.0 && flDuty < 0.05) || (errFL < 0.0 && flDuty > -0.05)) {
  //     flDuty = std::copysign(0.05, errFL);
  //   }
  //   const double rampDownCap = std::clamp(std::abs(errFL) * 3.5, 0.08, 0.45);
  //   m_FL_Steer.Set(std::clamp(flDuty, -rampDownCap, rampDownCap));
  // } else {
  //   m_FL_Steer.Set(0.0);
  // }

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
  SyncSmartDashboardPID();
  const double deadbandTurns = frc::SmartDashboard::GetNumber("Swerve/Steer_DeadbandTurns", DC::kSteerDeadbandTurns);

  // Live Swerve 3 (CAN 31) diagnostics while driving
  frc::SmartDashboard::PutNumber("Diagnostics/FL_Steer_Temp_C", m_FL_Steer.GetDeviceTemp().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Diagnostics/FL_Steer_Current_A", m_FL_Steer.GetStatorCurrent().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Diagnostics/FL_Steer_DutyCycle", m_FL_Steer.GetDutyCycle().GetValueAsDouble());

  // When both sticks are centered, stop all drive and steer motors so they never hum or twitch
  // ChimiSwerve Section 2.3d (Page 36): "we don't want to PID our angle if we aren't trying to move"
  if (x == 0.0 && y == 0.0 && x2 == 0.0) {
    targetAngle = GyroValue;
    oldspeedFL = 0; oldspeedFR = 0; oldspeedBL = 0; oldspeedBR = 0;
    m_wasDriving = false;
    m_steerActiveFL = false; m_steerActiveFR = false; m_steerActiveBL = false; m_steerActiveBR = false;
    iError = 0.0;
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

  // Get continuous raw and zeroed wheel positions in turns for all 4 swerve modules
  const double curRawFL = m_FL_Steer.GetPosition().GetValueAsDouble();
  const double curRawFR = m_FR_Steer.GetPosition().GetValueAsDouble();
  const double curRawBL = m_BL_Steer.GetPosition().GetValueAsDouble();
  const double curRawBR = m_BR_Steer.GetPosition().GetValueAsDouble();

  const double curContFL = curRawFL - kSwerve3FLZeroTurns;
  const double curContFR = curRawFR - kSwerve2FRZeroTurns;
  const double curContBL = curRawBL - kSwerve4BLZeroTurns;
  const double curContBR = curRawBR - kSwerve1BRZeroTurns;

  // const double curZeroedFL = std::remainder(GetCANcoderFLTurns() - kSwerve3FLZeroTurns, 1.0);
  const double curZeroedFL = std::remainder(curContFL, 1.0);
  const double curZeroedFR = std::remainder(curContFR, 1.0);
  const double curZeroedBL = std::remainder(curContBL, 1.0);
  const double curZeroedBR = std::remainder(curContBR, 1.0);

  double FL_pos = curZeroedFL * (2.0 * M_PI);
  double FR_pos = curZeroedFR * (2.0 * M_PI);
  double BL_pos = curZeroedBL * (2.0 * M_PI);
  double BR_pos = curZeroedBR * (2.0 * M_PI);

  double STR = x;
  double FWD = y;

  // ChimiSwerve Section 2.1b (Pages 11-13): Field-Centric Transformation
  //   FWD_new = FWD * cos(theta) + STR * sin(theta)
  //   STR_new = STR * cos(theta) - FWD * sin(theta)
  if (FieldCentric == true) {
    const double theta = GyroValue * M_PI / 180.0;
    const double fwdNew = FWD * std::cos(theta) + STR * std::sin(theta);
    const double strNew = STR * std::cos(theta) - FWD * std::sin(theta);
    FWD = fwdNew;
    STR = strNew;
  }

  // ChimiSwerve Section 2.3d (Pages 35-36): Straight-Drive Heading Lock while in Field-Centric
  const bool enableHeadingLock = frc::SmartDashboard::GetBoolean("Swerve/EnableHeadingLock", DC::kEnableHeadingLock);
  if (std::abs(x2) > 1e-3 || !FieldCentric) {
    targetAngle = GyroValue;
    ROT = x2;
  } else if (enableHeadingLock && (std::abs(FWD) > 1e-3 || std::abs(STR) > 1e-3)) {
    ROT = std::clamp(calcYawStraight(targetAngle, GyroValue, straightP),
                     -DC::rotationClamp, DC::rotationClamp);
  } else {
    targetAngle = GyroValue;
    ROT = 0.0;
  }

  // Swerve Inverse Kinematics (ChimiSwerve Section 2.1b, Pages 13-15):
  const ModuleStates states = InverseKinematics(FWD, STR, ROT);

  angleFL = states.waFL;
  angleFR = states.waFR;
  angleBL = states.waBL;
  angleBR = states.waBR;

  speedFL = states.wsFL;
  speedFR = states.wsFR;
  speedBL = states.wsBL;
  speedBR = states.wsBR;

  const double rawTargetTurnsFL = angleFL / (2.0 * M_PI);
  const double rawTargetTurnsFR = angleFR / (2.0 * M_PI);
  const double rawTargetTurnsBL = angleBL / (2.0 * M_PI);
  const double rawTargetTurnsBR = angleBR / (2.0 * M_PI);

  // ChimiSwerve Section 2.1b (Page 16): Inversion Awareness with 72<->108 deg (0.30 turn) Hysteresis
  // & 4-Module Synchronization:
  //   - When a wheel would need to turn > 108 deg (0.30 turns), flip its azimuth by 180 deg (0.5 turns)
  //     and negate its drive motor speed so wheels never turn more than ~90 deg.
  //   - Using a 0.30-turn (108 deg) threshold instead of 0.25 (90 deg) gives 36 deg of hysteresis so
  //     pure 90 deg strafes never sit on the flip boundary or chatter between +90 and -90 deg.
  //   - When translating (|ROT| < 0.15), all 4 modules share the same flip state so they always stay synchronized.
  const bool useInversion = frc::SmartDashboard::GetBoolean("Swerve/EnableInversionAwareness", true);
  if (!useInversion) {
    m_flipFL = false; m_flipFR = false; m_flipBL = false; m_flipBR = false;
  } else if (std::abs(ROT) < 0.15) {
    const bool sharedFlip = m_flipFL;
    const double sErrFL = std::remainder((rawTargetTurnsFL + (sharedFlip ? 0.5 : 0.0)) - curContFL, 1.0);
    const double sErrFR = std::remainder((rawTargetTurnsFR + (sharedFlip ? 0.5 : 0.0)) - curContFR, 1.0);
    const double sErrBL = std::remainder((rawTargetTurnsBL + (sharedFlip ? 0.5 : 0.0)) - curContBL, 1.0);
    const double sErrBR = std::remainder((rawTargetTurnsBR + (sharedFlip ? 0.5 : 0.0)) - curContBR, 1.0);
    const double avgAbsErr = (std::abs(sErrFL) + std::abs(sErrFR) + std::abs(sErrBL) + std::abs(sErrBR)) * 0.25;
    const bool newSharedFlip = (avgAbsErr > 0.30) ? !sharedFlip : sharedFlip;
    m_flipFL = newSharedFlip;
    m_flipFR = newSharedFlip;
    m_flipBL = newSharedFlip;
    m_flipBR = newSharedFlip;
  } else {
    const double errModeFL = std::remainder((rawTargetTurnsFL + (m_flipFL ? 0.5 : 0.0)) - curContFL, 1.0);
    const double errModeFR = std::remainder((rawTargetTurnsFR + (m_flipFR ? 0.5 : 0.0)) - curContFR, 1.0);
    const double errModeBL = std::remainder((rawTargetTurnsBL + (m_flipBL ? 0.5 : 0.0)) - curContBL, 1.0);
    const double errModeBR = std::remainder((rawTargetTurnsBR + (m_flipBR ? 0.5 : 0.0)) - curContBR, 1.0);
    if (std::abs(errModeFL) > 0.30) m_flipFL = !m_flipFL;
    if (std::abs(errModeFR) > 0.30) m_flipFR = !m_flipFR;
    if (std::abs(errModeBL) > 0.30) m_flipBL = !m_flipBL;
    if (std::abs(errModeBR) > 0.30) m_flipBR = !m_flipBR;
  }

  const double targetTurnsFL = std::remainder(rawTargetTurnsFL + (m_flipFL ? 0.5 : 0.0), 1.0);
  const double targetTurnsFR = std::remainder(rawTargetTurnsFR + (m_flipFR ? 0.5 : 0.0), 1.0);
  const double targetTurnsBL = std::remainder(rawTargetTurnsBL + (m_flipBL ? 0.5 : 0.0), 1.0);
  const double targetTurnsBR = std::remainder(rawTargetTurnsBR + (m_flipBR ? 0.5 : 0.0), 1.0);

  if (m_flipFL) speedFL = -speedFL;
  if (m_flipFR) speedFR = -speedFR;
  if (m_flipBL) speedBL = -speedBL;
  if (m_flipBR) speedBR = -speedBR;

  ctre::phoenix6::controls::PositionDutyCycle steerm_speedFL{0_tr};
  ctre::phoenix6::controls::PositionDutyCycle steerm_speedFR{0_tr};
  ctre::phoenix6::controls::PositionDutyCycle steerm_speedBL{0_tr};
  ctre::phoenix6::controls::PositionDutyCycle steerm_speedBR{0_tr};

  steerm_speedFL.Slot = 0;
  steerm_speedFR.Slot = 0;
  steerm_speedBL.Slot = 0;
  steerm_speedBR.Slot = 0;

  // Continuous unwrapped target tracking with synchronized wrap direction and 0.65-turn hysteresis:
  // Prevents modules from starting right and then deciding to go left when the stick crosses +-180 deg,
  // and keeps all 4 modules rotating in the same direction on near-180 deg reversals (both from rest and in motion).
  if (!m_wasDriving) {
    m_contTargetFL = curContFL;
    m_contTargetFR = curContFR;
    m_contTargetBL = curContBL;
    m_contTargetBR = curContBR;
    m_wasDriving = true;
  }

  double deltaFL = std::remainder(targetTurnsFL - m_contTargetFL, 1.0);
  double deltaFR = std::remainder(targetTurnsFR - m_contTargetFR, 1.0);
  double deltaBL = std::remainder(targetTurnsBL - m_contTargetBL, 1.0);
  double deltaBR = std::remainder(targetTurnsBR - m_contTargetBR, 1.0);

  const double avgDelta = (deltaFL + deltaFR + deltaBL + deltaBR) * 0.25;
  const double syncSign = (avgDelta >= 0.0) ? 1.0 : -1.0;
  auto syncNearHalfTurn = [syncSign](double& d) {
    if (std::abs(d) > 0.38 && (d * syncSign) < 0.0) {
      d += syncSign;
    }
  };
  syncNearHalfTurn(deltaFL);
  syncNearHalfTurn(deltaFR);
  syncNearHalfTurn(deltaBL);
  syncNearHalfTurn(deltaBR);

  m_contTargetFL += deltaFL;
  m_contTargetFR += deltaFR;
  m_contTargetBL += deltaBL;
  m_contTargetBR += deltaBR;

  auto clampContTarget = [](double& contTarget, double curCont) {
    if (contTarget - curCont > 0.65) {
      contTarget -= 1.0;
    } else if (contTarget - curCont < -0.65) {
      contTarget += 1.0;
    }
  };
  clampContTarget(m_contTargetFL, curContFL);
  clampContTarget(m_contTargetFR, curContFR);
  clampContTarget(m_contTargetBL, curContBL);
  clampContTarget(m_contTargetBR, curContBR);

  const double errFL = m_contTargetFL - curContFL;
  const double errFR = m_contTargetFR - curContFR;
  const double errBL = m_contTargetBL - curContBL;
  const double errBR = m_contTargetBR - curContBR;

  const double curFLDeg = curZeroedFL * 360.0;
  const double curFRDeg = curZeroedFR * 360.0;
  const double curBLDeg = curZeroedBL * 360.0;
  const double curBRDeg = curZeroedBR * 360.0;

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
  frc::SmartDashboard::PutNumber("Swerve/FL_Target_Deg", targetTurnsFL * 360.0);
  frc::SmartDashboard::PutNumber("Swerve/FR_Target_Deg", targetTurnsFR * 360.0);
  frc::SmartDashboard::PutNumber("Swerve/BL_Target_Deg", targetTurnsBL * 360.0);
  frc::SmartDashboard::PutNumber("Swerve/BR_Target_Deg", targetTurnsBR * 360.0);

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
  frc::SmartDashboard::PutNumber("Swerve/CANcoder_FL_RawTurns", CANcoderFL.GetAbsolutePosition().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Swerve/CANcoder_FR_RawTurns", CANcoderFR.GetAbsolutePosition().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Swerve/CANcoder_BL_RawTurns", CANcoderBL.GetAbsolutePosition().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Swerve/CANcoder_BR_RawTurns", CANcoderBR.GetAbsolutePosition().GetValueAsDouble());

  // Turns telemetry
  frc::SmartDashboard::PutNumber("Swerve/TargetTurns", targetTurnsFL);
  frc::SmartDashboard::PutNumber("Swerve/FL_ErrorTurns", errFL);
  frc::SmartDashboard::PutNumber("Swerve/FR_ErrorTurns", errFR);
  frc::SmartDashboard::PutNumber("Swerve/BL_ErrorTurns", errBL);
  frc::SmartDashboard::PutNumber("Swerve/BR_ErrorTurns", errBR);
  frc::SmartDashboard::PutNumber("Swerve/FL_ZeroedTurns", curZeroedFL);
  frc::SmartDashboard::PutNumber("Swerve/FR_ZeroedTurns", curZeroedFR);
  frc::SmartDashboard::PutNumber("Swerve/BL_ZeroedTurns", curZeroedBL);
  frc::SmartDashboard::PutNumber("Swerve/BR_ZeroedTurns", curZeroedBR);

  // Tight hysteresis deadband: stop at deadbandTurns (0.012 = 4.3 deg)
  // and re-engage if error exceeds deadbandTurns + 0.008 (7.2 deg).
  const double stopBand = deadbandTurns;
  const double startBand = deadbandTurns + 0.008;

  if (std::abs(errFL) > startBand) m_steerActiveFL = true;
  else if (std::abs(errFL) <= stopBand) m_steerActiveFL = false;

  if (std::abs(errFR) > startBand) m_steerActiveFR = true;
  else if (std::abs(errFR) <= stopBand) m_steerActiveFR = false;

  if (std::abs(errBL) > startBand) m_steerActiveBL = true;
  else if (std::abs(errBL) <= stopBand) m_steerActiveBL = false;

  if (std::abs(errBR) > startBand) m_steerActiveBR = true;
  else if (std::abs(errBR) <= stopBand) m_steerActiveBR = false;

  if (deadbandTurns <= 0.0 || m_steerActiveFL) {
    m_FL_Steer.SetControl(steerm_speedFL.WithPosition(units::angle::turn_t{kSwerve3FLZeroTurns + m_contTargetFL}));
  } else {
    m_FL_Steer.Set(0.0);
  }

  // --- Commented out: Old software PID for FL when CANcoder 32 was on "intakebus" ---
  // if (std::abs(errFL) > deadbandTurns) {
  //   iError = std::clamp(iError + errFL * 0.02, -0.1, 0.1);
  //   const double flWheelVel = m_FL_Steer.GetVelocity().GetValueAsDouble() / 18.0;
  //   const double pTerm = errFL * (static_cast<double>(FL_P) * 2.0);
  //   const double iTerm = iError * static_cast<double>(FL_I);
  //   const double dTerm = -flWheelVel * (0.10 * static_cast<double>(FL_D));
  //   double flDuty = pTerm + iTerm + dTerm;
  //   if ((errFL > 0.0 && flDuty < 0.05) || (errFL < 0.0 && flDuty > -0.05)) {
  //     flDuty = std::copysign(0.05, errFL);
  //   }
  //   const double rampDownCap = std::clamp(std::abs(errFL) * 3.5, 0.08, 0.45);
  //   flDuty = std::clamp(flDuty, -rampDownCap, rampDownCap);
  //   m_FL_Steer.Set(flDuty);
  // } else {
  //   iError = 0.0;
  //   m_FL_Steer.Set(0.0);
  // }

  if (deadbandTurns <= 0.0 || m_steerActiveFR) {
    m_FR_Steer.SetControl(steerm_speedFR.WithPosition(units::angle::turn_t{kSwerve2FRZeroTurns + m_contTargetFR}));
  } else {
    m_FR_Steer.Set(0.0);
  }

  if (deadbandTurns <= 0.0 || m_steerActiveBL) {
    m_BL_Steer.SetControl(steerm_speedBL.WithPosition(units::angle::turn_t{kSwerve4BLZeroTurns + m_contTargetBL}));
  } else {
    m_BL_Steer.Set(0.0);
  }

  if (deadbandTurns <= 0.0 || m_steerActiveBR) {
    m_BR_Steer.SetControl(steerm_speedBR.WithPosition(units::angle::turn_t{kSwerve1BRZeroTurns + m_contTargetBR}));
  } else {
    m_BR_Steer.Set(0.0);
  }

  // Synchronize drive output with module steering alignment so drive wheels do not
  // push the robot sideways while the modules are still turning toward their target angle:
  const double maxAbsErrTurns = std::max({std::abs(errFL), std::abs(errFR), std::abs(errBL), std::abs(errBR)});
  const double alignScale = std::max(0.0, std::cos(std::min(maxAbsErrTurns, 0.25) * 2.0 * M_PI));
  speedFL *= alignScale;
  speedFR *= alignScale;
  speedBL *= alignScale;
  speedBR *= alignScale;

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

  // Swerve module state arrays formatted for Reflect / AdvantageScope / Shuffleboard
  std::vector<double> measuredStates = {
      FL_pos, wheelSpeedFL,
      FR_pos, wheelSpeedFR,
      BL_pos, wheelSpeedBL,
      BR_pos, wheelSpeedBR
  };
  std::vector<double> desiredStates = {
      angleFL, speedFL * 4.0,
      angleFR, speedFR * 4.0,
      angleBL, speedBL * 4.0,
      angleBR, speedBR * 4.0
  };
  std::vector<double> moduleAnglesDeg = {curFLDeg, curFRDeg, curBLDeg, curBRDeg};
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
