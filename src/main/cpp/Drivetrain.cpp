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
  m_FL_Steer.Set(0.0);
  m_FR_Steer.Set(0.0);
  m_BL_Steer.Set(0.0);
  m_BR_Steer.Set(0.0);
  m_FL_Drive.Set(0.0);
  m_FR_Drive.Set(0.0);
  m_BL_Drive.Set(0.0);
  m_BR_Drive.Set(0.0);
}

void Drivetrain::TestRotateModules(double speed) {
  m_FL_Steer.Set(0.0);
  m_FR_Steer.Set(0.0);
  m_BL_Steer.Set(0.0);
  m_BR_Steer.Set(0.0);
  m_FL_Drive.Set(0.0);
  m_FR_Drive.Set(0.0);
  m_BL_Drive.Set(0.0);
  m_BR_Drive.Set(0.0);
}

void Drivetrain::Update(double x, double y, double x2, double GyroValue,
                        double triggerL, double triggerR, bool FieldCentric,
                        bool isTankDrive, bool holdSteer) {
  SyncSmartDashboardPID();

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

  // Swerve fully disabled: Steer motors permanently commanded 0.0 power
  // In Brake mode, modules stay locked mechanically where manually aligned
  m_FL_Steer.Set(0.0);
  m_FR_Steer.Set(0.0);
  m_BL_Steer.Set(0.0);
  m_BR_Steer.Set(0.0);

  // ========== TANK DRIVE ONLY (Swerve Fully Disabled) ==========
  // Forward/backward throttle from Left Y (y)
  // Turn from Right Stick X (x2) or Left Stick X (x) if right stick centered
  double throttle = y;
  double turn = (std::abs(x2) > 0.05) ? x2 : x;
  auto [leftSpeed, rightSpeed] = TankDriveKinematics(throttle, turn);

  speedFL = leftSpeed;
  speedBL = leftSpeed;
  speedFR = rightSpeed;
  speedBR = rightSpeed;

  frc::SmartDashboard::PutNumber("Tank/LeftSpeed", leftSpeed);
  frc::SmartDashboard::PutNumber("Tank/RightSpeed", rightSpeed);
  frc::SmartDashboard::PutBoolean("Drive/IsTankDrive", true);

  // Smooth acceleration ramps
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
