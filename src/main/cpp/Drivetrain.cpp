// ============================================================================
// FRC Team 8753 - 2026 Swerve Drivetrain implementation (ChimiSwerve style)
// ============================================================================

#include "Drivetrain.h"

#include <algorithm>
#include <cmath>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>

using namespace ctre::phoenix6;
namespace DC = DrivetrainConstants;

namespace {
constexpr double kPi = 3.14159265358979323846;

/// Wheel ground speed (m/s) from drive motor velocity (rotor turns/s).
double WheelMps(double motorRps) {
  return motorRps / DC::DriveGearRatio * (kPi * DC::WheelDiameter);
}

/// Kinematic angle (rad, CW+) of a wheel from its steer sensor turns (CCW+).
double SensorTurnsToKinematicRad(double turns) {
  return -std::remainder(turns, 1.0) * 2.0 * kPi;
}
}  // namespace

Drivetrain::Drivetrain()
    : m_FL_Drive{DC::kFLDriveId, CANBus{DC::kCanBus}},
      m_FR_Drive{DC::kFRDriveId, CANBus{DC::kCanBus}},
      m_BL_Drive{DC::kBLDriveId, CANBus{DC::kCanBus}},
      m_BR_Drive{DC::kBRDriveId, CANBus{DC::kCanBus}},
      m_FL_Steer{DC::kFLSteerId, CANBus{DC::kCanBus}},
      m_FR_Steer{DC::kFRSteerId, CANBus{DC::kCanBus}},
      m_BL_Steer{DC::kBLSteerId, CANBus{DC::kCanBus}},
      m_BR_Steer{DC::kBRSteerId, CANBus{DC::kCanBus}},
      CANcoderFL{DC::kFLEncoderId, CANBus{DC::kCanBus}},
      CANcoderFR{DC::kFREncoderId, CANBus{DC::kCanBus}},
      CANcoderBL{DC::kBLEncoderId, CANBus{DC::kCanBus}},
      CANcoderBR{DC::kBREncoderId, CANBus{DC::kCanBus}} {}

// ============================================================================
// ConfigureMotors
// ============================================================================
void Drivetrain::ConfigureMotors() {
  bool ok = true;

  // ---------- Drive motors: brake, current limit, open-loop ramp ----------
  auto applyDrive = [&](hardware::TalonFX& motor, bool inverted) {
    configs::TalonFXConfiguration cfg{};
    cfg.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
    cfg.MotorOutput.Inverted = inverted ? signals::InvertedValue::Clockwise_Positive
                                        : signals::InvertedValue::CounterClockwise_Positive;
    cfg.CurrentLimits.StatorCurrentLimit = units::ampere_t{DC::kDriveCurrentLimit};
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = units::second_t{DC::DriveOpenLoopRamp};
    ok &= motor.GetConfigurator().Apply(cfg).IsOK();
  };
  applyDrive(m_FL_Drive, DC::kFLDriveInverted);
  applyDrive(m_FR_Drive, DC::kFRDriveInverted);
  applyDrive(m_BL_Drive, DC::kBLDriveInverted);
  applyDrive(m_BR_Drive, DC::kBRDriveInverted);

  // ---------- CANcoders (only if offsets are managed in code) ----------
  if (DC::kWriteMagnetOffsets) {
    auto applyEncoder = [&](hardware::CANcoder& enc, double offset) {
      configs::CANcoderConfiguration cfg{};
      cfg.MagnetSensor.MagnetOffset = units::turn_t{offset};
      cfg.MagnetSensor.SensorDirection = signals::SensorDirectionValue::CounterClockwise_Positive;
      cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = units::turn_t{0.5};
      ok &= enc.GetConfigurator().Apply(cfg).IsOK();
    };
    applyEncoder(CANcoderFL, DC::kFLMagnetOffset);
    applyEncoder(CANcoderFR, DC::kFRMagnetOffset);
    applyEncoder(CANcoderBL, DC::kBLMagnetOffset);
    applyEncoder(CANcoderBR, DC::kBRMagnetOffset);
  }

  // ---------- Steer motors: onboard position loop on the RemoteCANcoder ----------
  auto applySteer = [&](hardware::TalonFX& motor, int canCoderId) {
    configs::TalonFXConfiguration cfg{};
    cfg.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
    cfg.MotorOutput.Inverted = DC::kSteerMotorInverted
                                   ? signals::InvertedValue::Clockwise_Positive
                                   : signals::InvertedValue::CounterClockwise_Positive;
    cfg.MotorOutput.PeakForwardDutyCycle = DC::kSteerPeakOutput;
    cfg.MotorOutput.PeakReverseDutyCycle = -DC::kSteerPeakOutput;
    cfg.CurrentLimits.StatorCurrentLimit = units::ampere_t{DC::kSteerCurrentLimit};
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.Feedback.FeedbackRemoteSensorID = canCoderId;
    cfg.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::RemoteCANcoder;
    cfg.Feedback.SensorToMechanismRatio = DC::sensorToMechanismRatio;
    cfg.Feedback.RotorToSensorRatio = DC::rotorToSensorRatio;
    cfg.ClosedLoopGeneral.ContinuousWrap = true;
    cfg.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = units::second_t{DC::SteeringRampRateSeconds};
    cfg.Slot0.kP = DC::ModuleP;
    cfg.Slot0.kI = DC::ModuleI;
    cfg.Slot0.kD = DC::ModuleD;
    ok &= motor.GetConfigurator().Apply(cfg).IsOK();
  };
  applySteer(m_FL_Steer, DC::kFLEncoderId);
  applySteer(m_FR_Steer, DC::kFREncoderId);
  applySteer(m_BL_Steer, DC::kBLEncoderId);
  applySteer(m_BR_Steer, DC::kBREncoderId);

  hasBootstrapped = false;
  frc::SmartDashboard::PutBoolean("Swerve/ConfigsApplied", ok);
}

// ============================================================================
// Disabled
// ============================================================================
void Drivetrain::DisabledInit() {
  SetDrive(0.0, 0.0, 0.0, 0.0);
}

void Drivetrain::DisabledPeriodic() {
  ReadSteerTurns();

  // Keep setpoints equal to where the wheels physically are -> no snap on enable.
  lastTargetTurnsFL = turnsFL;
  lastTargetTurnsFR = turnsFR;
  lastTargetTurnsBL = turnsBL;
  lastTargetTurnsBR = turnsBR;
  hasBootstrapped = true;

  // Alignment readout [-180, 180] deg: every wheel pointing forward should read ~0.
  frc::SmartDashboard::PutNumber("Swerve/FL_CANcoderDeg", std::remainder(turnsFL, 1.0) * 360.0);
  frc::SmartDashboard::PutNumber("Swerve/FR_CANcoderDeg", std::remainder(turnsFR, 1.0) * 360.0);
  frc::SmartDashboard::PutNumber("Swerve/BL_CANcoderDeg", std::remainder(turnsBL, 1.0) * 360.0);
  frc::SmartDashboard::PutNumber("Swerve/BR_CANcoderDeg", std::remainder(turnsBR, 1.0) * 360.0);
  PublishTelemetry();
}

// ============================================================================
// Inverse kinematics (ChimiSwerve / Ether)
// ============================================================================
ModuleStates Drivetrain::InverseKinematics(double FWD, double STR, double RCW) {
  const double A = STR - RCW * (DC::L / DC::R);
  const double B = STR + RCW * (DC::L / DC::R);
  const double C = FWD - RCW * (DC::W / DC::R);
  const double D = FWD + RCW * (DC::W / DC::R);

  ModuleStates s;
  s.wsFR = std::hypot(B, C);  s.waFR = std::atan2(B, C);
  s.wsFL = std::hypot(B, D);  s.waFL = std::atan2(B, D);
  s.wsBL = std::hypot(A, D);  s.waBL = std::atan2(A, D);
  s.wsBR = std::hypot(A, C);  s.waBR = std::atan2(A, C);

  // Normalize so no wheel is commanded above 1.0
  const double max = std::max({s.wsFL, s.wsFR, s.wsBL, s.wsBR});
  if (max > 1.0) {
    s.wsFL /= max; s.wsFR /= max; s.wsBL /= max; s.wsBR /= max;
  }
  return s;
}

// ============================================================================
// Azimuth optimization (inversion awareness + cosine scaling)
// ============================================================================
double Drivetrain::ChimiOptimizeAzimuth(double waRad, double currentTurns, double& ws) {
  // Kinematic angle is CW+, sensor is CCW+ -> negate.
  const double targetTurns = -waRad / (2.0 * kPi);

  // Shortest error in [-0.5, 0.5] turns
  double errorTurns = std::remainder(targetTurns - currentTurns, 1.0);

  // More than 90 deg away: point the other way and drive backward instead.
  if (std::abs(errorTurns) > 0.25) {
    errorTurns -= std::copysign(0.5, errorTurns);
    ws = -ws;
  }

  // Don't push while the wheel is still pointing the wrong way.
  ws *= std::max(0.0, std::cos(errorTurns * 2.0 * kPi));

  return currentTurns + errorTurns;
}

// ============================================================================
// Update - main swerve loop
// ============================================================================
void Drivetrain::Update(double x, double y, double x2, double GyroValue,
                        double triggerL, double triggerR, bool FieldCentric) {
  ReadSteerTurns();

  if (!hasBootstrapped) {
    lastTargetTurnsFL = turnsFL;
    lastTargetTurnsFR = turnsFR;
    lastTargetTurnsBL = turnsBL;
    lastTargetTurnsBR = turnsBR;
    storedYaw = GyroValue;
    hasBootstrapped = true;
  }

  // Live wheel angles/speeds for odometry
  const double waFL = SensorTurnsToKinematicRad(turnsFL);
  const double waFR = SensorTurnsToKinematicRad(turnsFR);
  const double waBL = SensorTurnsToKinematicRad(turnsBL);
  const double waBR = SensorTurnsToKinematicRad(turnsBR);
  const double vFL = WheelMps(m_FL_Drive.GetVelocity().GetValueAsDouble());
  const double vFR = WheelMps(m_FR_Drive.GetVelocity().GetValueAsDouble());
  const double vBL = WheelMps(m_BL_Drive.GetVelocity().GetValueAsDouble());
  const double vBR = WheelMps(m_BR_Drive.GetVelocity().GetValueAsDouble());
  odometryUpdate(waFL, waFR, waBL, waBR, vFL, vFR, vBL, vBR, GyroValue);

  const bool isTranslating = (x != 0.0 || y != 0.0);
  const bool isRotating = (x2 != 0.0);

  // ---------- Neutral: stop driving, hold wheel angles ----------
  if (!isTranslating && !isRotating) {
    storedYaw = GyroValue;
    SetDrive(0.0, 0.0, 0.0, 0.0);
    HoldSteer();
    PublishTelemetry();
    return;
  }

  // ---------- Field-centric transform ----------
  double FWD = y;
  double STR = x;
  if (FieldCentric) {
    const double g = GyroValue * kPi / 180.0;
    const double temp = FWD * std::cos(g) + STR * std::sin(g);
    STR = -FWD * std::sin(g) + STR * std::cos(g);
    FWD = temp;
  }

  // ---------- Rotation / straight-drive heading lock ----------
  double RCW = 0.0;
  if (isRotating) {
    RCW = x2;
    storedYaw = GyroValue;
  } else if (DC::kEnableHeadingLock) {
    const double errorDeg = std::remainder(storedYaw - GyroValue, 360.0);
    RCW = std::clamp(errorDeg * DC::straightP, -DC::rotationClamp, DC::rotationClamp);
  }

  // ---------- Kinematics + optimization ----------
  ModuleStates s = InverseKinematics(FWD, STR, RCW);
  lastTargetTurnsFL = ChimiOptimizeAzimuth(s.waFL, turnsFL, s.wsFL);
  lastTargetTurnsFR = ChimiOptimizeAzimuth(s.waFR, turnsFR, s.wsFR);
  lastTargetTurnsBL = ChimiOptimizeAzimuth(s.waBL, turnsBL, s.wsBL);
  lastTargetTurnsBR = ChimiOptimizeAzimuth(s.waBR, turnsBR, s.wsBR);
  HoldSteer();

  // ---------- Speed scaling (triggers) ----------
  double pct = DC::DefaultDriveSpeed + (triggerR - triggerL) * DC::TriggerConstant;
  pct = std::clamp(pct, DC::MinDriveSpeed, 100.0);
  const double scale = pct / 100.0;
  frc::SmartDashboard::PutNumber("Swerve/SpeedPercent", pct);

  if (std::isnan(s.wsFL) || std::isnan(s.wsFR) || std::isnan(s.wsBL) || std::isnan(s.wsBR)) {
    SetDrive(0.0, 0.0, 0.0, 0.0);
  } else {
    SetDrive(s.wsFL * scale, s.wsFR * scale, s.wsBL * scale, s.wsBR * scale);
  }
  PublishTelemetry();
}

// ============================================================================
// Helpers
// ============================================================================
void Drivetrain::ReadSteerTurns() {
  turnsFL = CANcoderFL.GetPosition().GetValueAsDouble();
  turnsFR = CANcoderFR.GetPosition().GetValueAsDouble();
  turnsBL = CANcoderBL.GetPosition().GetValueAsDouble();
  turnsBR = CANcoderBR.GetPosition().GetValueAsDouble();
}

void Drivetrain::HoldSteer() {
  m_FL_Steer.SetControl(steerRequest.WithPosition(units::turn_t{lastTargetTurnsFL}));
  m_FR_Steer.SetControl(steerRequest.WithPosition(units::turn_t{lastTargetTurnsFR}));
  m_BL_Steer.SetControl(steerRequest.WithPosition(units::turn_t{lastTargetTurnsBL}));
  m_BR_Steer.SetControl(steerRequest.WithPosition(units::turn_t{lastTargetTurnsBR}));
}

void Drivetrain::SetDrive(double fl, double fr, double bl, double br) {
  const double lim = DC::DriveMotorsHardLimit;
  cmdFL = std::clamp(fl, -lim, lim);
  cmdFR = std::clamp(fr, -lim, lim);
  cmdBL = std::clamp(bl, -lim, lim);
  cmdBR = std::clamp(br, -lim, lim);
  m_FL_Drive.SetControl(driveRequest.WithOutput(cmdFL));
  m_FR_Drive.SetControl(driveRequest.WithOutput(cmdFR));
  m_BL_Drive.SetControl(driveRequest.WithOutput(cmdBL));
  m_BR_Drive.SetControl(driveRequest.WithOutput(cmdBR));
}

void Drivetrain::PublishTelemetry() {
  frc::SmartDashboard::PutNumber("Swerve/FL_TargetTurns", lastTargetTurnsFL);
  frc::SmartDashboard::PutNumber("Swerve/FR_TargetTurns", lastTargetTurnsFR);
  frc::SmartDashboard::PutNumber("Swerve/BL_TargetTurns", lastTargetTurnsBL);
  frc::SmartDashboard::PutNumber("Swerve/BR_TargetTurns", lastTargetTurnsBR);
  frc::SmartDashboard::PutNumber("Swerve/FL_ActualTurns", turnsFL);
  frc::SmartDashboard::PutNumber("Swerve/FR_ActualTurns", turnsFR);
  frc::SmartDashboard::PutNumber("Swerve/BL_ActualTurns", turnsBL);
  frc::SmartDashboard::PutNumber("Swerve/BR_ActualTurns", turnsBR);
  frc::SmartDashboard::PutNumber("Swerve/FL_DriveCmd", cmdFL);
  frc::SmartDashboard::PutNumber("Swerve/FR_DriveCmd", cmdFR);
  frc::SmartDashboard::PutNumber("Swerve/BL_DriveCmd", cmdBL);
  frc::SmartDashboard::PutNumber("Swerve/BR_DriveCmd", cmdBR);
  frc::SmartDashboard::PutNumber("Swerve/StoredYaw", storedYaw);
  frc::SmartDashboard::PutBoolean("Swerve/FL_EncoderConnected", CANcoderFL.IsConnected());
  frc::SmartDashboard::PutBoolean("Swerve/FR_EncoderConnected", CANcoderFR.IsConnected());
  frc::SmartDashboard::PutBoolean("Swerve/BL_EncoderConnected", CANcoderBL.IsConnected());
  frc::SmartDashboard::PutBoolean("Swerve/BR_EncoderConnected", CANcoderBR.IsConnected());
}

// ============================================================================
// Odometry (forward kinematics)
// ============================================================================
void Drivetrain::odometryUpdate(double waFL, double waFR, double waBL, double waBR,
                                double wsFL, double wsFR, double wsBL, double wsBR,
                                double GyroValue) {
  const double now = frc::Timer::GetFPGATimestamp().value();
  const double dt = now - odoLastTime;
  odoLastTime = now;
  if (dt <= 0.0 || dt > 0.5) return;

  // Robot-relative velocity = average of the four wheel vectors
  // (rotation components cancel on a symmetric chassis).
  odoFWD = (wsFL * std::cos(waFL) + wsFR * std::cos(waFR) +
            wsBL * std::cos(waBL) + wsBR * std::cos(waBR)) / 4.0;
  odoSTR = (wsFL * std::sin(waFL) + wsFR * std::sin(waFR) +
            wsBL * std::sin(waBL) + wsBR * std::sin(waBR)) / 4.0;

  // Robot frame -> field frame (inverse of the field-centric transform)
  const double g = GyroValue * kPi / 180.0;
  const double fieldFWD = odoFWD * std::cos(g) - odoSTR * std::sin(g);
  const double fieldSTR = odoFWD * std::sin(g) + odoSTR * std::cos(g);

  positionFWDField += fieldFWD * dt;
  positionSTRField += fieldSTR * dt;

  frc::SmartDashboard::PutNumber("Odometry/FieldFWD", positionFWDField);
  frc::SmartDashboard::PutNumber("Odometry/FieldSTR", positionSTRField);
  frc::SmartDashboard::PutNumber("Odometry/RobotFWD_mps", odoFWD);
  frc::SmartDashboard::PutNumber("Odometry/RobotSTR_mps", odoSTR);
}

// ============================================================================
// Simulation: wheels snap to their setpoints, drive speed follows duty
// ============================================================================
void Drivetrain::UpdateSim(units::time::second_t dt) {
  constexpr double kMaxRps = 100.0;  // rotor turns/s at full duty

  auto simDrive = [&](hardware::TalonFX& motor, bool inverted, double cmd) {
    auto& sim = motor.GetSimState();
    sim.Orientation = inverted ? sim::ChassisReference::Clockwise_Positive
                               : sim::ChassisReference::CounterClockwise_Positive;
    const double rps = cmd * kMaxRps;
    sim.SetRotorVelocity(units::turns_per_second_t{rps});
    sim.AddRotorPosition(units::turn_t{rps * dt.value()});
  };
  simDrive(m_FL_Drive, DC::kFLDriveInverted, cmdFL);
  simDrive(m_FR_Drive, DC::kFRDriveInverted, cmdFR);
  simDrive(m_BL_Drive, DC::kBLDriveInverted, cmdBL);
  simDrive(m_BR_Drive, DC::kBRDriveInverted, cmdBR);

  CANcoderFL.GetSimState().SetRawPosition(units::turn_t{lastTargetTurnsFL});
  CANcoderFR.GetSimState().SetRawPosition(units::turn_t{lastTargetTurnsFR});
  CANcoderBL.GetSimState().SetRawPosition(units::turn_t{lastTargetTurnsBL});
  CANcoderBR.GetSimState().SetRawPosition(units::turn_t{lastTargetTurnsBR});
}
