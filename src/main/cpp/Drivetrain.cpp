// ============================================================================
// FRC Team 8753 - 2026 Swerve Drivetrain Implementation (ChimiSwerve White Paper)
// ============================================================================

#include "Drivetrain.h"

#include <algorithm>
#include <cmath>
#include <string>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/frequency.h>

using namespace ctre::phoenix6;
using namespace units::literals;
namespace DC = DrivetrainConstants;

namespace {
constexpr double kPi = 3.14159265358979323846;

/// Wheel ground speed (m/s) from drive motor velocity (rotor turns/s).
/// ChimiSwerve Section 2.2b (Page 18): revolutions/sec / gearRatio * (pi * diameter)
double WheelMps(double motorRps) {
  return (motorRps / DC::DriveGearRatio) * (kPi * DC::WheelDiameter);
}

/// Kinematic angle (rad, CW+) of a wheel from its CANcoder turns.
double SensorTurnsToKinematicRad(double turns) {
  return std::remainder(turns, 1.0) * 2.0 * kPi;
}
}  // namespace

// Note: CANcoderFL (ID 32, Swerve 3) is on the "intake" CAN bus; all other swerve
// motors and CANcoders (12, 22, 42) are on the "rio" CAN bus.
Drivetrain::Drivetrain()
    : m_FL_Drive{DC::kFLDriveId, CANBus{DC::kCanBus}},
      m_FR_Drive{DC::kFRDriveId, CANBus{DC::kCanBus}},
      m_BL_Drive{DC::kBLDriveId, CANBus{DC::kCanBus}},
      m_BR_Drive{DC::kBRDriveId, CANBus{DC::kCanBus}},
      m_FL_Steer{DC::kFLSteerId, CANBus{DC::kCanBus}},
      m_FR_Steer{DC::kFRSteerId, CANBus{DC::kCanBus}},
      m_BL_Steer{DC::kBLSteerId, CANBus{DC::kCanBus}},
      m_BR_Steer{DC::kBRSteerId, CANBus{DC::kCanBus}},
      CANcoderFL{DC::kFLEncoderId, CANBus{DC::kIntakeCanBus}},
      CANcoderFR{DC::kFREncoderId, CANBus{DC::kCanBus}},
      CANcoderBL{DC::kBLEncoderId, CANBus{DC::kCanBus}},
      CANcoderBR{DC::kBREncoderId, CANBus{DC::kCanBus}} {}

// ============================================================================
// ConfigureMotors (applies motor configs & new CANcoder magnet offsets)
// ============================================================================
void Drivetrain::ConfigureMotors() {
  bool ok = true;

  auto applyWithRetry = [](hardware::TalonFX& motor,
                           const configs::TalonFXConfiguration& cfg,
                           const char* name) -> bool {
    for (int attempt = 1; attempt <= 3; ++attempt) {
      auto status = motor.GetConfigurator().Apply(cfg, 50_ms);
      if (status.IsOK()) {
        frc::SmartDashboard::PutString(std::string("Swerve/Config_") + name, "OK");
        return true;
      }
      if (attempt == 3) {
        frc::SmartDashboard::PutString(std::string("Swerve/Config_") + name, status.GetName());
      }
    }
    return false;
  };

  // ---------- Drive motors: brake, current limit, voltage & open-loop ramps ----------
  // ChimiSwerve Section 3.6 (Pages 40-41): Ramp rate + current limiting prevent brownouts
  auto applyDrive = [&](hardware::TalonFX& motor, bool inverted, const char* name) {
    configs::TalonFXConfiguration cfg{};
    cfg.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
    cfg.MotorOutput.Inverted = inverted ? signals::InvertedValue::Clockwise_Positive
                                        : signals::InvertedValue::CounterClockwise_Positive;
    cfg.CurrentLimits.StatorCurrentLimit = units::ampere_t{DC::kDriveCurrentLimit};
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.ClosedLoopRamps.VoltageClosedLoopRampPeriod = units::second_t{DC::DriveRampRateSeconds};
    cfg.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = units::second_t{DC::DriveOpenLoopRamp};
    ok &= applyWithRetry(motor, cfg, name);
  };
  applyDrive(m_FL_Drive, DC::kFLDriveInverted, "FL_Drive");
  applyDrive(m_FR_Drive, DC::kFRDriveInverted, "FR_Drive");
  applyDrive(m_BL_Drive, DC::kBLDriveInverted, "BL_Drive");
  applyDrive(m_BR_Drive, DC::kBRDriveInverted, "BR_Drive");

  // ---------- CANcoders: write magnet offsets for Swerve 1-4 ----------
  if (DC::kWriteMagnetOffsets) {
    auto applyEncoder = [&](hardware::CANcoder& enc, double offset, const char* name) {
      configs::MagnetSensorConfigs magCfg{};
      // Refresh only MagnetSensorConfigs so SensorDirection/DiscontinuityPoint are preserved
      enc.GetConfigurator().Refresh(magCfg, 50_ms);
      magCfg.MagnetOffset = units::turn_t{offset};
      for (int attempt = 1; attempt <= 2; ++attempt) {
        auto status = enc.GetConfigurator().Apply(magCfg, 50_ms);
        if (status.IsOK()) {
          frc::SmartDashboard::PutString(std::string("Swerve/Config_") + name, "OK");
          break;
        }
        if (attempt == 2) {
          frc::SmartDashboard::PutString(std::string("Swerve/Config_") + name, status.GetName());
        }
      }
    };
    applyEncoder(CANcoderBR, DC::kBRMagnetOffset, "BR_CANcoder12"); // Swerve 1: -0.3969
    applyEncoder(CANcoderFR, DC::kFRMagnetOffset, "FR_CANcoder22"); // Swerve 2:  0.168701
    applyEncoder(CANcoderFL, DC::kFLMagnetOffset, "FL_CANcoder32"); // Swerve 3:  0.599121 ("intakebus")
    applyEncoder(CANcoderBL, DC::kBLMagnetOffset, "BL_CANcoder42"); // Swerve 4:  0.329102
  }

  // Publish active magnet offsets to SmartDashboard
  frc::SmartDashboard::PutNumber("Swerve/BR_MagnetOffsetTurns", DC::kBRMagnetOffset);
  frc::SmartDashboard::PutNumber("Swerve/FR_MagnetOffsetTurns", DC::kFRMagnetOffset);
  frc::SmartDashboard::PutNumber("Swerve/FL_MagnetOffsetTurns", DC::kFLMagnetOffset);
  frc::SmartDashboard::PutNumber("Swerve/BL_MagnetOffsetTurns", DC::kBLMagnetOffset);

  // ---------- Steer motors (Onboard 1000Hz Slot 0 PositionDutyCycle on all 4 modules) ----------
  auto applySteer = [&](hardware::TalonFX& motor, int canCoderId, bool sameCanBus, const char* name) {
    configs::TalonFXConfiguration cfg{};
    cfg.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
    cfg.MotorOutput.Inverted = DC::kSteerMotorInverted
                                   ? signals::InvertedValue::Clockwise_Positive
                                   : signals::InvertedValue::CounterClockwise_Positive;
    cfg.MotorOutput.PeakForwardDutyCycle = DC::kSteerPeakOutput;
    cfg.MotorOutput.PeakReverseDutyCycle = -DC::kSteerPeakOutput;
    cfg.CurrentLimits.StatorCurrentLimit = units::ampere_t{DC::kSteerCurrentLimit};
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.ClosedLoopGeneral.ContinuousWrap = true;
    cfg.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = units::second_t{DC::SteeringRampRateSeconds};
    cfg.Slot0.kP = DC::ModuleP;
    cfg.Slot0.kI = DC::ModuleI;
    cfg.Slot0.kD = DC::ModuleD;
    if (sameCanBus) {
      cfg.Feedback.FeedbackRemoteSensorID = canCoderId;
      cfg.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::RemoteCANcoder;
      cfg.Feedback.SensorToMechanismRatio = DC::sensorToMechanismRatio;
      cfg.Feedback.RotorToSensorRatio = DC::rotorToSensorRatio;
    } else {
      // FL Steer (ID 31 on "rio") uses its internal RotorSensor scaled by 18:1 (rotorToSensorRatio)
      // and is seeded from CANcoder 32 (on "intakebus"), so it runs the exact same 1000Hz onboard PID!
      cfg.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::RotorSensor;
      cfg.Feedback.SensorToMechanismRatio = DC::rotorToSensorRatio;
    }
    ok &= applyWithRetry(motor, cfg, name);
  };
  applySteer(m_FL_Steer, DC::kFLEncoderId, false, "FL_Steer"); // CANcoder 32 is on "intakebus"
  applySteer(m_FR_Steer, DC::kFREncoderId, true,  "FR_Steer"); // CANcoder 22 is on "rio" bus
  applySteer(m_BL_Steer, DC::kBLEncoderId, true,  "BL_Steer"); // CANcoder 42 is on "rio" bus
  applySteer(m_BR_Steer, DC::kBREncoderId, true,  "BR_Steer"); // CANcoder 12 is on "rio" bus

  SyncFLSteerToCANcoder();
  hasBootstrapped = false;
  m_configsApplied = ok;
  frc::SmartDashboard::PutBoolean("Swerve/ConfigsApplied", ok);
}

// ============================================================================
// Disabled
// ============================================================================
void Drivetrain::DisabledInit() {
  SetDrive(0.0, 0.0, 0.0, 0.0);
  StopSteer();
}

void Drivetrain::DisabledPeriodic() {
  if (!m_configsApplied) {
    static int retryLoops = 0;
    if (++retryLoops <= 150 && (retryLoops % 50 == 0)) {
      ConfigureMotors();
    }
  }

  SyncFLSteerToCANcoder();
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
// Inverse Kinematics (ChimiSwerve Section 2.1b, Pages 13-15)
// ============================================================================
ModuleStates Drivetrain::InverseKinematics(double FWD, double STR, double ROT) {
  // ChimiSwerve Page 14:
  //   A = STR - ROT * (L / R)
  //   B = STR + ROT * (L / R)
  //   C = FWD - ROT * (W / R)
  //   D = FWD + ROT * (W / R)
  const double A = STR - ROT * (DC::L / DC::R);
  const double B = STR + ROT * (DC::L / DC::R);
  const double C = FWD - ROT * (DC::W / DC::R);
  const double D = FWD + ROT * (DC::W / DC::R);

  ModuleStates s;
  s.wsFR = std::hypot(B, C);  s.waFR = std::atan2(B, C);
  s.wsFL = std::hypot(B, D);  s.waFL = std::atan2(B, D);
  s.wsBR = std::hypot(A, C);  s.waBR = std::atan2(A, C);
  s.wsBL = std::hypot(A, D);  s.waBL = std::atan2(A, D);

  // ChimiSwerve Page 15: Normalize wheel speeds if max > 1.0
  const double maxSpeed = std::max({s.wsFL, s.wsFR, s.wsBL, s.wsBR});
  if (maxSpeed > 1.0) {
    s.wsFL /= maxSpeed;
    s.wsFR /= maxSpeed;
    s.wsBL /= maxSpeed;
    s.wsBR /= maxSpeed;
  }
  return s;
}

// ============================================================================
// Azimuth Optimization & Inversion Awareness (ChimiSwerve Section 2.1b, Page 16)
// ============================================================================
double Drivetrain::ChimiOptimizeAzimuth(double waRad, double currentTurns, double& ws) {
  // Convert target azimuth (radians) to sensor turns
  const double targetTurns = waRad / (2.0 * kPi);

  // ChimiSwerve Page 16: azimuthError = remainder(azimuthAngle - wa)
  double errorTurns = std::remainder(targetTurns - currentTurns, 1.0);

  // ChimiSwerve Page 16 Inversion Awareness:
  // If |azimuthError| > 90 deg (0.25 turns), flip azimuth by 180 deg (0.5 turns)
  // and invert the drive motor speed.
  if (std::abs(errorTurns) > 0.25) {
    errorTurns -= std::copysign(0.5, errorTurns);
    ws = -ws;
  }

  // CANcoder static buffer: ignore tiny sensor fluctuations (< kSteerDeadbandTurns)
  if (std::abs(errorTurns) < DC::kSteerDeadbandTurns) {
    errorTurns = 0.0;
  }

  return currentTurns + errorTurns;
}

double Drivetrain::MinimizeRotation(double targetAngleRad, double currentAngleRad, double& speedPercent) {
  double errorRad = std::remainder(targetAngleRad - currentAngleRad, 2.0 * kPi);
  if (std::abs(errorRad) > kPi / 2.0) {
    errorRad -= std::copysign(kPi, errorRad);
    speedPercent = -speedPercent;
  }
  if (std::abs(errorRad) < DC::kSteerDeadbandTurns * 2.0 * kPi) {
    errorRad = 0.0;
  }
  return std::remainder(currentAngleRad + errorRad, 2.0 * kPi);
}

// ============================================================================
// Straight-Drive Heading Lock (ChimiSwerve Section 2.3d, Pages 34-35)
// ============================================================================
double Drivetrain::calcYawStraight(double targetAngle, double currentAngle, double kP) {
  const double errorAngle = std::remainder(targetAngle - currentAngle, 360.0);
  return errorAngle * kP;
}

// ============================================================================
// Update - Main ChimiSwerve Control Loop
// ============================================================================
void Drivetrain::Update(double x, double y, double x2, double GyroValue,
                        double triggerL, double triggerR, bool FieldCentric) {
  ReadSteerTurns();

  const double now = frc::Timer::GetFPGATimestamp().value();
  double dt = now - m_lastLoopTime;
  m_lastLoopTime = now;
  if (dt <= 0.0001 || dt > 0.5) {
    dt = 0.02;
  }

  if (!hasBootstrapped) {
    lastTargetTurnsFL = turnsFL;
    lastTargetTurnsFR = turnsFR;
    lastTargetTurnsBL = turnsBL;
    lastTargetTurnsBR = turnsBR;
    storedYaw = GyroValue;
    m_headingIError = 0.0;
    m_headingLastError = 0.0;
    hasBootstrapped = true;
  }

  // Live wheel angles/speeds for odometry (ChimiSwerve Section 2.2b)
  const double waFL = SensorTurnsToKinematicRad(turnsFL);
  const double waFR = SensorTurnsToKinematicRad(turnsFR);
  const double waBL = SensorTurnsToKinematicRad(turnsBL);
  const double waBR = SensorTurnsToKinematicRad(turnsBR);
  const double vFL = WheelMps(m_FL_Drive.GetVelocity().GetValueAsDouble());
  const double vFR = WheelMps(m_FR_Drive.GetVelocity().GetValueAsDouble());
  const double vBL = WheelMps(m_BL_Drive.GetVelocity().GetValueAsDouble());
  const double vBR = WheelMps(m_BR_Drive.GetVelocity().GetValueAsDouble());

  const bool isTranslating = (std::abs(x) > 1e-3 || std::abs(y) > 1e-3);
  const bool isRotating = (std::abs(x2) > 1e-3);

  // ---------- Neutral: stop drive & steer motors immediately to prevent idle jitter ----------
  // ChimiSwerve Page 35: "we don't want to PID our angle if we aren't trying to move"
  if (!isTranslating && !isRotating) {
    storedYaw = GyroValue;
    m_headingIError = 0.0;
    m_headingLastError = 0.0;
    SetDrive(0.0, 0.0, 0.0, 0.0);
    StopSteer();
    SyncFLSteerToCANcoder();
    odometryUpdate(waFL, waFR, waBL, waBR, 0.0, 0.0, 0.0, 0.0, GyroValue);
    PublishTelemetry();
    return;
  }

  odometryUpdate(waFL, waFR, waBL, waBR, vFL, vFR, vBL, vBR, GyroValue);

  // ---------- Field-Centric Transformation (ChimiSwerve Section 2.1b, Pages 10-12) ----------
  //   FWD_new = FWD * cos(theta) + STR * sin(theta)
  //   STR_new = STR * cos(theta) - FWD * sin(theta)
  double FWD = y;
  double STR = x;
  if (FieldCentric) {
    const double theta = GyroValue * kPi / 180.0;
    const double fwdNew = FWD * std::cos(theta) + STR * std::sin(theta);
    const double strNew = STR * std::cos(theta) - FWD * std::sin(theta);
    FWD = fwdNew;
    STR = strNew;
  }

  // ---------- Rotation / Straight-Drive Heading Lock (ChimiSwerve Section 2.3d, Pages 34-35) ----------
  double ROT = 0.0;
  if (isRotating) {
    ROT = x2;
    storedYaw = GyroValue;
    m_headingIError = 0.0;
    m_headingLastError = 0.0;
  } else if (DC::kEnableHeadingLock && isTranslating) {
    const double errorDeg = std::remainder(storedYaw - GyroValue, 360.0);
    m_headingIError += errorDeg * dt;
    const double dError = (errorDeg - m_headingLastError) / dt;
    m_headingLastError = errorDeg;

    const double yawCorrection = calcYawStraight(storedYaw, GyroValue, DC::straightP) +
                                 DC::straightI * m_headingIError +
                                 DC::straightD * dError;
    ROT = std::clamp(yawCorrection, -DC::rotationClamp, DC::rotationClamp);
  }

  // ---------- Inverse Kinematics + Inversion Awareness ----------
  ModuleStates s = InverseKinematics(FWD, STR, ROT);
  lastTargetTurnsFL = ChimiOptimizeAzimuth(s.waFL, turnsFL, s.wsFL);
  lastTargetTurnsFR = ChimiOptimizeAzimuth(s.waFR, turnsFR, s.wsFR);
  lastTargetTurnsBL = ChimiOptimizeAzimuth(s.waBL, turnsBL, s.wsBL);
  lastTargetTurnsBR = ChimiOptimizeAzimuth(s.waBR, turnsBR, s.wsBR);
  HoldSteer();

  // ---------- Speed Scaling (PS5 L2 / R2 Triggers) ----------
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
// ChimiSwerve Section 3.1 (Page 35): Pre-Match Module Testing in Test Mode
// ============================================================================
void Drivetrain::TestZeroAzimuth() {
  ReadSteerTurns();
  SetDrive(0.0, 0.0, 0.0, 0.0);

  // Command all wheels to 0 deg (straight forward) along shortest path
  double dummySpeed = 0.0;
  lastTargetTurnsFL = ChimiOptimizeAzimuth(0.0, turnsFL, dummySpeed);
  lastTargetTurnsFR = ChimiOptimizeAzimuth(0.0, turnsFR, dummySpeed);
  lastTargetTurnsBL = ChimiOptimizeAzimuth(0.0, turnsBL, dummySpeed);
  lastTargetTurnsBR = ChimiOptimizeAzimuth(0.0, turnsBR, dummySpeed);
  HoldSteer();
  PublishTelemetry();
}

void Drivetrain::TestRotateModules(double dt) {
  ReadSteerTurns();
  SetDrive(0.0, 0.0, 0.0, 0.0);

  // Slowly rotate all wheels at 0.25 turns/sec (90 deg/sec) to inspect smooth motion
  m_testRotateTurns += 0.25 * dt;
  lastTargetTurnsFL = m_testRotateTurns;
  lastTargetTurnsFR = m_testRotateTurns;
  lastTargetTurnsBL = m_testRotateTurns;
  lastTargetTurnsBR = m_testRotateTurns;
  HoldSteer();
  PublishTelemetry();
}

// ============================================================================
// Hardware Helpers
// ============================================================================
void Drivetrain::SyncFLSteerToCANcoder() {
  if (CANcoderFL.IsConnected()) {
    const double ccTurns = CANcoderFL.GetPosition().GetValueAsDouble();
    const double motorTurns = m_FL_Steer.GetPosition().GetValueAsDouble();
    if (std::abs(std::remainder(ccTurns - motorTurns, 1.0)) > DC::kSteerDeadbandTurns) {
      m_FL_Steer.SetPosition(units::turn_t{ccTurns});
    }
  }
}

void Drivetrain::ReadSteerTurns() {
  // Read live wheel position (in turns):
  //   FR, BL, BR read fused RemoteCANcoder position from the steer TalonFX on "rio"
  //   FL reads CANcoder 32 on "intakebus" (or seeded m_FL_Steer position)
  turnsFL = CANcoderFL.IsConnected()
                ? CANcoderFL.GetPosition().GetValueAsDouble()
                : m_FL_Steer.GetPosition().GetValueAsDouble();
  turnsFR = m_FR_Steer.GetPosition().GetValueAsDouble();
  turnsBL = m_BL_Steer.GetPosition().GetValueAsDouble();
  turnsBR = m_BR_Steer.GetPosition().GetValueAsDouble();
}

void Drivetrain::StopSteer() {
  m_FL_Steer.Set(0.0);
  m_FR_Steer.Set(0.0);
  m_BL_Steer.Set(0.0);
  m_BR_Steer.Set(0.0);
}

void Drivetrain::HoldSteer() {
  steerRequest.Slot = 0;

  auto commandModuleSteer = [&](hardware::TalonFX& motor, double targetTurns, double actualTurns) {
    const double err = std::remainder(targetTurns - actualTurns, 1.0);
    // Static CANcoder noise buffer: if within kSteerDeadbandTurns (~0.5% turn), hold brake at 0.0
    if (std::abs(err) < DC::kSteerDeadbandTurns) {
      motor.Set(0.0);
    } else {
      const double wrappedTarget = std::remainder(targetTurns, 1.0);
      motor.SetControl(steerRequest.WithPosition(units::turn_t{wrappedTarget}));
    }
  };

  commandModuleSteer(m_FL_Steer, lastTargetTurnsFL, turnsFL);
  commandModuleSteer(m_FR_Steer, lastTargetTurnsFR, turnsFR);
  commandModuleSteer(m_BL_Steer, lastTargetTurnsBL, turnsBL);
  commandModuleSteer(m_BR_Steer, lastTargetTurnsBR, turnsBR);
}

void Drivetrain::SetDrive(double fl, double fr, double bl, double br) {
  const double lim = DC::DriveMotorsHardLimit;
  cmdFL = std::clamp(fl, -lim, lim);
  cmdFR = std::clamp(fr, -lim, lim);
  cmdBL = std::clamp(bl, -lim, lim);
  cmdBR = std::clamp(br, -lim, lim);
  m_FL_Drive.Set(cmdFL);
  m_FR_Drive.Set(cmdFR);
  m_BL_Drive.Set(cmdBL);
  m_BR_Drive.Set(cmdBR);
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
// Odometry - ChimiSwerve Forward Kinematics (Section 2.2b, Pages 17-19)
// ============================================================================
void Drivetrain::odometryUpdate(double waFL, double waFR, double waBL, double waBR,
                                double wsFL, double wsFR, double wsBL, double wsBR,
                                double GyroValue) {
  const double now = frc::Timer::GetFPGATimestamp().value();
  const double dt = now - odoLastTime;
  odoLastTime = now;
  if (dt <= 0.0 || dt > 0.5) return;

  // ChimiSwerve Page 18: Decompose each wheel's velocity into strafe (sin) and forward (cos)
  const double B_FL = std::sin(waFL) * wsFL;
  const double D_FL = std::cos(waFL) * wsFL;

  const double B_FR = std::sin(waFR) * wsFR;
  const double C_FR = std::cos(waFR) * wsFR;

  const double A_BL = std::sin(waBL) * wsBL;
  const double D_BL = std::cos(waBL) * wsBL;

  const double A_BR = std::sin(waBR) * wsBR;
  const double C_BR = std::cos(waBR) * wsBR;

  // ChimiSwerve Page 18: Average each pair of components
  const double A = (A_BR + A_BL) / 2.0;
  const double B = (B_FL + B_FR) / 2.0;
  const double C = (C_FR + C_BR) / 2.0;
  const double D = (D_FL + D_BL) / 2.0;

  // ChimiSwerve Page 19: Rotational velocity estimate (rad/s)
  const double ROT1 = (B - A) / DC::L;
  const double ROT2 = (C - D) / DC::W;
  odoROT = (ROT1 + ROT2) / 2.0;

  // ChimiSwerve Page 19: Incorporate geometry to obtain robot-relative forward & strafe speeds
  const double STR1 = odoROT * (DC::L / 2.0) + A;
  const double STR2 = -odoROT * (DC::L / 2.0) + B;
  odoSTR = (STR1 + STR2) / 2.0;

  const double FWD1 = odoROT * (DC::W / 2.0) + C;
  const double FWD2 = -odoROT * (DC::W / 2.0) + D;
  odoFWD = (FWD1 + FWD2) / 2.0;

  // Transform robot-relative velocities into field-relative coordinates
  const double g = GyroValue * kPi / 180.0;
  const double fieldFWD = odoFWD * std::cos(g) - odoSTR * std::sin(g);
  const double fieldSTR = odoFWD * std::sin(g) + odoSTR * std::cos(g);

  // ChimiSwerve Page 19: Integrate over timestep
  positionFWDField += fieldFWD * dt;
  positionSTRField += fieldSTR * dt;

  frc::SmartDashboard::PutNumber("Odometry/FieldFWD", positionFWDField);
  frc::SmartDashboard::PutNumber("Odometry/FieldSTR", positionSTRField);
  frc::SmartDashboard::PutNumber("Odometry/RobotFWD_mps", odoFWD);
  frc::SmartDashboard::PutNumber("Odometry/RobotSTR_mps", odoSTR);
  frc::SmartDashboard::PutNumber("PositionForwardField", positionFWDField);
  frc::SmartDashboard::PutNumber("PositionStrafeField", positionSTRField);
}

// ============================================================================
// Simulation: wheels follow setpoints, drive speed follows duty cycle
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

  m_FL_Steer.GetSimState().SetRawRotorPosition(units::turn_t{lastTargetTurnsFL * DC::rotorToSensorRatio});
  m_FR_Steer.GetSimState().SetRawRotorPosition(units::turn_t{lastTargetTurnsFR * DC::rotorToSensorRatio});
  m_BL_Steer.GetSimState().SetRawRotorPosition(units::turn_t{lastTargetTurnsBL * DC::rotorToSensorRatio});
  m_BR_Steer.GetSimState().SetRawRotorPosition(units::turn_t{lastTargetTurnsBR * DC::rotorToSensorRatio});

  CANcoderFL.GetSimState().SetRawPosition(units::turn_t{lastTargetTurnsFL});
  CANcoderFR.GetSimState().SetRawPosition(units::turn_t{lastTargetTurnsFR});
  CANcoderBL.GetSimState().SetRawPosition(units::turn_t{lastTargetTurnsBL});
  CANcoderBR.GetSimState().SetRawPosition(units::turn_t{lastTargetTurnsBR});
}
