// ============================================================================
// FRC Team 8753 - 2026 Swerve Drivetrain Implementation
// ============================================================================

#define _USE_MATH_DEFINES
#include <cmath>

#include <algorithm>
#include <string>
#include <vector>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/time.h>

#include "Drivetrain.h"
#include "DrivetrainConstants.h"

using namespace ctre::phoenix6;
using namespace units::literals;
namespace DC = DrivetrainConstants;

namespace {
constexpr double kPi = 3.14159265358979323846;

double WheelMps(double motorRps) {
  return (motorRps / DC::DriveGearRatio) * (kPi * DC::WheelDiameter);
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
      CANcoderFL{DC::kFLEncoderId, CANBus{DC::kIntakeCanBus}},
      CANcoderFR{DC::kFREncoderId, CANBus{DC::kCanBus}},
      CANcoderBL{DC::kBLEncoderId, CANBus{DC::kCanBus}},
      CANcoderBR{DC::kBREncoderId, CANBus{DC::kCanBus}} {}

// ============================================================================
// ConfigureMotors (from known-working CLEO2026 / 5d15aa2 codebase)
// ============================================================================
void Drivetrain::ConfigureMotors() {
  bool ok = true;

  auto applyWithRetry = [](auto& device, const auto& cfg, const char* name) -> bool {
    for (int attempt = 1; attempt <= 4; ++attempt) {
      auto status = device.GetConfigurator().Apply(cfg, 50_ms);
      if (status.IsOK()) {
        frc::SmartDashboard::PutString(std::string("Swerve/Config_") + name, "OK");
        return true;
      }
      if (attempt == 4) {
        frc::SmartDashboard::PutString(std::string("Swerve/Config_") + name, status.GetName());
      }
    }
    return false;
  };

  // ---------- Drive Krakens: Brake mode, voltage ramp rate ----------
  configs::TalonFXConfiguration driveConfig{};
  driveConfig.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
  driveConfig.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;
  driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = units::second_t{DC::DriveRampRateSeconds};
  driveConfig.CurrentLimits.StatorCurrentLimit = units::ampere_t{DC::kDriveCurrentLimit};
  driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;

  ok &= applyWithRetry(m_FL_Drive, driveConfig, "FL_Drive");
  ok &= applyWithRetry(m_FR_Drive, driveConfig, "FR_Drive");
  ok &= applyWithRetry(m_BL_Drive, driveConfig, "BL_Drive");
  ok &= applyWithRetry(m_BR_Drive, driveConfig, "BR_Drive");

  // ---------- CANcoders: apply calibrated magnet offsets ----------
  if (DC::kWriteMagnetOffsets) {
    auto applyEncoder = [&](hardware::CANcoder& enc, double offset, const char* name) {
      configs::CANcoderConfiguration cfg{};
      cfg.MagnetSensor.MagnetOffset = units::turn_t{offset};
      cfg.MagnetSensor.SensorDirection = signals::SensorDirectionValue::CounterClockwise_Positive;
      cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = units::turn_t{0.5};
      ok &= applyWithRetry(enc, cfg, name);
    };
    applyEncoder(CANcoderBR, DC::kBRMagnetOffset, "BR_CANcoder12");
    applyEncoder(CANcoderFR, DC::kFRMagnetOffset, "FR_CANcoder22");
    applyEncoder(CANcoderFL, DC::kFLMagnetOffset, "FL_CANcoder32");
    applyEncoder(CANcoderBL, DC::kBLMagnetOffset, "BL_CANcoder42");
  }

  // ---------- Steer Krakens: Coast mode, ContinuousWrap, RemoteCANcoder fusion ----------
  auto applySteer = [&](hardware::TalonFX& motor, int canCoderId, bool sameBus, const char* name) {
    configs::TalonFXConfiguration cfg{};
    cfg.MotorOutput.NeutralMode = signals::NeutralModeValue::Coast;
    cfg.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;
    cfg.ClosedLoopGeneral.ContinuousWrap = true;
    cfg.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = units::second_t{DC::SteeringRampRateSeconds};

    if (sameBus) {
      cfg.Feedback.FeedbackRemoteSensorID = canCoderId;
      cfg.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::RemoteCANcoder;
      cfg.Feedback.SensorToMechanismRatio = DC::sensorToMechanismRatio;
      cfg.Feedback.RotorToSensorRatio = DC::rotorToSensorRatio;
    } else {
      // FL Steer (ID 31 on "rio") tracks CANcoder 32 (on "intakebus") via 18:1 RotorSensor
      cfg.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::RotorSensor;
      cfg.Feedback.SensorToMechanismRatio = DC::rotorToSensorRatio;
    }

    cfg.Slot0.kP = DC::ModuleP;
    cfg.Slot0.kI = DC::ModuleI;
    cfg.Slot0.kD = DC::ModuleD;

    ok &= applyWithRetry(motor, cfg, name);
  };

  applySteer(m_FL_Steer, DC::kFLEncoderId, false, "FL_Steer");
  applySteer(m_FR_Steer, DC::kFREncoderId, true,  "FR_Steer");
  applySteer(m_BL_Steer, DC::kBLEncoderId, true,  "BL_Steer");
  applySteer(m_BR_Steer, DC::kBREncoderId, true,  "BR_Steer");

  SyncFLSteerToCANcoder();

  hasBootstrapped = false;
  m_configsApplied = ok;
  frc::SmartDashboard::PutBoolean("Swerve/ConfigsApplied", ok);
}

// ============================================================================
// Disabled Lifecycle
// ============================================================================
void Drivetrain::DisabledInit() {
  m_FL_Drive.Set(0.0);
  m_FR_Drive.Set(0.0);
  m_BL_Drive.Set(0.0);
  m_BR_Drive.Set(0.0);
}

void Drivetrain::DisabledPeriodic() {
  if (!m_configsApplied) {
    static int retryLoops = 0;
    if (++retryLoops <= 150 && (retryLoops % 50 == 0)) {
      ConfigureMotors();
    }
  }

  SyncFLSteerToCANcoder();
  PublishTelemetry();
}

void Drivetrain::SyncFLSteerToCANcoder() {
  if (CANcoderFL.IsConnected()) {
    const double ccTurns = CANcoderFL.GetPosition().GetValueAsDouble();
    const double motorTurns = m_FL_Steer.GetPosition().GetValueAsDouble();
    if (std::abs(std::remainder(ccTurns - motorTurns, 1.0)) > DC::kSteerDeadbandTurns) {
      m_FL_Steer.SetPosition(units::turn_t{ccTurns});
    }
  }
}

// ============================================================================
// Math Helpers (MinimizeRotation from 5d15aa2 / swerve-base)
// ============================================================================
double Drivetrain::MinimizeRotation(double targetAngleRad, double currentAngleRad, double& speedPercent) {
  double errorRad = std::remainder(targetAngleRad - currentAngleRad, 2.0 * M_PI);

  if (std::abs(errorRad) > M_PI / 2.0) {
    errorRad -= std::copysign(M_PI, errorRad);
    speedPercent = -speedPercent;
  }

  return std::remainder(currentAngleRad + errorRad, 2.0 * M_PI);
}

double Drivetrain::MinimizeRotation(double targetAngle, double currentAngle) {
  double error = targetAngle - currentAngle;
  while (error > M_PI) error -= 2.0 * M_PI;
  while (error < -M_PI) error += 2.0 * M_PI;

  if (std::abs(error) > M_PI / 2.0) {
    targetAngle -= M_PI;
  }
  return targetAngle;
}

double Drivetrain::WrapError(double targetAngle, double currentAngle) {
  return std::remainder(targetAngle - currentAngle, 2.0 * M_PI);
}

double Drivetrain::ChimiOptimizeAzimuth(double waRad, double currentTurns, double& ws) {
  const double targetTurns = waRad / (2.0 * kPi);
  double errorTurns = std::remainder(targetTurns - currentTurns, 1.0);
  if (std::abs(errorTurns) > 0.25) {
    errorTurns -= std::copysign(0.5, errorTurns);
    ws = -ws;
  }
  if (std::abs(errorTurns) < DC::kSteerDeadbandTurns) {
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
    s.wsFL /= maxSpeed;
    s.wsFR /= maxSpeed;
    s.wsBL /= maxSpeed;
    s.wsBR /= maxSpeed;
  }
  return s;
}

double Drivetrain::calcYawStraight(double targetAngle, double currentAngle, double kP) {
  const double errorAngle = std::remainder(targetAngle - currentAngle, 360.0);
  return errorAngle * kP;
}

// ============================================================================
// Main Swerve Update Loop (Exact from 5d15aa2 / swerve-base)
// ============================================================================
void Drivetrain::Update(double x, double y, double x2, double GyroValue,
                        double triggerL, double triggerR, bool FieldCentric) {
  // Read live wheel angles in radians (0 to 2pi) directly from absolute encoders
  double FL_pos = std::fmod(CANcoderFL.GetPosition().GetValueAsDouble(), 1.0) * (2.0 * M_PI);
  double FR_pos = std::fmod(CANcoderFR.GetPosition().GetValueAsDouble(), 1.0) * (2.0 * M_PI);
  double BL_pos = std::fmod(CANcoderBL.GetPosition().GetValueAsDouble(), 1.0) * (2.0 * M_PI);
  double BR_pos = std::fmod(CANcoderBR.GetPosition().GetValueAsDouble(), 1.0) * (2.0 * M_PI);

  units::time::second_t currentTime = frc::Timer::GetFPGATimestamp();
  lastTime = currentTime;

  double STR = x;
  double FWD = y;

  // Field-Centric transformation
  if (FieldCentric) {
    double temp = FWD * std::cos(GyroValue * M_PI / 180.0) + STR * std::sin(GyroValue * M_PI / 180.0);
    STR = -FWD * std::sin(GyroValue * M_PI / 180.0) + STR * std::cos(GyroValue * M_PI / 180.0);
    FWD = temp;
  }

  double ROT = x2;

  // Swerve Drive Kinematics (L = wheelbase, W = trackwidth)
  double A = (STR - ROT * DC::L / 2.0);
  double B = (STR + ROT * DC::L / 2.0);
  double C = (FWD - ROT * DC::W / 2.0);
  double D = (FWD + ROT * DC::W / 2.0);

  // Wheel speeds
  if (x != 0.0 || y != 0.0 || x2 != 0.0) {
    speedFL = std::hypot(B, D);
    speedFR = std::hypot(B, C);
    speedBL = std::hypot(A, D);
    speedBR = std::hypot(A, C);
  } else {
    speedFL = 0.0;
    speedFR = 0.0;
    speedBL = 0.0;
    speedBR = 0.0;
  }

  // Normalize wheel speeds
  std::vector<double> wheelSpeeds = {speedFL, speedFR, speedBL, speedBR};
  double maxSpeed = *std::max_element(wheelSpeeds.begin(), wheelSpeeds.end());
  if (maxSpeed >= 1.0) {
    speedFL /= maxSpeed;
    speedFR /= maxSpeed;
    speedBL /= maxSpeed;
    speedBR /= maxSpeed;
  }

  // Calculate target angle for each wheel
  if (x != 0.0 || y != 0.0 || x2 != 0.0) {
    tempangleFL = std::atan2(B, D);
    tempangleFR = std::atan2(B, C);
    tempangleBL = std::atan2(A, D);
    tempangleBR = std::atan2(A, C);
  }

  // Minimize rotation: finds shortest angular distance (≤ 90 deg) and inverts drive if needed
  angleFL = MinimizeRotation(tempangleFL, FL_pos, speedFL);
  angleFR = MinimizeRotation(tempangleFR, FR_pos, speedFR);
  angleBL = MinimizeRotation(tempangleBL, BL_pos, speedBL);
  angleBR = MinimizeRotation(tempangleBR, BR_pos, speedBR);

  // Steering commands: angle in radians -> mechanism turns: (angle / M_PI) / 2.0
  ctre::phoenix6::controls::PositionDutyCycle steerCmd{0_tr};
  steerCmd.Slot = 0;

  m_FL_Steer.SetControl(steerCmd.WithPosition(units::angle::turn_t((angleFL / M_PI) / 2.0)));
  m_FR_Steer.SetControl(steerCmd.WithPosition(units::angle::turn_t((angleFR / M_PI) / 2.0)));
  m_BL_Steer.SetControl(steerCmd.WithPosition(units::angle::turn_t((angleBL / M_PI) / 2.0)));
  m_BR_Steer.SetControl(steerCmd.WithPosition(units::angle::turn_t((angleBR / M_PI) / 2.0)));

  // Calculate speed scaling based on PS5 triggers (L2 slows down, R2 speeds up)
  double speedConst = 100.0 / (DC::DefaultDriveSpeed +
                              (-triggerL * DC::TriggerConstant) +
                              (triggerR * DC::TriggerConstant));

  // Drive motor commands:
  //   In 5d15aa2 / CLEO chassis: FL, FR, BL are positive, BR is negated (-speedBR)
  if (std::isnan(speedFL) || (x == 0.0 && y == 0.0 && x2 == 0.0)) {
    cmdFL = 0.0; cmdFR = 0.0; cmdBL = 0.0; cmdBR = 0.0;
    m_FL_Drive.Set(0.0);
    m_FR_Drive.Set(0.0);
    m_BL_Drive.Set(0.0);
    m_BR_Drive.Set(0.0);
  } else {
    cmdFL = std::clamp(speedFL / speedConst, -DC::DriveMotorsHardLimit, DC::DriveMotorsHardLimit);
    cmdFR = std::clamp(speedFR / speedConst, -DC::DriveMotorsHardLimit, DC::DriveMotorsHardLimit);
    cmdBL = std::clamp(speedBL / speedConst, -DC::DriveMotorsHardLimit, DC::DriveMotorsHardLimit);
    cmdBR = std::clamp(-speedBR / speedConst, -DC::DriveMotorsHardLimit, DC::DriveMotorsHardLimit);

    m_FL_Drive.Set(cmdFL);
    m_FR_Drive.Set(cmdFR);
    m_BL_Drive.Set(cmdBL);
    m_BR_Drive.Set(cmdBR);
  }

  // Odometry update
  const double vFL = WheelMps(m_FL_Drive.GetVelocity().GetValueAsDouble());
  const double vFR = WheelMps(m_FR_Drive.GetVelocity().GetValueAsDouble());
  const double vBL = WheelMps(m_BL_Drive.GetVelocity().GetValueAsDouble());
  const double vBR = -WheelMps(m_BR_Drive.GetVelocity().GetValueAsDouble());
  odometryUpdate(angleFL, angleFR, angleBL, angleBR, vFL, vFR, vBL, vBR, GyroValue);

  PublishTelemetry();
}

// ============================================================================
// Odometry Update (from 5d15aa2)
// ============================================================================
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
  double FWD1 = odoROT * (DC::L / 2.0) + A;
  double FWD2 = -odoROT * (DC::L / 2.0) + B;
  odoSTR = (FWD1 + FWD2) / 2.0;

  double STR1 = odoROT * (DC::W / 2.0) + C;
  double STR2 = -odoROT * (DC::W / 2.0) + D;
  odoFWD = (STR1 + STR2) / 2.0;

  double gyroRad = GyroValue * M_PI / 180.0;
  double fieldFWD = odoFWD * std::cos(gyroRad) - odoSTR * std::sin(gyroRad);
  double fieldSTR = odoFWD * std::sin(gyroRad) + odoSTR * std::cos(gyroRad);

  positionFWDField += units::length::meter_t(fieldFWD * odoDeltaTime);
  positionSTRField += units::length::meter_t(fieldSTR * odoDeltaTime);
  ROTField = frc::Rotation2d(units::angle::radian_t(odoROT));

  frc::SmartDashboard::PutNumber("PositionForwardField", positionFWDField.value());
  frc::SmartDashboard::PutNumber("PositionStrafeField", positionSTRField.value());
}

// ============================================================================
// Test Mode Helpers
// ============================================================================
void Drivetrain::TestZeroAzimuth() {
  m_FL_Drive.Set(0.0);
  m_FR_Drive.Set(0.0);
  m_BL_Drive.Set(0.0);
  m_BR_Drive.Set(0.0);

  ctre::phoenix6::controls::PositionDutyCycle steerCmd{0_tr};
  steerCmd.Slot = 0;
  m_FL_Steer.SetControl(steerCmd.WithPosition(0_tr));
  m_FR_Steer.SetControl(steerCmd.WithPosition(0_tr));
  m_BL_Steer.SetControl(steerCmd.WithPosition(0_tr));
  m_BR_Steer.SetControl(steerCmd.WithPosition(0_tr));

  PublishTelemetry();
}

void Drivetrain::TestRotateModules(double dt) {
  m_FL_Drive.Set(0.0);
  m_FR_Drive.Set(0.0);
  m_BL_Drive.Set(0.0);
  m_BR_Drive.Set(0.0);

  m_testRotateTurns += 0.25 * dt;
  ctre::phoenix6::controls::PositionDutyCycle steerCmd{0_tr};
  steerCmd.Slot = 0;
  m_FL_Steer.SetControl(steerCmd.WithPosition(units::turn_t{m_testRotateTurns}));
  m_FR_Steer.SetControl(steerCmd.WithPosition(units::turn_t{m_testRotateTurns}));
  m_BL_Steer.SetControl(steerCmd.WithPosition(units::turn_t{m_testRotateTurns}));
  m_BR_Steer.SetControl(steerCmd.WithPosition(units::turn_t{m_testRotateTurns}));

  PublishTelemetry();
}

// ============================================================================
// Telemetry & Simulation
// ============================================================================
void Drivetrain::PublishTelemetry() {
  frc::SmartDashboard::PutNumber("Swerve/FL_ActualTurns", CANcoderFL.GetPosition().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Swerve/FR_ActualTurns", CANcoderFR.GetPosition().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Swerve/BL_ActualTurns", CANcoderBL.GetPosition().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Swerve/BR_ActualTurns", CANcoderBR.GetPosition().GetValueAsDouble());

  frc::SmartDashboard::PutNumber("Swerve/FL_DriveCmd", cmdFL);
  frc::SmartDashboard::PutNumber("Swerve/FR_DriveCmd", cmdFR);
  frc::SmartDashboard::PutNumber("Swerve/BL_DriveCmd", cmdBL);
  frc::SmartDashboard::PutNumber("Swerve/BR_DriveCmd", cmdBR);

  frc::SmartDashboard::PutBoolean("Swerve/FL_EncoderConnected", CANcoderFL.IsConnected());
  frc::SmartDashboard::PutBoolean("Swerve/FR_EncoderConnected", CANcoderFR.IsConnected());
  frc::SmartDashboard::PutBoolean("Swerve/BL_EncoderConnected", CANcoderBL.IsConnected());
  frc::SmartDashboard::PutBoolean("Swerve/BR_EncoderConnected", CANcoderBR.IsConnected());
}

void Drivetrain::UpdateSim(units::time::second_t dt) {
  constexpr double kMaxRps = 100.0;
  auto simDrive = [&](hardware::TalonFX& motor, double cmd) {
    auto& sim = motor.GetSimState();
    const double rps = cmd * kMaxRps;
    sim.SetRotorVelocity(units::turns_per_second_t{rps});
    sim.AddRotorPosition(units::turn_t{rps * dt.value()});
  };
  simDrive(m_FL_Drive, cmdFL);
  simDrive(m_FR_Drive, cmdFR);
  simDrive(m_BL_Drive, cmdBL);
  simDrive(m_BR_Drive, cmdBR);

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
