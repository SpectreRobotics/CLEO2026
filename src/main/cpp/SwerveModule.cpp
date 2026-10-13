// ============================================================================
// FRC Team 8753 - Swerve Module implementation (from scratch)
// ============================================================================

#include "SwerveModule.h"

#include <algorithm>
#include <cmath>
#include <string>

#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>
#include <units/time.h>

using namespace ctre::phoenix6;
namespace DC = DrivetrainConstants;

namespace {
constexpr double kPi = 3.14159265358979323846;

/// Wrap any angle in turns to [-0.5, 0.5).
double WrapTurns(double t) {
  return t - std::floor(t + 0.5);
}

std::string Key(const ModuleConstants& c, const char* field) {
  return std::string("Swerve/") + c.name + "/" + field;
}
}  // namespace

SwerveModule::SwerveModule(const ModuleConstants& c)
    : m_c(c),
      m_drive(c.driveId, CANBus{DC::kCanBusName}),
      m_steer(c.steerId, CANBus{DC::kCanBusName}),
      m_encoder(c.encoderId, CANBus{DC::kCanBusName}) {}

bool SwerveModule::Configure() {
  bool ok = true;

  // ----- Drive motor: brake, inversion, current limit, open-loop ramp -----
  configs::TalonFXConfiguration drive{};
  drive.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
  drive.MotorOutput.Inverted = m_c.driveInverted
                                   ? signals::InvertedValue::Clockwise_Positive
                                   : signals::InvertedValue::CounterClockwise_Positive;
  drive.CurrentLimits.StatorCurrentLimitEnable = true;
  drive.CurrentLimits.StatorCurrentLimit = units::ampere_t{DC::kDriveCurrentLimitAmps};
  drive.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = units::second_t{DC::kDriveRampSec};

  // ----- Steer motor: plain duty-cycle output, NO onboard closed loop -----
  configs::TalonFXConfiguration steer{};
  steer.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
  steer.MotorOutput.Inverted = m_c.steerInverted
                                   ? signals::InvertedValue::Clockwise_Positive
                                   : signals::InvertedValue::CounterClockwise_Positive;
  steer.CurrentLimits.StatorCurrentLimitEnable = true;
  steer.CurrentLimits.StatorCurrentLimit = units::ampere_t{DC::kSteerCurrentLimitAmps};

  for (int i = 0; i < 5; ++i) { if (m_drive.GetConfigurator().Apply(drive, 100_ms).IsOK()) break; if (i == 4) ok = false; }
  for (int i = 0; i < 5; ++i) { if (m_steer.GetConfigurator().Apply(steer, 100_ms).IsOK()) break; if (i == 4) ok = false; }

  // The CANcoder config is intentionally NOT touched: offsets are applied in
  // software (encoderOffsetTurns) so there is no hidden on-device state.
  // Faster position/velocity updates for the roboRIO steering loop.
  m_encoder.GetAbsolutePosition().SetUpdateFrequency(100_Hz);
  m_encoder.GetVelocity().SetUpdateFrequency(100_Hz);

  frc::SmartDashboard::PutBoolean(Key(m_c, "ConfigOK"), ok);
  return ok;
}

double SwerveModule::GetRawEncoderTurns() {
  return m_encoder.GetAbsolutePosition().GetValueAsDouble();
}

double SwerveModule::GetAngleTurns() {
  return WrapTurns(GetRawEncoderTurns() - m_c.encoderOffsetTurns);
}

double SwerveModule::GetWheelSpeedMps() {
  double motorRps = m_drive.GetVelocity().GetValueAsDouble();  // already respects inversion
  return motorRps / DC::kDriveGearRatio * (kPi * DC::kWheelDiameterMeters);
}

double SwerveModule::ComputeSteerOutput(double targetTurns, double dt) {
  double error = WrapTurns(targetTurns - GetAngleTurns());  // shortest way, [-0.5, 0.5)
  double dErr = (dt > 0.0) ? (error - m_lastError) / dt : 0.0;
  m_lastError = error;

  if (std::abs(error) < DC::kSteerToleranceTurns) {
    return 0.0;
  }
  double out = DC::kSteerP * error + DC::kSteerD * dErr;
  out += std::copysign(DC::kSteerStatic, error);
  return std::clamp(out, -DC::kSteerMaxOutput, DC::kSteerMaxOutput);
}

void SwerveModule::SetDesired(double speed, double angleTurns, double dt) {
  double current = GetAngleTurns();

  // Optimize: never rotate a wheel more than 90 deg; reverse the drive instead.
  double delta = WrapTurns(angleTurns - current);
  if (std::abs(delta) > 0.25) {
    angleTurns = WrapTurns(angleTurns + 0.5);
    speed = -speed;
    delta = WrapTurns(angleTurns - current);
  }

  // Cosine scaling: don't push the wheel hard while it's still pointing wrong.
  speed *= std::max(0.0, std::cos(delta * 2.0 * kPi));

  m_targetTurns = angleTurns;
  m_hasTarget = true;

  ApplySteer(ComputeSteerOutput(m_targetTurns, dt));
  ApplyDrive(std::clamp(speed, -1.0, 1.0) * DC::kDriveMaxOutput);
}

void SwerveModule::Hold(double dt) {
  if (!m_hasTarget) {
    // First enable: hold wherever the wheel is right now (no snapping).
    m_targetTurns = GetAngleTurns();
    m_hasTarget = true;
    m_lastError = 0.0;
  }
  ApplySteer(ComputeSteerOutput(m_targetTurns, dt));
  ApplyDrive(0.0);
}

void SwerveModule::Stop() {
  m_hasTarget = false;
  m_lastError = 0.0;
  m_runawayTimer = 0.0;
  ApplySteer(0.0);
  ApplyDrive(0.0);
}

void SwerveModule::RunPhaseTest() {
  ApplyDrive(0.0);
  ApplySteer(DC::kSteerTestDuty);
  double vel = m_encoder.GetVelocity().GetValueAsDouble();  // turns/s, CCW+
  if (std::abs(vel) > 0.05) {
    m_phaseOk = vel > 0.0;  // +duty must make the encoder increase
  }
  frc::SmartDashboard::PutBoolean(Key(m_c, "SteerPhaseOK"), m_phaseOk);
  frc::SmartDashboard::PutNumber(Key(m_c, "TestEncoderVel"), vel);
}

void SwerveModule::ApplySteer(double duty) {
  // ---- Runaway protection ----
  // If we are pushing hard one way and the encoder moves the other way, the
  // motor/encoder directions disagree. Kill steer instead of spinning forever.
  double vel = m_encoder.GetVelocity().GetValueAsDouble();
  bool pushingHard = std::abs(duty) > 0.8 * DC::kSteerMaxOutput;
  bool movingOpposite = (vel * duty < 0.0) && std::abs(vel) > DC::kRunawayMinVelTurnsPerSec;
  if (pushingHard && movingOpposite) {
    m_runawayTimer += 0.02;
  } else {
    m_runawayTimer = 0.0;
  }
  if (m_runawayTimer > DC::kRunawayTimeSec) {
    m_runawayTripped = true;
  }
  if (m_runawayTripped) {
    duty = 0.0;
  }

  m_steerDuty = duty;
  m_steer.SetControl(m_steerReq.WithOutput(duty));
}

void SwerveModule::ApplyDrive(double duty) {
  if (m_runawayTripped) duty = 0.0;
  m_driveDuty = duty;
  m_drive.SetControl(m_driveReq.WithOutput(duty));
}

void SwerveModule::PublishTelemetry() {
  frc::SmartDashboard::PutNumber(Key(m_c, "RawEncoderTurns"), GetRawEncoderTurns());
  frc::SmartDashboard::PutNumber(Key(m_c, "OffsetTurns"), m_c.encoderOffsetTurns);
  frc::SmartDashboard::PutNumber(Key(m_c, "AngleDeg"), GetAngleTurns() * 360.0);
  frc::SmartDashboard::PutNumber(Key(m_c, "TargetDeg"), m_targetTurns * 360.0);
  frc::SmartDashboard::PutNumber(Key(m_c, "SteerDuty"), m_steerDuty);
  frc::SmartDashboard::PutNumber(Key(m_c, "DriveDuty"), m_driveDuty);
  frc::SmartDashboard::PutBoolean(Key(m_c, "RUNAWAY_TRIPPED"), m_runawayTripped);
  frc::SmartDashboard::PutBoolean(Key(m_c, "EncoderConnected"), m_encoder.IsConnected());
  frc::SmartDashboard::PutBoolean(Key(m_c, "SteerConnected"), m_steer.IsConnected());
  frc::SmartDashboard::PutBoolean(Key(m_c, "DriveConnected"), m_drive.IsConnected());
}

void SwerveModule::UpdateSim(double dt) {
  // Simple model: steer duty 1.0 -> 5 wheel turns/sec, phase assumed correct.
  constexpr double kSimSteerTurnsPerSecAtFull = 5.0;
  constexpr double kSimDriveRpsAtFull = 100.0;

  double steerVel = m_steerDuty * kSimSteerTurnsPerSecAtFull;
  m_simAngleTurns += steerVel * dt;
  auto& enc = m_encoder.GetSimState();
  enc.SetRawPosition(units::turn_t{WrapTurns(m_simAngleTurns + m_c.encoderOffsetTurns)});
  enc.SetVelocity(units::turns_per_second_t{steerVel});

  double driveRps = m_driveDuty * kSimDriveRpsAtFull;
  auto& drv = m_drive.GetSimState();
  drv.SetRotorVelocity(units::turns_per_second_t{m_c.driveInverted ? -driveRps : driveRps});
  drv.AddRotorPosition(units::turn_t{(m_c.driveInverted ? -driveRps : driveRps) * dt});
}
