// ============================================================================
// FRC Team 8753 - 2026 Modular Turret Subsystem Implementation
// ============================================================================

#include "Turret.h"

#include <algorithm>
#include <cmath>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace units::literals;

Turret::Turret() {
  // Motor configurations are applied in ConfigureMotors()
}

void Turret::ConfigureMotors() {
  // ========== 1. Turret Azimuth Rotation (Kraken X44) ==========
  ctre::phoenix6::configs::TalonFXConfiguration rotationConfig{};
  rotationConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

  // Slot 0 Gains for Motion Magic
  auto& slot0 = rotationConfig.Slot0;
  slot0.kP = TurretConstants::kRotationP;
  slot0.kI = TurretConstants::kRotationI;
  slot0.kD = TurretConstants::kRotationD;
  slot0.kS = TurretConstants::kRotationS;
  slot0.kV = TurretConstants::kRotationV;
  slot0.kA = TurretConstants::kRotationA;

  // Motion Magic Cruise Velocity and Acceleration limits
  auto& mm = rotationConfig.MotionMagic;
  mm.MotionMagicCruiseVelocity = units::turns_per_second_t(TurretConstants::kRotationCruiseVelocityRps);
  mm.MotionMagicAcceleration = units::turns_per_second_squared_t(TurretConstants::kRotationAccelerationRps2);
  mm.MotionMagicJerk = units::turns_per_second_cubed_t(TurretConstants::kRotationJerkRps3);

  m_rotationMotor.GetConfigurator().Apply(rotationConfig);
  m_rotationMotor.SetPosition(0_tr);

  // ========== 2. Flywheel Shooter (2x Kraken X60) ==========
  ctre::phoenix6::configs::TalonFXConfiguration shooterConfig{};
  // Coast mode so flywheels spin down naturally without abrupt stress on belts/pulleys
  shooterConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;

  auto& shooterSlot0 = shooterConfig.Slot0;
  shooterSlot0.kP = TurretConstants::kShooterP;
  shooterSlot0.kI = TurretConstants::kShooterI;
  shooterSlot0.kD = TurretConstants::kShooterD;
  shooterSlot0.kS = TurretConstants::kShooterS;
  shooterSlot0.kV = TurretConstants::kShooterV;

  m_flywheelLeft.GetConfigurator().Apply(shooterConfig);
  m_flywheelRight.GetConfigurator().Apply(shooterConfig);

  // ========== 3. Hood Pitch Servos (REV Smart Servos via ServoHub) ==========
  rev::servohub::ServoChannelConfig servo0Config{rev::servohub::ServoChannel::ChannelId::kChannelId0};
  rev::servohub::ServoChannelConfig servo1Config{rev::servohub::ServoChannel::ChannelId::kChannelId1};

  servo0Config.PulseRange(TurretConstants::kHoodMinPulseWidthUs, 1500.0, TurretConstants::kHoodMaxPulseWidthUs);
  servo1Config.PulseRange(TurretConstants::kHoodMinPulseWidthUs, 1500.0, TurretConstants::kHoodMaxPulseWidthUs);

  rev::servohub::ServoHubConfig hubConfig{};
  hubConfig.Apply(rev::servohub::ServoChannel::ChannelId::kChannelId0, servo0Config);
  hubConfig.Apply(rev::servohub::ServoChannel::ChannelId::kChannelId1, servo1Config);
  m_servoHub.Configure(hubConfig, rev::ResetMode::kNoResetSafeParameters);

  // Default hood to lower rest position
  SetHoodAngle(units::angle::degree_t(TurretConstants::kHoodMinAngleDeg));
}

// ========== Azimuth Rotation Controls ==========

void Turret::SetRotationAngle(units::angle::degree_t angle) {
  double targetDeg = std::clamp(angle.value(), TurretConstants::kTurretMinAngleDeg, TurretConstants::kTurretMaxAngleDeg);
  m_targetAngleDeg = targetDeg;

  double motorRotations = targetDeg * TurretConstants::kMotorRotationsPerDegree;
  m_rotationMotor.SetControl(m_motionMagicRequest.WithPosition(units::angle::turn_t(motorRotations)));
}

void Turret::SetRotationSpeed(double percent) {
  double currentDeg = GetRotationAngleDegrees();

  // Soft limit protection: prevent driving past mechanical ends
  if (percent > 0.0 && currentDeg >= TurretConstants::kTurretMaxAngleDeg) {
    m_rotationMotor.Set(0.0);
  } else if (percent < 0.0 && currentDeg <= TurretConstants::kTurretMinAngleDeg) {
    m_rotationMotor.Set(0.0);
  } else {
    m_rotationMotor.Set(percent);
  }
}

void Turret::ZeroRotation() {
  m_rotationMotor.SetPosition(0_tr);
  m_targetAngleDeg = 0.0;
}

double Turret::GetRotationAngleDegrees() {
  return m_rotationMotor.GetPosition().GetValueAsDouble() * TurretConstants::kDegreesPerMotorRotation;
}

// ========== Shooter Flywheel Controls ==========

void Turret::SetShooterRPM(double rpm) {
  m_targetShooterRPM = rpm;
  double rps = (rpm / 60.0) * TurretConstants::kShooterGearRatio;
  SetShooterVelocity(units::angular_velocity::turns_per_second_t(rps));
}

void Turret::SetShooterVelocity(units::angular_velocity::turns_per_second_t rps) {
  m_flywheelLeft.SetControl(m_velocityRequest.WithVelocity(rps));
  m_flywheelRight.SetControl(m_velocityRequest.WithVelocity(rps));
}

void Turret::SetShooterPercent(double percent) {
  m_flywheelLeft.Set(percent);
  m_flywheelRight.Set(percent);
}

void Turret::StopShooter() {
  m_targetShooterRPM = 0.0;
  m_flywheelLeft.Set(0.0);
  m_flywheelRight.Set(0.0);
}

double Turret::GetShooterRPM() {
  return (m_flywheelLeft.GetVelocity().GetValueAsDouble() * 60.0) / TurretConstants::kShooterGearRatio;
}

bool Turret::IsShooterAtSpeed(double targetRPM, double toleranceRPM) {
  return std::abs(GetShooterRPM() - targetRPM) <= toleranceRPM;
}

// ========== Hood Pitch Servo Controls ==========

void Turret::SetHoodAngle(units::angle::degree_t angle) {
  double clamped = std::clamp(angle.value(), TurretConstants::kHoodMinAngleDeg, TurretConstants::kHoodMaxAngleDeg);
  m_currentHoodAngleDeg = clamped;

  // Linear mapping between min/max angle and pulse width
  double normalized = (clamped - TurretConstants::kHoodMinAngleDeg) /
                      (TurretConstants::kHoodMaxAngleDeg - TurretConstants::kHoodMinAngleDeg);

  double pw = TurretConstants::kHoodMinPulseWidthUs +
              normalized * (TurretConstants::kHoodMaxPulseWidthUs - TurretConstants::kHoodMinPulseWidthUs);

  SetHoodPulseWidth(pw);
}

void Turret::SetHoodPulseWidth(double pulseWidthUs) {
  // Servo 0 standard
  m_hoodServo0.SetPulseWidth(pulseWidthUs);
  // Servo 1 mirrored/opposing for symmetrical dual-sided hood mount
  double reversedPw = TurretConstants::kHoodMaxPulseWidthUs -
                      (pulseWidthUs - TurretConstants::kHoodMinPulseWidthUs);
  m_hoodServo1.SetPulseWidth(reversedPw);
}

// ========== Telemetry ==========

void Turret::Periodic() {
  frc::SmartDashboard::PutNumber("Turret/Angle Deg", GetRotationAngleDegrees());
  frc::SmartDashboard::PutNumber("Turret/Target Angle Deg", m_targetAngleDeg);
  frc::SmartDashboard::PutNumber("Turret/Shooter RPM", GetShooterRPM());
  frc::SmartDashboard::PutNumber("Turret/Target Shooter RPM", m_targetShooterRPM);
  frc::SmartDashboard::PutNumber("Turret/Hood Angle Deg", m_currentHoodAngleDeg);
}
