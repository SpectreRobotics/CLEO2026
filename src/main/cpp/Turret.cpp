// ============================================================================
// FRC Team 8753 - 2026 Shooter Subsystem Implementation (Fixed Chassis Mount)
// ============================================================================

#include "Turret.h"

#define _USE_MATH_DEFINES
#include <algorithm>
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>

#include "LimelightHelpers.h"

using namespace units::literals;

Turret::Turret() {}

void Turret::ConfigureMotors() {
  // ========== 1. Flywheel Shooter (2x Kraken X60) ==========
  ctre::phoenix6::configs::TalonFXConfiguration shooterConfig{};
  shooterConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;

  auto& slot0 = shooterConfig.Slot0;
  slot0.kP = TurretConstants::kShooterP;
  slot0.kI = TurretConstants::kShooterI;
  slot0.kD = TurretConstants::kShooterD;
  slot0.kS = TurretConstants::kShooterS;
  slot0.kV = TurretConstants::kShooterV;

  m_flywheelLeft.GetConfigurator().Apply(shooterConfig);

  // Right flywheel spins opposite direction (counter-rotating pair)
  shooterConfig.MotorOutput.Inverted =
      ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
  m_flywheelRight.GetConfigurator().Apply(shooterConfig);

  // ========== 2. Hood Pitch Servos (REV Smart Servos via ServoHub) ==========
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

// ========== Flywheel Shooter Controls ==========

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
  SetHoodAngle(units::angle::degree_t(TurretConstants::kHoodMinAngleDeg));
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

  double normalized = (clamped - TurretConstants::kHoodMinAngleDeg) /
                      (TurretConstants::kHoodMaxAngleDeg - TurretConstants::kHoodMinAngleDeg);

  double pw = TurretConstants::kHoodMinPulseWidthUs +
              normalized * (TurretConstants::kHoodMaxPulseWidthUs - TurretConstants::kHoodMinPulseWidthUs);

  SetHoodPulseWidth(pw);
}

void Turret::SetHoodPulseWidth(double pulseWidthUs) {
  m_hoodServo0.SetPulseWidth(pulseWidthUs);
  // Mirrored channel for symmetrical dual-sided hood mount
  double reversedPw = TurretConstants::kHoodMaxPulseWidthUs -
                      (pulseWidthUs - TurretConstants::kHoodMinPulseWidthUs);
  m_hoodServo1.SetPulseWidth(reversedPw);
}

// ========== Auto-Targeting & Ballistics ==========

double Turret::CalculateTargetRPM(double distanceMeters) const {
  const auto& table = TurretConstants::kBallisticTable;
  if (distanceMeters <= table.front().distanceMeters) {
    return table.front().shooterRPM;
  }
  if (distanceMeters >= table.back().distanceMeters) {
    return table.back().shooterRPM;
  }

  for (size_t i = 0; i < table.size() - 1; ++i) {
    if (distanceMeters >= table[i].distanceMeters && distanceMeters <= table[i + 1].distanceMeters) {
      double t = (distanceMeters - table[i].distanceMeters) /
                 (table[i + 1].distanceMeters - table[i].distanceMeters);
      return table[i].shooterRPM + t * (table[i + 1].shooterRPM - table[i].shooterRPM);
    }
  }
  return TurretConstants::kShooterDefaultRPM;
}

double Turret::CalculateTargetHoodAngle(double distanceMeters) const {
  const auto& table = TurretConstants::kBallisticTable;
  if (distanceMeters <= table.front().distanceMeters) {
    return table.front().hoodAngleDeg;
  }
  if (distanceMeters >= table.back().distanceMeters) {
    return table.back().hoodAngleDeg;
  }

  for (size_t i = 0; i < table.size() - 1; ++i) {
    if (distanceMeters >= table[i].distanceMeters && distanceMeters <= table[i + 1].distanceMeters) {
      double t = (distanceMeters - table[i].distanceMeters) /
                 (table[i + 1].distanceMeters - table[i].distanceMeters);
      return table[i].hoodAngleDeg + t * (table[i + 1].hoodAngleDeg - table[i].hoodAngleDeg);
    }
  }
  return TurretConstants::kHoodMinAngleDeg;
}

void Turret::SetShooterAndHoodForDistance(double distanceMeters) {
  const auto& table = TurretConstants::kBallisticTable;
  double clampedDist = std::clamp(distanceMeters, table.front().distanceMeters, table.back().distanceMeters);
  SetShooterRPM(CalculateTargetRPM(clampedDist));
  SetHoodAngle(units::angle::degree_t(CalculateTargetHoodAngle(clampedDist)));
}

void Turret::UpdateAutoTarget(units::length::meter_t robotX,
                              units::length::meter_t robotY,
                              units::angle::degree_t robotHeading) {
  bool visionFound = false;

  // 1. Try Limelight first
  if (LimelightHelpers::getTV("")) {
    visionFound = true;
    m_hasVisionTarget = true;
    m_lastVisionTargetTime = frc::Timer::GetFPGATimestamp();
    m_visionYawDeg = LimelightHelpers::getTX("");
    m_visionPitchDeg = LimelightHelpers::getTY("");

    double totalPitchRad = (TurretConstants::kCameraMountPitchDeg + m_visionPitchDeg) * (M_PI / 180.0);
    double deltaH = TurretConstants::kHubTargetHeightMeters - TurretConstants::kCameraMountHeightMeters;
    if (totalPitchRad > 0.05 && deltaH > 0.0) {
      m_targetDistanceMeters = deltaH / std::tan(totalPitchRad);
    }
  } else {
    // 2. Fall back to PhotonVision
    auto results = m_camera.GetAllUnreadResults();
    if (!results.empty() && results.back().HasTargets()) {
      auto bestTarget = results.back().GetBestTarget();
      visionFound = true;
      m_hasVisionTarget = true;
      m_lastVisionTargetTime = frc::Timer::GetFPGATimestamp();
      m_visionYawDeg = bestTarget.GetYaw();
      m_visionPitchDeg = bestTarget.GetPitch();
      m_targetFiducialId = bestTarget.GetFiducialId();

      double totalPitchRad = (TurretConstants::kCameraMountPitchDeg + m_visionPitchDeg) * (M_PI / 180.0);
      double deltaH = TurretConstants::kHubTargetHeightMeters - TurretConstants::kCameraMountHeightMeters;
      if (totalPitchRad > 0.05 && deltaH > 0.0) {
        m_targetDistanceMeters = deltaH / std::tan(totalPitchRad);
      }
    }
  }

  // Grace period across frame drops (150ms)
  if (!visionFound) {
    units::time::second_t now = frc::Timer::GetFPGATimestamp();
    if (now - m_lastVisionTargetTime > 0.15_s) {
      m_hasVisionTarget = false;
    }
  }

  // 3. Odometry Target Calculation (for search & fallback)
  double hubX = m_isRedAlliance ? TurretConstants::kRedHubX : TurretConstants::kBlueHubX;
  double hubY = m_isRedAlliance ? TurretConstants::kRedHubY : TurretConstants::kBlueHubY;
  double dx = hubX - robotX.value();
  double dy = hubY - robotY.value();
  double odoDist = std::hypot(dx, dy);

  double fieldAngleDeg = std::atan2(dy, dx) * (180.0 / M_PI);
  // Chassis heading required to aim the fixed shooter (accounting for its mount angle to the left)
  m_odometryTargetHeadingDeg = std::remainder(fieldAngleDeg - TurretConstants::kShooterMountYawOffsetDeg, 360.0);

  if (!m_hasVisionTarget) {
    m_targetDistanceMeters = odoDist;
  }

  // 4. Dynamic Tolerance Calculation
  double effectiveDist = std::max(m_targetDistanceMeters, 0.5);
  double apparentRadiusDeg = std::atan2(TurretConstants::kHubRadiusMeters, effectiveDist) * (180.0 / M_PI);
  m_dynamicToleranceDeg = std::clamp(
      apparentRadiusDeg * TurretConstants::kToleranceMarginRatio,
      TurretConstants::kMinToleranceDeg,
      TurretConstants::kMaxToleranceDeg);

  // 5. Target Lock State
  if (m_hasVisionTarget) {
    double aimError = std::abs(m_visionYawDeg - TurretConstants::kShooterMountYawOffsetDeg);
    m_targetLocked = (aimError <= m_dynamicToleranceDeg);
  } else {
    double headingError = std::abs(std::remainder(m_odometryTargetHeadingDeg - robotHeading.value(), 360.0));
    m_targetLocked = (headingError <= m_dynamicToleranceDeg);
  }
}

double Turret::CalculateAimRotation(double currentGyroYawDeg) {
  if (m_hasVisionTarget) {
    // Optical tracking: in Limelight, positive TX means target is to the RIGHT.
    // In our CCW+ convention, turning right requires negative rotation.
    double error = m_visionYawDeg - TurretConstants::kShooterMountYawOffsetDeg;
    if (std::abs(error) < TurretConstants::kAimToleranceDeg) {
      return 0.0;
    }

    double dErr = (error - m_lastAimErrorDeg) / 0.02;
    m_lastAimErrorDeg = error;

    double out = -TurretConstants::kAimP_Vision * error - TurretConstants::kAimD_Vision * dErr;
    out += std::copysign(TurretConstants::kAimStatic_Vision, -error);
    return std::clamp(out, -TurretConstants::kAimMaxRotOutput, TurretConstants::kAimMaxRotOutput);
  } else {
    // Odometry fallback: turn robot to the calculated field angle to the Hub
    double headingError = std::remainder(m_odometryTargetHeadingDeg - currentGyroYawDeg, 360.0);
    if (std::abs(headingError) < TurretConstants::kAimToleranceDeg) {
      return 0.0;
    }

    double out = TurretConstants::kAimP_Odometry * headingError;
    out += std::copysign(TurretConstants::kAimStatic_Vision, headingError);
    return std::clamp(out, -TurretConstants::kAimMaxRotOutput, TurretConstants::kAimMaxRotOutput);
  }
}

void Turret::Shoot(double triggerValue) {
  if (triggerValue > 0.15) {
    SetShooterAndHoodForDistance(m_targetDistanceMeters);
  } else {
    StopShooter();
  }
}

void Turret::Periodic() {
  frc::SmartDashboard::PutNumber("Shooter/Shooter RPM", GetShooterRPM());
  frc::SmartDashboard::PutNumber("Shooter/Target Shooter RPM", m_targetShooterRPM);
  frc::SmartDashboard::PutNumber("Shooter/Hood Angle Deg", m_currentHoodAngleDeg);
  frc::SmartDashboard::PutBoolean("Shooter/HasVisionTarget", m_hasVisionTarget);
  frc::SmartDashboard::PutBoolean("Shooter/TargetLocked", m_targetLocked);
  frc::SmartDashboard::PutBoolean("Shooter/ReadyToShoot", IsReadyToShoot());
  frc::SmartDashboard::PutNumber("Shooter/VisionYaw Deg", m_visionYawDeg);
  frc::SmartDashboard::PutNumber("Shooter/Distance Meters", m_targetDistanceMeters);
  frc::SmartDashboard::PutNumber("Shooter/DynamicTolerance Deg", m_dynamicToleranceDeg);
  frc::SmartDashboard::PutNumber("Shooter/MountYawOffsetDeg", TurretConstants::kShooterMountYawOffsetDeg);
}

void Turret::UpdateSim(units::time::second_t dt) {
  double rps = m_targetShooterRPM / 60.0;
  m_flywheelLeft.GetSimState().SetRotorVelocity(units::angular_velocity::turns_per_second_t(rps));
  m_flywheelRight.GetSimState().SetRotorVelocity(units::angular_velocity::turns_per_second_t(-rps));
}
