// ============================================================================
// FRC Team 8753 - 2026 Modular Turret Subsystem Implementation
// ============================================================================

#include "Turret.h"

#define _USE_MATH_DEFINES
#include <algorithm>
#include <cmath>
#include <frc/DriverStation.h>
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
  double angle = table.front().hoodAngleDeg;

  if (distanceMeters <= table.front().distanceMeters) {
    angle = table.front().hoodAngleDeg;
  } else if (distanceMeters >= table.back().distanceMeters) {
    angle = table.back().hoodAngleDeg;
  } else {
    for (size_t i = 0; i < table.size() - 1; ++i) {
      if (distanceMeters >= table[i].distanceMeters && distanceMeters <= table[i + 1].distanceMeters) {
        double t = (distanceMeters - table[i].distanceMeters) /
                   (table[i + 1].distanceMeters - table[i].distanceMeters);
        angle = table[i].hoodAngleDeg + t * (table[i + 1].hoodAngleDeg - table[i].hoodAngleDeg);
        break;
      }
    }
  }
  return std::clamp(angle, TurretConstants::kHoodMinAngleDeg, TurretConstants::kHoodMaxAngleDeg);
}

void Turret::SetShooterAndHoodForDistance(double distanceMeters) {
  if (distanceMeters > 0.5) {
    SetShooterRPM(CalculateTargetRPM(distanceMeters));
    SetHoodAngle(units::angle::degree_t(CalculateTargetHoodAngle(distanceMeters)));
  }
}

void Turret::UpdateAutoTarget(units::length::meter_t robotX,
                              units::length::meter_t robotY,
                              units::angle::degree_t robotHeading) {
  if (m_targetingMode == TargetingMode::kManual) {
    return;
  }

  // 1. Read latest unread pipeline results from PhotonVision
  std::vector<photon::PhotonPipelineResult> results = m_camera.GetAllUnreadResults();
  bool visionFound = false;

  if (!results.empty()) {
    const auto& latestResult = results.back();
    if (latestResult.HasTargets()) {
      auto bestTarget = latestResult.GetBestTarget();
      visionFound = true;
      m_hasVisionTarget = true;
      m_visionYawDeg = bestTarget.GetYaw();
      m_visionPitchDeg = bestTarget.GetPitch();
      m_targetFiducialId = bestTarget.GetFiducialId();

      // Estimate distance from 3D camera-to-target transform or pitch trigonometry
      auto transform = bestTarget.GetBestCameraToTarget();
      double dist3d = std::hypot(transform.X().value(), transform.Y().value());
      if (dist3d > 0.4) {
        m_targetDistanceMeters = dist3d;
      } else {
        double totalPitchRad = (TurretConstants::kCameraMountPitchDeg + m_visionPitchDeg) * (M_PI / 180.0);
        double deltaH = TurretConstants::kHubTargetHeightMeters - TurretConstants::kCameraMountHeightMeters;
        if (totalPitchRad > 0.05) {
          m_targetDistanceMeters = deltaH / std::tan(totalPitchRad);
        }
      }
    }
  }

  if (!visionFound) {
    m_hasVisionTarget = false;
  }

  // 2. Closed-loop control selection
  if (m_hasVisionTarget && (m_targetingMode == TargetingMode::kAuto || m_targetingMode == TargetingMode::kVisionOnly)) {
    // Dynamic Turret Tolerance calculation (OnyxTronix #2231 model)
    double effectiveDist = std::max(m_targetDistanceMeters, 0.5);
    double apparentRadiusDeg = std::atan2(TurretConstants::kHubRadiusMeters, effectiveDist) * (180.0 / M_PI);
    m_dynamicToleranceDeg = std::clamp(
        apparentRadiusDeg * TurretConstants::kToleranceMarginRatio,
        TurretConstants::kMinToleranceDeg,
        TurretConstants::kMaxToleranceDeg);

    // Target locked if absolute camera yaw is within dynamic tolerance window
    m_targetLocked = (std::abs(m_visionYawDeg) <= m_dynamicToleranceDeg);

    // Camera is mounted directly on turret: adjust current azimuth by yaw offset
    double currentAngle = GetRotationAngleDegrees();
    double targetTurretDeg = currentAngle + (m_visionYawDeg * TurretConstants::kVisionYawSign) -
                             TurretConstants::kCameraMountYawOffsetDeg;

    SetRotationAngle(units::angle::degree_t(targetTurretDeg));

    // Dynamic ballistics adjustment
    SetShooterAndHoodForDistance(m_targetDistanceMeters);

  } else if ((!m_hasVisionTarget && m_targetingMode == TargetingMode::kAuto) ||
             m_targetingMode == TargetingMode::kOdometryOnly) {
    // Fallback / Pre-alignment using robot field odometry and known Hub pose
    double hubX = TurretConstants::kBlueHubX;
    double hubY = TurretConstants::kBlueHubY;
    auto alliance = frc::DriverStation::GetAlliance();
    if (alliance && alliance.value() == frc::DriverStation::Alliance::kRed) {
      hubX = TurretConstants::kRedHubX;
      hubY = TurretConstants::kRedHubY;
    }

    // Account for robot heading and turret physical offset on robot frame
    double headingRad = robotHeading.value() * (M_PI / 180.0);
    double turretFieldX = robotX.value() +
        (TurretConstants::kTurretOffsetX * std::cos(headingRad) -
         TurretConstants::kTurretOffsetY * std::sin(headingRad));
    double turretFieldY = robotY.value() +
        (TurretConstants::kTurretOffsetX * std::sin(headingRad) +
         TurretConstants::kTurretOffsetY * std::cos(headingRad));

    double dx = hubX - turretFieldX;
    double dy = hubY - turretFieldY;
    double dist = std::hypot(dx, dy);
    m_targetDistanceMeters = dist;

    // Field-relative angle to Hub
    double fieldAngleDeg = std::atan2(dy, dx) * (180.0 / M_PI);

    // Robot-relative target angle
    double robotRelativeAngleDeg = std::remainder(fieldAngleDeg - robotHeading.value(), 360.0);
    if (robotRelativeAngleDeg < 0.0) {
      robotRelativeAngleDeg += 360.0;
    }

    double effectiveDist = std::max(dist, 0.5);
    double apparentRadiusDeg = std::atan2(TurretConstants::kHubRadiusMeters, effectiveDist) * (180.0 / M_PI);
    m_dynamicToleranceDeg = std::clamp(
        apparentRadiusDeg * TurretConstants::kToleranceMarginRatio,
        TurretConstants::kMinToleranceDeg,
        TurretConstants::kMaxToleranceDeg);

    SetRotationAngle(units::angle::degree_t(robotRelativeAngleDeg));

    double angleError = std::abs(std::remainder(GetRotationAngleDegrees() - robotRelativeAngleDeg, 360.0));
    m_targetLocked = (angleError <= m_dynamicToleranceDeg);

    if (dist > 0.5) {
      SetShooterAndHoodForDistance(dist);
    }
  } else {
    m_targetLocked = false;
  }
}

// ========== Telemetry ==========

void Turret::Periodic() {
  frc::SmartDashboard::PutNumber("Turret/Angle Deg", GetRotationAngleDegrees());
  frc::SmartDashboard::PutNumber("Turret/Target Angle Deg", m_targetAngleDeg);
  frc::SmartDashboard::PutNumber("Turret/Shooter RPM", GetShooterRPM());
  frc::SmartDashboard::PutNumber("Turret/Target Shooter RPM", m_targetShooterRPM);
  frc::SmartDashboard::PutNumber("Turret/Hood Angle Deg", m_currentHoodAngleDeg);

  // Vision & Auto-Targeting Telemetry
  frc::SmartDashboard::PutBoolean("Turret/HasTarget", m_hasVisionTarget);
  frc::SmartDashboard::PutBoolean("Turret/TargetLocked", m_targetLocked);
  frc::SmartDashboard::PutNumber("Turret/VisionYaw Deg", m_visionYawDeg);
  frc::SmartDashboard::PutNumber("Turret/VisionPitch Deg", m_visionPitchDeg);
  frc::SmartDashboard::PutNumber("Turret/Distance Meters", m_targetDistanceMeters);
  frc::SmartDashboard::PutNumber("Turret/DynamicTolerance Deg", m_dynamicToleranceDeg);
  frc::SmartDashboard::PutNumber("Turret/Target Tag ID", m_targetFiducialId);

  std::string modeStr = "Auto";
  switch (m_targetingMode) {
    case TargetingMode::kManual: modeStr = "Manual"; break;
    case TargetingMode::kVisionOnly: modeStr = "VisionOnly"; break;
    case TargetingMode::kOdometryOnly: modeStr = "OdometryOnly"; break;
    case TargetingMode::kAuto: modeStr = "Auto"; break;
  }
  frc::SmartDashboard::PutString("Turret/TargetingMode", modeStr);
}
