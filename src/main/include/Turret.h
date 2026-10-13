// ============================================================================
// FRC Team 8753 - 2026 Shooter Subsystem Header (Fixed Chassis Mount)
// ============================================================================

#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <ctre/phoenix6/controls/DutyCycleOut.hpp>

#include <rev/ServoHub.h>

#include <vector>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>

#include <photon/PhotonCamera.h>
#include <photon/targeting/PhotonPipelineResult.h>

#include "TurretConstants.h"

/**
 * @brief Fixed-Mount Shooter Subsystem
 *
 * Controls:
 * - Flywheel shooter: 2x Kraken X60 with closed-loop velocity control (opposing pair)
 * - Hood pitch: 2x REV Smart Servos via REV ServoHub (mirrored channels)
 * - Auto-targeting: Limelight / PhotonVision with field odometry fallback.
 *   Calculates chassis rotation demand to align the entire robot with the Hub.
 */
class Turret {
 public:
  enum class TargetingMode {
    kManual,       ///< Manual rotation (no auto-aim)
    kVisionOnly,   ///< Track only when camera detects target
    kOdometryOnly, ///< Track using robot field coordinates and known Hub pose
    kAuto          ///< Vision tracking when available, odometry fallback when searching
  };

  Turret();

  /**
   * @brief Configures flywheel Krakens and REV ServoHub pulse limits.
   */
  void ConfigureMotors();

  // ========== Flywheel Shooter (2x Kraken X60) ==========
  void SetShooterRPM(double rpm);
  void SetShooterVelocity(units::angular_velocity::turns_per_second_t rps);
  void SetShooterPercent(double percent);
  void StopShooter();
  double GetShooterRPM();
  bool IsShooterAtSpeed(double targetRPM, double toleranceRPM = 150.0);
  double GetTargetShooterRPM() const { return m_targetShooterRPM; }

  // ========== Hood Pitch Servos (2x REV Smart Servos) ==========
  void SetHoodAngle(units::angle::degree_t angle);
  void SetHoodPulseWidth(double pulseWidthUs);
  double GetHoodAngleDegrees() const { return m_currentHoodAngleDeg; }

  // ========== Auto-Targeting & Chassis Aiming ==========
  /**
   * @brief Updates vision target detection and odometry fallback calculations.
   */
  void UpdateAutoTarget(units::length::meter_t robotX = units::length::meter_t(0),
                        units::length::meter_t robotY = units::length::meter_t(0),
                        units::angle::degree_t robotHeading = units::angle::degree_t(0));

  /**
   * @brief Calculates chassis rotation command (rotCCW in [-1, 1]) to point the shooter at the Hub.
   * @param currentGyroYawDeg Robot gyro yaw in degrees (CCW+).
   * @return rotCCW rotation output for Drivetrain::Drive.
   */
  double CalculateAimRotation(double currentGyroYawDeg);

  bool HasVisionTarget() const { return m_hasVisionTarget; }
  double GetVisionYaw() const { return m_visionYawDeg; }
  double GetVisionPitch() const { return m_visionPitchDeg; }
  double GetTargetDistanceMeters() const { return m_targetDistanceMeters; }
  double GetDynamicToleranceDegrees() const { return m_dynamicToleranceDeg; }
  bool IsTargetLocked() const { return m_targetLocked; }

  /**
   * @brief Returns true if the robot is aimed within tolerance AND flywheels are at speed.
   */
  bool IsReadyToShoot() {
    return m_targetLocked && (m_targetShooterRPM > 500.0) && IsShooterAtSpeed(m_targetShooterRPM, 150.0);
  }

  void SetTargetingMode(TargetingMode mode) { m_targetingMode = mode; }
  TargetingMode GetTargetingMode() const { return m_targetingMode; }
  void SetAllianceRed(bool isRed) { m_isRedAlliance = isRed; }

  /**
   * @brief Hold trigger (>0.15) to spin up flywheels + raise hood for target distance.
   *        Release trigger to stop shooter + lower hood.
   */
  void Shoot(double triggerValue);

  void UpdateSim(units::time::second_t dt);
  void Periodic();

  double CalculateTargetRPM(double distanceMeters) const;
  double CalculateTargetHoodAngle(double distanceMeters) const;
  void SetShooterAndHoodForDistance(double distanceMeters);

 private:
  // Hardware Motors
  ctre::phoenix6::hardware::TalonFX m_flywheelLeft{
      TurretConstants::kShooterLeftMotorId,
      ctre::phoenix6::CANBus{TurretConstants::kShooterCanBus}};
  ctre::phoenix6::hardware::TalonFX m_flywheelRight{
      TurretConstants::kShooterRightMotorId,
      ctre::phoenix6::CANBus{TurretConstants::kShooterCanBus}};

  // REV ServoHub & Hood Servos
  rev::servohub::ServoHub m_servoHub{TurretConstants::kServoHubId};
  rev::servohub::ServoChannel m_hoodServo0 =
      m_servoHub.GetServoChannel(rev::servohub::ServoChannel::ChannelId::kChannelId0);
  rev::servohub::ServoChannel m_hoodServo1 =
      m_servoHub.GetServoChannel(rev::servohub::ServoChannel::ChannelId::kChannelId1);

  // Vision Cameras
  photon::PhotonCamera m_camera{TurretConstants::kCameraName};

  // Control Requests
  ctre::phoenix6::controls::VelocityVoltage m_velocityRequest{0_tps};
  ctre::phoenix6::controls::DutyCycleOut m_dutyCycleRequest{0.0};

  // State
  double m_targetShooterRPM = 0.0;
  double m_currentHoodAngleDeg = 15.0;

  // Auto-Targeting & Vision State
  TargetingMode m_targetingMode = TargetingMode::kAuto;
  bool m_hasVisionTarget = false;
  bool m_targetLocked = false;
  double m_visionYawDeg = 0.0;
  double m_visionPitchDeg = 0.0;
  double m_targetDistanceMeters = 3.0;
  double m_dynamicToleranceDeg = TurretConstants::kMaxToleranceDeg;
  int m_targetFiducialId = -1;
  bool m_isRedAlliance = false;
  units::time::second_t m_lastVisionTargetTime{0_s};

  double m_lastAimErrorDeg = 0.0;
  double m_odometryTargetHeadingDeg = 0.0;
};
