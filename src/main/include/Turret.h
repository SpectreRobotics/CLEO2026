// ============================================================================
// FRC Team 8753 - 2026 Modular Turret Subsystem Header
// ============================================================================

#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/MotionMagicVoltage.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <ctre/phoenix6/controls/DutyCycleOut.hpp>

#include <rev/ServoHub.h>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/time.h>

#include "TurretConstants.h"

/**
 * @brief Modular Turret Subsystem
 *
 * Controls:
 * - Azimuth rotation: Kraken X44 with Motion Magic position control
 * - Flywheel shooter: 2x Kraken X60 with closed-loop velocity control
 * - Hood pitch: 2x REV Smart Servos via REV ServoHub (opposing channels)
 */
class Turret {
 public:
  Turret();

  /**
   * @brief Applies motor configs, PID gains, Motion Magic settings, and servo pulse ranges.
   */
  void ConfigureMotors();

  // ========== Azimuth Rotation (Kraken X44) ==========
  /**
   * @brief Positions the turret to a target field/robot azimuth angle.
   * @param angle Target angle in degrees (clamped to safe travel limits).
   */
  void SetRotationAngle(units::angle::degree_t angle);

  /**
   * @brief Manual jog control for turret rotation.
   * @param percent Output duty cycle (-1.0 to 1.0).
   */
  void SetRotationSpeed(double percent);

  /**
   * @brief Calibrates the current physical position as 0 degrees.
   */
  void ZeroRotation();

  /**
   * @brief Returns the current turret azimuth angle in degrees.
   */
  double GetRotationAngleDegrees();

  // ========== Flywheel Shooter (2x Kraken X60) ==========
  /**
   * @brief Runs flywheels at a closed-loop velocity target in RPM.
   */
  void SetShooterRPM(double rpm);

  /**
   * @brief Runs flywheels at a closed-loop velocity in turns per second.
   */
  void SetShooterVelocity(units::angular_velocity::turns_per_second_t rps);

  /**
   * @brief Runs flywheels at an open-loop duty cycle.
   */
  void SetShooterPercent(double percent);

  /**
   * @brief Stops both shooter flywheel motors.
   */
  void StopShooter();

  /**
   * @brief Gets current shooter velocity in RPM.
   */
  double GetShooterRPM();

  /**
   * @brief Checks if flywheels are within tolerance of the target RPM.
   */
  bool IsShooterAtSpeed(double targetRPM, double toleranceRPM = 100.0);

  // ========== Hood Pitch Servos (2x REV Smart Servos) ==========
  /**
   * @brief Sets the hood angle in degrees (adjusts trajectory pitch).
   * @param angle Desired angle between kHoodMinAngleDeg and kHoodMaxAngleDeg.
   */
  void SetHoodAngle(units::angle::degree_t angle);

  /**
   * @brief Sets raw microsecond pulse width on both servos.
   */
  void SetHoodPulseWidth(double pulseWidthUs);

  // ========== Periodic Telemetry ==========
  /**
   * @brief Telemetry update to be called from RobotPeriodic.
   */
  void Periodic();

 private:
  // Hardware Motors
  ctre::phoenix6::hardware::TalonFX m_rotationMotor{TurretConstants::kRotationMotorId};
  ctre::phoenix6::hardware::TalonFX m_flywheelLeft{TurretConstants::kShooterLeftMotorId};
  ctre::phoenix6::hardware::TalonFX m_flywheelRight{TurretConstants::kShooterRightMotorId};

  // REV ServoHub & Hood Servos
  rev::servohub::ServoHub m_servoHub{TurretConstants::kServoHubId};
  rev::servohub::ServoChannel m_hoodServo0 =
      m_servoHub.GetServoChannel(rev::servohub::ServoChannel::ChannelId::kChannelId0);
  rev::servohub::ServoChannel m_hoodServo1 =
      m_servoHub.GetServoChannel(rev::servohub::ServoChannel::ChannelId::kChannelId1);

  // Control Requests
  ctre::phoenix6::controls::MotionMagicVoltage m_motionMagicRequest{0_tr};
  ctre::phoenix6::controls::VelocityVoltage m_velocityRequest{0_tps};
  ctre::phoenix6::controls::DutyCycleOut m_dutyCycleRequest{0.0};

  // State
  double m_targetAngleDeg = 0.0;
  double m_targetShooterRPM = 0.0;
  double m_currentHoodAngleDeg = 15.0;
};
