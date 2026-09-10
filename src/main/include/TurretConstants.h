// ============================================================================
// FRC Team 8753 - 2026 Turret Constants
// ============================================================================

#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/time.h>

namespace TurretConstants {
  // ========== CAN IDs & Hardware Channels ==========
  constexpr int kRotationMotorId = 7;     ///< Kraken X44 turret azimuth rotation
  constexpr int kShooterLeftMotorId = 5;  ///< Kraken X60 primary shooter flywheel
  constexpr int kShooterRightMotorId = 6; ///< Kraken X60 secondary shooter flywheel
  constexpr int kServoHubId = 8;          ///< REV ServoHub CAN ID

  // ========== Gear Ratios & Conversions (UPDATE WITH YOUR EXACT RATIOS) ==========
  /**
   * Turret azimuth gearing:
   * By default: 4.4 motor rotations = 345 degrees of turret travel.
   * Adjust these values when physical gear ratios are measured!
   */
  constexpr double kTurretMaxAngleDegrees = 345.0;
  constexpr double kTurretMaxMotorRotations = 4.4;
  constexpr double kDegreesPerMotorRotation = kTurretMaxAngleDegrees / kTurretMaxMotorRotations;
  constexpr double kMotorRotationsPerDegree = kTurretMaxMotorRotations / kTurretMaxAngleDegrees;

  /// Flywheel gear ratio (motor rotations per flywheel rotation, 1.0 = direct drive)
  constexpr double kShooterGearRatio = 1.0;

  // ========== Soft Limits & Safe Travel Range ==========
  constexpr double kTurretMinAngleDeg = 0.0;
  constexpr double kTurretMaxAngleDeg = 345.0;

  // Hood pitch angle limits (degrees)
  constexpr double kHoodMinAngleDeg = 15.0;
  constexpr double kHoodMaxAngleDeg = 75.0;

  // Hood servo pulse width range (microseconds) for REV Smart Servos
  constexpr double kHoodMinPulseWidthUs = 650.0;
  constexpr double kHoodMaxPulseWidthUs = 2500.0;

  // ========== Turret Rotation PID & Motion Magic Configuration ==========
  constexpr double kRotationP = 2.4;
  constexpr double kRotationI = 0.0;
  constexpr double kRotationD = 0.2;
  constexpr double kRotationS = 0.15;                      ///< Static friction compensation (V)
  constexpr double kRotationV = 0.09;                      ///< Velocity feedforward (V / rps)
  constexpr double kRotationA = 0.01;                      ///< Acceleration feedforward (V / rps/s)

  constexpr double kRotationCruiseVelocityRps = 18.0;     ///< Motion Magic cruise velocity (rps)
  constexpr double kRotationAccelerationRps2 = 70.0;      ///< Motion Magic acceleration (rps/s)
  constexpr double kRotationJerkRps3 = 800.0;             ///< Motion Magic jerk (rps/s/s)

  // ========== Shooter Flywheel PID Configuration ==========
  constexpr double kShooterP = 0.5;
  constexpr double kShooterI = 0.0;
  constexpr double kShooterD = 0.01;
  constexpr double kShooterS = 0.05;                      ///< Static friction compensation (V)
  constexpr double kShooterV = 0.12;                      ///< Velocity feedforward (V / rps)

  // ========== Default Operating Setpoints ==========
  constexpr double kShooterDefaultRPM = 2500.0;
  constexpr double kShooterIdleRPM = 0.0;
  constexpr double kManualRotationSpeed = 0.2;            ///< Manual jog duty cycle (0.0 - 1.0)
}
