// ============================================================================
// FRC Team 8753 - 2026 Turret Constants
// ============================================================================

#pragma once

#include <array>
#include <string_view>
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

  // ========== PhotonVision Camera Configuration ==========
  constexpr std::string_view kCameraName = "turret_camera"; ///< Camera name in PhotonVision UI
  constexpr double kVisionYawSign = 1.0;                   ///< 1.0 if CCW positive, -1.0 if inverted
  constexpr double kCameraMountYawOffsetDeg = 0.0;         ///< Optical bore alignment offset (deg)
  constexpr double kCameraMountPitchDeg = 25.0;            ///< Camera pitch angle above horizontal (deg)
  constexpr double kCameraMountHeightMeters = 0.55;        ///< Lens center height above floor (m)
  constexpr double kHubTargetHeightMeters = 2.10;          ///< Target AprilTag center height above floor (m)
  constexpr double kHubRadiusMeters = 0.60;                ///< Target Hub opening radius (m)

  // ========== Dynamic Turret Tolerance (OnyxTronix #2231 Model) ==========
  constexpr double kMinToleranceDeg = 1.2;                 ///< Tightest tolerance at long range (deg)
  constexpr double kMaxToleranceDeg = 5.5;                 ///< Widest tolerance at close range (deg)
  constexpr double kToleranceMarginRatio = 0.45;           ///< Margin scaling factor of apparent angular radius

  // ========== Field Target Coordinates (Fallback / Search via Odometry) ==========
  constexpr double kRedHubX = 11.91;                       ///< Red Hub X on field (meters)
  constexpr double kRedHubY = 4.05;                        ///< Red Hub Y on field (meters)
  constexpr double kBlueHubX = 4.63;                       ///< Blue Hub X on field (meters)
  constexpr double kBlueHubY = 4.05;                       ///< Blue Hub Y on field (meters)
  constexpr double kTurretOffsetX = 0.0;                   ///< Turret pivot X offset from robot center (m)
  constexpr double kTurretOffsetY = 0.0;                   ///< Turret pivot Y offset from robot center (m)

  // ========== Ballistic Calibration Table (Distance -> Hood Angle & Shooter RPM) ==========
  struct BallisticPoint {
    double distanceMeters;
    double hoodAngleDeg;
    double shooterRPM;
  };

  constexpr std::array<BallisticPoint, 6> kBallisticTable{{
      {1.5, 20.0, 2200.0},
      {2.5, 28.0, 2600.0},
      {3.5, 36.0, 3050.0},
      {4.5, 45.0, 3550.0},
      {5.5, 54.0, 4100.0},
      {6.5, 62.0, 4700.0}
  }};
}

