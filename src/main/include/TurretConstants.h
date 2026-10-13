// ============================================================================
// FRC Team 8753 - 2026 Shooter Subsystem Constants (Fixed Mount)
// ============================================================================
// The shooter is fixed forward on the chassis (no azimuth rotation motor),
// slightly pointed to the left. The entire robot turns to face the Hub when
// shooting, and the hood pitch adjusts dynamically for distance.
// ============================================================================

#pragma once

#include <array>
#include <string_view>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/time.h>

namespace TurretConstants {
  // ========== CAN IDs & Hardware Channels ==========
  // Note: Azimuth rotation motor removed (shooter is fixed to the robot chassis)
  constexpr int kShooterLeftMotorId = 5;  ///< Kraken X60 primary shooter flywheel
  constexpr int kShooterRightMotorId = 6; ///< Kraken X60 secondary shooter flywheel
  constexpr int kServoHubId = 8;          ///< REV ServoHub CAN ID

  // CAN bus name for shooter Kraken motors ("outtake" or "rio")
  inline constexpr const char* kShooterCanBus = "outtake";

  // ========== Fixed Shooter Chassis Orientation ==========
  /**
   * Angle that the fixed shooter points relative to the robot's front (+X).
   * Positive = points to the LEFT (+7.0 deg CCW), Negative = points to the RIGHT (CW).
   */
  constexpr double kShooterMountYawOffsetDeg = 7.0;

  /**
   * Limelight Crosshair Calibration Mode:
   * - false (Default / Software Math Mode): Limelight crosshair is centered at (0, 0)
   *   along the robot's forward centerline. Code automatically aims chassis so
   *   Limelight tx settles at -7.0 deg (matching the 7 deg left shooter offset).
   * - true (Limelight UI Mode): Crosshair was calibrated inside Limelight Web UI
   *   directly along the shooter's barrel line. Code aims chassis so Limelight tx settles at 0.0 deg.
   */
  constexpr bool kVisionCalibratedInLimelight = false;

  /// Flywheel gear ratio (motor rotations per flywheel rotation, 1.0 = direct drive)
  constexpr double kShooterGearRatio = 1.0;

  // ========== Hood Pitch Range (REV Smart Servos via ServoHub) ==========
  constexpr double kHoodMinAngleDeg = 15.0;  ///< Resting / lowest pitch angle (degrees)
  constexpr double kHoodMaxAngleDeg = 75.0;  ///< Maximum pitch angle (degrees)
  constexpr double kHoodMinPulseWidthUs = 650.0;
  constexpr double kHoodMaxPulseWidthUs = 2500.0;

  // ========== Shooter Flywheel PID Configuration (Phoenix 6 Slot 0) ==========
  constexpr double kShooterP = 0.5;
  constexpr double kShooterI = 0.0;
  constexpr double kShooterD = 0.01;
  constexpr double kShooterS = 0.05;  ///< Static friction compensation (V)
  constexpr double kShooterV = 0.12;  ///< Velocity feedforward (V / rps)

  constexpr double kShooterDefaultRPM = 2500.0;
  constexpr double kShooterIdleRPM = 0.0;

  // ========== Chassis Auto-Aim Heading Controller ==========
  // When shooting, the drivetrain automatically turns the chassis to face the Hub.
  constexpr double kAimP_Vision = 0.025;      ///< Proportional gain for turning toward vision target
  constexpr double kAimD_Vision = 0.002;      ///< Derivative damping for auto-aim
  constexpr double kAimStatic_Vision = 0.035; ///< Minimum static output to overcome carpet scrub
  constexpr double kAimMaxRotOutput = 0.55;   ///< Maximum rotation output speed allowed during auto-aim
  constexpr double kAimToleranceDeg = 1.5;    ///< Heading deadband (degrees) for target lock

  constexpr double kAimP_Odometry = 0.018;    ///< Proportional gain when searching/tracking with odometry

  // ========== Vision Camera Configuration ==========
  constexpr std::string_view kCameraName = "turret_camera"; ///< Camera name in PhotonVision / Limelight
  constexpr double kCameraMountPitchDeg = 25.0;            ///< Camera pitch angle above horizontal (deg)
  constexpr double kCameraMountHeightMeters = 0.55;        ///< Lens center height above floor (m)
  constexpr double kHubTargetHeightMeters = 2.10;          ///< Target AprilTag center height above floor (m)
  constexpr double kHubRadiusMeters = 0.60;                ///< Target Hub opening radius (m)

  // Dynamic Angular Tolerance (OnyxTronix #2231 Model)
  constexpr double kMinToleranceDeg = 1.2;
  constexpr double kMaxToleranceDeg = 5.5;
  constexpr double kToleranceMarginRatio = 0.45;

  // ========== Field Target Coordinates (Hub Pose for Odometry Fallback) ==========
  constexpr double kRedHubX = 11.91;                       ///< Red Hub X on field (meters)
  constexpr double kRedHubY = 4.05;                        ///< Red Hub Y on field (meters)
  constexpr double kBlueHubX = 4.63;                       ///< Blue Hub X on field (meters)
  constexpr double kBlueHubY = 4.05;                       ///< Blue Hub Y on field (meters)

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
