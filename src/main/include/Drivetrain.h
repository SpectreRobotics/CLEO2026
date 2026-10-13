// ============================================================================
// FRC Team 8753 - 2026 Swerve Drivetrain (from-scratch rewrite)
// ============================================================================
// No WPILib swerve/kinematics/PID/geometry classes are used. All math is here.
// Convention: +X forward, +Y left, +rotation CCW (viewed from above).
// ============================================================================

#pragma once

#include <array>

#include "DrivetrainConstants.h"
#include "SwerveModule.h"

class Drivetrain {
 public:
  Drivetrain();

  /// Apply configs to all 8 motors. Safe to call repeatedly.
  void ConfigureMotors();
  bool AreConfigsApplied() const { return m_configsApplied; }

  /**
   * @param forward   +1 = full forward   (field or robot frame)
   * @param left      +1 = full left
   * @param rotCCW    +1 = full CCW rotation
   * @param speedScale overall output scale 0..1
   * @param headingRad robot heading, CCW+ radians (used if fieldCentric)
   */
  void Drive(double forward, double left, double rotCCW, double speedScale,
             double headingRad, bool fieldCentric);

  void Stop();
  void TestPhase();
  void ResetRunaway();

  /// Call every loop (any mode): telemetry + odometry.
  void Periodic(double headingRad);
  void UpdateSim(double dt);

  void ResetOdometry(double xMeters = 0.0, double yMeters = 0.0) { m_x = xMeters; m_y = yMeters; }
  double GetX() const { return m_x; }
  double GetY() const { return m_y; }

 private:
  double LoopDt();

  std::array<SwerveModule, 4> m_modules;
  bool m_configsApplied = false;
  double m_radius;  // center-to-farthest-module distance

  double m_lastLoopTime = 0.0;
  double m_lastOdoTime = 0.0;
  double m_x = 0.0;
  double m_y = 0.0;

  double m_lastRotCmd = 0.0;
};
