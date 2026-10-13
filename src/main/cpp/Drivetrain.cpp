// ============================================================================
// FRC Team 8753 - 2026 Swerve Drivetrain implementation (from scratch)
// ============================================================================

#include "Drivetrain.h"

#include <algorithm>
#include <cmath>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

namespace DC = DrivetrainConstants;

namespace {
constexpr double kPi = 3.14159265358979323846;
}

Drivetrain::Drivetrain()
    : m_modules{SwerveModule{DC::kModules[0]}, SwerveModule{DC::kModules[1]},
                SwerveModule{DC::kModules[2]}, SwerveModule{DC::kModules[3]}} {
  m_radius = 0.0;
  for (const auto& m : DC::kModules) {
    m_radius = std::max(m_radius, std::hypot(m.xMeters, m.yMeters));
  }
}

void Drivetrain::ConfigureMotors() {
  bool ok = true;
  for (auto& m : m_modules) ok &= m.Configure();
  m_configsApplied = ok;
  frc::SmartDashboard::PutBoolean("Swerve/AllConfigsApplied", ok);
}

double Drivetrain::LoopDt() {
  double now = frc::Timer::GetFPGATimestamp().value();
  double dt = now - m_lastLoopTime;
  m_lastLoopTime = now;
  if (dt <= 0.0 || dt > 0.1) dt = 0.02;
  return dt;
}

void Drivetrain::Drive(double forward, double left, double rotCCW, double speedScale,
                       double headingRad, bool fieldCentric) {
  double dt = LoopDt();
  m_lastRotCmd = rotCCW;

  // ---- Field-centric: rotate the field-frame request into the robot frame ----
  // Robot frame = field frame rotated by heading, so apply -heading.
  double vx = forward;
  double vy = left;
  if (fieldCentric) {
    double c = std::cos(headingRad);
    double s = std::sin(headingRad);
    vx = forward * c + left * s;
    vy = -forward * s + left * c;
  }

  bool idle = std::abs(vx) < 1e-3 && std::abs(vy) < 1e-3 && std::abs(rotCCW) < 1e-3;
  if (idle) {
    for (auto& m : m_modules) m.Hold(dt);
    return;
  }

  // ---- Inverse kinematics ----
  // Wheel velocity = robot velocity + omega x r
  //   omega x r (2D, CCW+) = (-omega * y, +omega * x)
  // rotCCW is normalized so rotCCW=1 makes the farthest wheel move at speed 1.
  double speed[4];
  double angle[4];
  double maxSpeed = 0.0;
  for (int i = 0; i < 4; ++i) {
    const auto& mc = DC::kModules[i];
    double wx = vx - rotCCW * (mc.yMeters / m_radius);
    double wy = vy + rotCCW * (mc.xMeters / m_radius);
    speed[i] = std::hypot(wx, wy);
    angle[i] = std::atan2(wy, wx) / (2.0 * kPi);  // turns, CCW+, 0 = forward
    maxSpeed = std::max(maxSpeed, speed[i]);
  }

  // ---- Desaturate: keep ratios, cap fastest wheel at 1 ----
  double norm = (maxSpeed > 1.0) ? maxSpeed : 1.0;
  for (int i = 0; i < 4; ++i) {
    m_modules[i].SetDesired(speed[i] / norm * speedScale, angle[i], dt);
  }
}

void Drivetrain::Stop() {
  for (auto& m : m_modules) m.Stop();
}

void Drivetrain::TestPhase() {
  for (auto& m : m_modules) m.RunPhaseTest();
}

void Drivetrain::ResetRunaway() {
  for (auto& m : m_modules) m.ResetRunawayLatch();
}

void Drivetrain::Periodic(double headingRad) {
  for (auto& m : m_modules) m.PublishTelemetry();

  // =========================================================================
  // CANCODER OFFSETS TABLE (Write down these Raw values when wheels point straight forward!)
  // =========================================================================
  frc::SmartDashboard::PutNumber("Swerve/Calibration/FL_RawTurns", m_modules[0].GetRawEncoderTurns());
  frc::SmartDashboard::PutNumber("Swerve/Calibration/FR_RawTurns", m_modules[1].GetRawEncoderTurns());
  frc::SmartDashboard::PutNumber("Swerve/Calibration/BL_RawTurns", m_modules[2].GetRawEncoderTurns());
  frc::SmartDashboard::PutNumber("Swerve/Calibration/BR_RawTurns", m_modules[3].GetRawEncoderTurns());

  frc::SmartDashboard::PutNumber("Swerve/Calibration/FL_CurrentOffset", m_modules[0].Constants().encoderOffsetTurns);
  frc::SmartDashboard::PutNumber("Swerve/Calibration/FR_CurrentOffset", m_modules[1].Constants().encoderOffsetTurns);
  frc::SmartDashboard::PutNumber("Swerve/Calibration/BL_CurrentOffset", m_modules[2].Constants().encoderOffsetTurns);
  frc::SmartDashboard::PutNumber("Swerve/Calibration/BR_CurrentOffset", m_modules[3].Constants().encoderOffsetTurns);

  frc::SmartDashboard::PutNumber("Swerve/Calibration/FL_CurrentAngleDeg", m_modules[0].GetAngleTurns() * 360.0);
  frc::SmartDashboard::PutNumber("Swerve/Calibration/FR_CurrentAngleDeg", m_modules[1].GetAngleTurns() * 360.0);
  frc::SmartDashboard::PutNumber("Swerve/Calibration/BL_CurrentAngleDeg", m_modules[2].GetAngleTurns() * 360.0);
  frc::SmartDashboard::PutNumber("Swerve/Calibration/BR_CurrentAngleDeg", m_modules[3].GetAngleTurns() * 360.0);

  // ---- Odometry (forward kinematics, least squares) ----
  double now = frc::Timer::GetFPGATimestamp().value();
  double dt = now - m_lastOdoTime;
  m_lastOdoTime = now;
  if (dt <= 0.0 || dt > 0.1) return;

  // Translation = average of module velocity vectors (rotation terms cancel
  // for a symmetric chassis).
  double sumX = 0.0, sumY = 0.0;
  for (auto& m : m_modules) {
    double v = m.GetWheelSpeedMps();
    double a = m.GetAngleTurns() * 2.0 * kPi;
    sumX += v * std::cos(a);
    sumY += v * std::sin(a);
  }
  double robotVx = sumX / 4.0;
  double robotVy = sumY / 4.0;

  // Robot frame -> field frame
  double c = std::cos(headingRad);
  double s = std::sin(headingRad);
  m_x += (robotVx * c - robotVy * s) * dt;
  m_y += (robotVx * s + robotVy * c) * dt;

  frc::SmartDashboard::PutNumber("Odometry/X", m_x);
  frc::SmartDashboard::PutNumber("Odometry/Y", m_y);
  frc::SmartDashboard::PutNumber("Odometry/RobotVx", robotVx);
  frc::SmartDashboard::PutNumber("Odometry/RobotVy", robotVy);
}

void Drivetrain::UpdateSim(double dt) {
  for (auto& m : m_modules) m.UpdateSim(dt);
}
