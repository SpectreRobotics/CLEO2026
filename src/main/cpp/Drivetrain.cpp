// ============================================================================
// FRC Team 8753 - 2026 Swerve Drivetrain (Restored from swerve-base / 2026_Bot)
// Runs Swerve 1 (BR 10/11/12), Swerve 2 (FR 20/21/22), and Swerve 4 (BL 40/41/42)
// Swerve 3 (FL 30/31/32) disabled/ignored in Coast mode
// ============================================================================

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <algorithm>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/time.h>

#include "Drivetrain.h"
#include "DrivetrainConstants.h"

using namespace ctre::phoenix6;
using namespace units::literals;
namespace DC = DrivetrainConstants;

namespace {
// Straight-ahead AbsolutePosition readings from Phoenix Tuner X screenshots:
//   Swerve 1 (BR, CANcoder 12 on "rio"):       -0.297363 turns
//   Swerve 2 (FR, CANcoder 22 on "rio"):       -0.173584 turns
//   Swerve 4 (BL, CANcoder 42 on "rio"):        0.158203 turns
//   Swerve 3 (FL, CANcoder 32 on "intakebus"): -0.359131 turns (disabled)
constexpr double kSwerve1BRZeroTurns = -0.297363;
constexpr double kSwerve2FRZeroTurns = -0.173584;
constexpr double kSwerve4BLZeroTurns =  0.158203;
}  // namespace

Drivetrain::Drivetrain() {
  positionFWDField = units::length::meter_t(0);
  positionSTRField = units::length::meter_t(0);

  ConfigureMotors();
}

void Drivetrain::ConfigureMotors() {
  // Drive Krakens configurations: Swerve 1 (BR), Swerve 2 (FR), Swerve 4 (BL) active
  driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = units::time::second_t(0.5);
  driveConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

  m_FR_Drive.GetConfigurator().Apply(driveConfig);
  m_BL_Drive.GetConfigurator().Apply(driveConfig);
  m_BR_Drive.GetConfigurator().Apply(driveConfig);

  // Swerve 3 (FL: Drive 30, Steer 31) disabled in Coast mode so it casters freely
  ctre::phoenix6::configs::TalonFXConfiguration flCoastConfig{};
  flCoastConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
  m_FL_Drive.GetConfigurator().Apply(flCoastConfig);
  m_FL_Steer.GetConfigurator().Apply(flCoastConfig);
  m_FL_Drive.Set(0.0);
  m_FL_Steer.Set(0.0);

  // Steer Krakens (Swerve 2 FR, Swerve 4 BL, Swerve 1 BR):
  // Brake mode + reduced PeakDutyCycle (0.35) + 0.08s ramp + reduced kP (0.65) to eliminate oscillation
  configFR.Feedback.FeedbackRemoteSensorID = CANcoderFR.GetDeviceID();
  configFR.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
  configFR.Feedback.SensorToMechanismRatio = 1.0;
  configFR.Feedback.RotorToSensorRatio = 18;
  configFR.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = units::time::second_t(0.08);
  configFR.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
  configFR.MotorOutput.PeakForwardDutyCycle = 0.35;
  configFR.MotorOutput.PeakReverseDutyCycle = -0.35;
  configFR.ClosedLoopGeneral.ContinuousWrap = true;

  configBL.Feedback.FeedbackRemoteSensorID = CANcoderBL.GetDeviceID();
  configBL.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
  configBL.Feedback.SensorToMechanismRatio = 1.0;
  configBL.Feedback.RotorToSensorRatio = 18;
  configBL.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = units::time::second_t(0.08);
  configBL.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
  configBL.MotorOutput.PeakForwardDutyCycle = 0.35;
  configBL.MotorOutput.PeakReverseDutyCycle = -0.35;
  configBL.ClosedLoopGeneral.ContinuousWrap = true;

  configBR.Feedback.FeedbackRemoteSensorID = CANcoderBR.GetDeviceID();
  configBR.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
  configBR.Feedback.SensorToMechanismRatio = 1.0;
  configBR.Feedback.RotorToSensorRatio = 18;
  configBR.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = units::time::second_t(0.08);
  configBR.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
  configBR.MotorOutput.PeakForwardDutyCycle = 0.35;
  configBR.MotorOutput.PeakReverseDutyCycle = -0.35;
  configBR.ClosedLoopGeneral.ContinuousWrap = true;

  // Tuned down proportional gain (0.65 instead of 2.0) to prevent overshoot and oscillation
  Posconfig.kP = 0.65;
  Posconfig.kI = 0.0;
  Posconfig.kD = 0.0;

  m_FR_Steer.GetConfigurator().Apply(configFR);
  m_FR_Steer.GetConfigurator().Apply(Posconfig);
  m_BL_Steer.GetConfigurator().Apply(configBL);
  m_BL_Steer.GetConfigurator().Apply(Posconfig);
  m_BR_Steer.GetConfigurator().Apply(configBR);
  m_BR_Steer.GetConfigurator().Apply(Posconfig);
}

void Drivetrain::DisabledInit() {
  m_FL_Drive.Set(0.0);
  m_FR_Drive.Set(0.0);
  m_BL_Drive.Set(0.0);
  m_BR_Drive.Set(0.0);
  m_FL_Steer.Set(0.0);
  m_FR_Steer.Set(0.0);
  m_BL_Steer.Set(0.0);
  m_BR_Steer.Set(0.0);
  oldspeedFL = 0; oldspeedFR = 0; oldspeedBL = 0; oldspeedBR = 0;
}

void Drivetrain::DisabledPeriodic() {
  m_FL_Drive.Set(0.0);
  m_FL_Steer.Set(0.0);

  frc::SmartDashboard::PutNumber(
      "Swerve/FR_ZeroedTurns",
      std::remainder(m_FR_Steer.GetPosition().GetValueAsDouble() - kSwerve2FRZeroTurns, 1.0));
  frc::SmartDashboard::PutNumber(
      "Swerve/BL_ZeroedTurns",
      std::remainder(m_BL_Steer.GetPosition().GetValueAsDouble() - kSwerve4BLZeroTurns, 1.0));
  frc::SmartDashboard::PutNumber(
      "Swerve/BR_ZeroedTurns",
      std::remainder(m_BR_Steer.GetPosition().GetValueAsDouble() - kSwerve1BRZeroTurns, 1.0));
}

void Drivetrain::TestZeroAzimuth() {
  ctre::phoenix6::controls::PositionDutyCycle steerPos{0_tr};
  m_FL_Steer.Set(0.0);  // Swerve 3 disabled
  m_FR_Steer.SetControl(steerPos.WithPosition(units::angle::turn_t{kSwerve2FRZeroTurns}));
  m_BL_Steer.SetControl(steerPos.WithPosition(units::angle::turn_t{kSwerve4BLZeroTurns}));
  m_BR_Steer.SetControl(steerPos.WithPosition(units::angle::turn_t{kSwerve1BRZeroTurns}));
  m_FL_Drive.Set(0.0);
  m_FR_Drive.Set(0.0);
  m_BL_Drive.Set(0.0);
  m_BR_Drive.Set(0.0);
}

void Drivetrain::TestRotateModules(double speed) {
  ctre::phoenix6::controls::DutyCycleOut duty{speed};
  m_FL_Steer.Set(0.0);  // Swerve 3 disabled
  m_FR_Steer.SetControl(duty);
  m_BL_Steer.SetControl(duty);
  m_BR_Steer.SetControl(duty);
  m_FL_Drive.Set(0.0);
  m_FR_Drive.Set(0.0);
  m_BL_Drive.Set(0.0);
  m_BR_Drive.Set(0.0);
}

void Drivetrain::Update(double x, double y, double x2, double GyroValue,
                        double triggerL, double triggerR, bool FieldCentric) {
  // Always keep Swerve 3 (FL: IDs 30 & 31) disabled/coasting
  m_FL_Drive.Set(0.0);
  m_FL_Steer.Set(0.0);

  // Rotation disabled: only forwards/backwards (y) and side-to-side (x)
  (void)x2;
  ROT = 0.0;

  // When stick is centered, stop all drive and steer motors so they never hum or twitch
  if (x == 0.0 && y == 0.0) {
    oldspeedFL = 0; oldspeedFR = 0; oldspeedBL = 0; oldspeedBR = 0;
    m_FL_Drive.Set(0.0);
    m_FR_Drive.Set(0.0);
    m_BL_Drive.Set(0.0);
    m_BR_Drive.Set(0.0);
    m_FR_Steer.Set(0.0);
    m_BL_Steer.Set(0.0);
    m_BR_Steer.Set(0.0);
    return;
  }

  // Get zeroed wheel angles in radians for Swerve 2 (FR), Swerve 4 (BL), and Swerve 1 (BR)
  double FR_pos = std::remainder(m_FR_Steer.GetPosition().GetValueAsDouble() - kSwerve2FRZeroTurns, 1.0) * (2.0 * M_PI);
  double BL_pos = std::remainder(m_BL_Steer.GetPosition().GetValueAsDouble() - kSwerve4BLZeroTurns, 1.0) * (2.0 * M_PI);
  double BR_pos = std::remainder(m_BR_Steer.GetPosition().GetValueAsDouble() - kSwerve1BRZeroTurns, 1.0) * (2.0 * M_PI);

  double STR = x;
  double FWD = y;

  // Field Centric // offsetting vectors by the gyro angle
  if (FieldCentric == true) {
    double temp = FWD * std::cos(GyroValue * M_PI / 180.0) + STR * std::sin(GyroValue * M_PI / 180.0);
    STR = -FWD * std::sin(GyroValue * M_PI / 180.0) + STR * std::cos(GyroValue * M_PI / 180.0);
    FWD = temp;
  }

  // Pure translation: all 3 active wheels (FR, BL, BR) turn to the EXACT SAME angle (no 180-deg flip)
  //   Push straight forward (STR=0, FWD=+1) -> targetAngle = 0 rad (all wheels straight)
  //   Push left/right       (STR=+-1, FWD=0) -> targetAngle = +-pi/2 rad (all wheels 90 deg left/right)
  const double targetAngleRad = std::atan2(STR, FWD);
  const double targetTurnsOffset = targetAngleRad / (2.0 * M_PI);
  const double targetSpeed = std::clamp(std::hypot(STR, FWD), 0.0, 1.0);

  angleFR = targetAngleRad;
  angleBL = targetAngleRad;
  angleBR = targetAngleRad;

  speedFL = 0.0;
  speedFR = targetSpeed;
  speedBL = targetSpeed;
  speedBR = targetSpeed;

  ctre::phoenix6::controls::PositionDutyCycle steerm_speedFR{0_tr};
  ctre::phoenix6::controls::PositionDutyCycle steerm_speedBL{0_tr};
  ctre::phoenix6::controls::PositionDutyCycle steerm_speedBR{0_tr};

  steerm_speedFR.Slot = 0;
  steerm_speedBL.Slot = 0;
  steerm_speedBR.Slot = 0;

  m_FR_Steer.SetControl(steerm_speedFR.WithPosition(units::angle::turn_t{kSwerve2FRZeroTurns + targetTurnsOffset}));
  m_BL_Steer.SetControl(steerm_speedBL.WithPosition(units::angle::turn_t{kSwerve4BLZeroTurns + targetTurnsOffset}));
  m_BR_Steer.SetControl(steerm_speedBR.WithPosition(units::angle::turn_t{kSwerve1BRZeroTurns + targetTurnsOffset}));

  double speedFREr = speedFR - oldspeedFR;
  oldspeedFR += (speedFREr >= 0.07) ? 0.07 : (speedFREr <= -0.07) ? -0.07 : speedFREr;
  double speedBLEr = speedBL - oldspeedBL;
  oldspeedBL += (speedBLEr >= 0.07) ? 0.07 : (speedBLEr <= -0.07) ? -0.07 : speedBLEr;
  double speedBREr = speedBR - oldspeedBR;
  oldspeedBR += (speedBREr >= 0.07) ? 0.07 : (speedBREr <= -0.07) ? -0.07 : speedBREr;

  // Control Driving motors: FL (Swerve 3) is disabled (0.0), BL is negated (-oldspeed), FR and BR are positive
  const double cap = std::clamp(0.5 + 0.4 * triggerR - 0.35 * triggerL, 0.1, 1.0);
  m_FL_Drive.Set(0.0);  // Swerve 3 disabled
  m_FR_Drive.Set(std::clamp(oldspeedFR / 2.0, -cap, cap));
  m_BL_Drive.Set(std::clamp(-oldspeedBL / 2.0, -cap, cap));
  m_BR_Drive.Set(std::clamp(oldspeedBR / 2.0, -cap, cap));

  double wheelSpeedFR = (m_FR_Drive.GetVelocity().GetValueAsDouble() / 6.0375) * (M_PI * 0.0889);
  double wheelSpeedBL = (m_BL_Drive.GetVelocity().GetValueAsDouble() / 6.0375) * (M_PI * 0.0889);
  double wheelSpeedBR = (m_BR_Drive.GetVelocity().GetValueAsDouble() / 6.0375) * (M_PI * 0.0889);

  odometryUpdate(0.0, FR_pos, BL_pos, BR_pos, 0.0, wheelSpeedFR, wheelSpeedBL, wheelSpeedBR, GyroValue);
}

// Function to minimize rotation within +-PI/2
double Drivetrain::MinimizeRotation(double targetAngle, double currentAngle) {
  double error = targetAngle - currentAngle;
  while (error > M_PI) error -= 2.0 * M_PI;
  while (error < -M_PI) error += 2.0 * M_PI;

  if (std::abs(error) > M_PI / 2.0) {
    targetAngle -= M_PI;
  }
  return targetAngle;
}

double Drivetrain::MinimizeRotation(double targetAngleRad, double currentAngleRad, double& speedPercent) {
  double errorRad = std::remainder(targetAngleRad - currentAngleRad, 2.0 * M_PI);
  if (std::abs(errorRad) > M_PI / 2.0) {
    errorRad -= std::copysign(M_PI, errorRad);
    speedPercent = -speedPercent;
  }
  return std::remainder(currentAngleRad + errorRad, 2.0 * M_PI);
}

double Drivetrain::WrapError(double targetAngle, double currentAngle) {
  double error = targetAngle + M_PI - currentAngle;
  while (error > M_PI) error -= 2.0 * M_PI;
  while (error < -M_PI) error += 2.0 * M_PI;
  return error;
}

double Drivetrain::ChimiOptimizeAzimuth(double waRad, double currentTurns, double& ws) {
  const double targetTurns = waRad / (2.0 * M_PI);
  double errorTurns = std::remainder(targetTurns - currentTurns, 1.0);
  if (std::abs(errorTurns) > 0.25) {
    errorTurns -= std::copysign(0.5, errorTurns);
    ws = -ws;
  }
  if (std::abs(errorTurns) < 0.003) {
    errorTurns = 0.0;
  }
  return currentTurns + errorTurns;
}

ModuleStates Drivetrain::InverseKinematics(double FWD, double STR, double ROT) {
  const double A = STR - ROT * (DC::L / DC::R);
  const double B = STR + ROT * (DC::L / DC::R);
  const double C = FWD - ROT * (DC::W / DC::R);
  const double D = FWD + ROT * (DC::W / DC::R);

  ModuleStates s;
  s.wsFR = std::hypot(B, C);  s.waFR = std::atan2(B, C);
  s.wsFL = std::hypot(B, D);  s.waFL = std::atan2(B, D);
  s.wsBL = std::hypot(A, D);  s.waBL = std::atan2(A, D);
  s.wsBR = std::hypot(A, C);  s.waBR = std::atan2(A, C);

  const double maxSpeed = std::max({s.wsFL, s.wsFR, s.wsBL, s.wsBR});
  if (maxSpeed > 1.0) {
    s.wsFL /= maxSpeed; s.wsFR /= maxSpeed; s.wsBL /= maxSpeed; s.wsBR /= maxSpeed;
  }
  return s;
}

double Drivetrain::calcYawStraight(double targetAngle, double currentAngle, double kP) {
  return std::remainder(targetAngle - currentAngle, 360.0) * kP;
}

void Drivetrain::odometryUpdate(double angleFL, double angleFR, double angleBL, double angleBR,
                                double wheelSpeedFL, double wheelSpeedFR, double wheelSpeedBL,
                                double wheelSpeedBR, double GyroValue) {
  (void)angleFL;
  (void)wheelSpeedFL;
  units::time::second_t odoCurrentTime = frc::Timer::GetFPGATimestamp();
  odoDeltaTime = (odoCurrentTime - odoLastTime).value();
  odoLastTime = odoCurrentTime;

  double B_FR = std::sin(angleFR) * wheelSpeedFR;
  double A_BL = std::sin(angleBL) * wheelSpeedBL;
  double A_BR = std::sin(angleBR) * wheelSpeedBR;

  double C_FR = std::cos(angleFR) * wheelSpeedFR;
  double D_BL = std::cos(angleBL) * wheelSpeedBL;
  double C_BR = std::cos(angleBR) * wheelSpeedBR;

  double A = (A_BL + A_BR) / 2.0;
  double B = B_FR;
  double C = (C_FR + C_BR) / 2.0;
  double D = D_BL;

  odoROT = GyroValue * M_PI / 180.0;

  double FWD1 = odoROT * (L / 2.0) + A;
  double FWD2 = -odoROT * (L / 2.0) + B;
  odoFWD = (FWD1 + FWD2) / 2.0;

  double STR1 = odoROT * (W / 2.0) + C;
  double STR2 = -odoROT * (W / 2.0) + D;
  odoSTR = (STR1 + STR2) / 2.0;

  double temp = odoFWD * std::cos(GyroValue * M_PI / 180.0) + odoSTR * std::sin(GyroValue * M_PI / 180.0);
  odoSTR = odoSTR * std::cos(GyroValue * M_PI / 180.0) - odoFWD * std::sin(GyroValue * M_PI / 180.0);
  odoFWD = temp;

  if (frc::DriverStation::IsEnabled()) {
    positionFWDField -= units::length::meter_t(odoFWD * odoDeltaTime);
    positionSTRField += units::length::meter_t(odoSTR * odoDeltaTime);
    ROTField = frc::Rotation2d(units::angle::radian_t(odoROT));
  } else {
    odoFWD = 0; odoSTR = 0; odoROT = 0; odoDeltaTime = 0;
    odoLastTime = units::time::second_t(0);
    positionFWDField = 0_m; positionSTRField = 0_m;
    ROTField = frc::Rotation2d(0_rad);
  }

  frc::SmartDashboard::PutNumber("positionFWDField", positionFWDField.value());
  frc::SmartDashboard::PutNumber("positionSTRField", positionSTRField.value());
  frc::SmartDashboard::PutNumber("ROTField_deg", ROTField.Degrees().value());
}

void Drivetrain::UpdateSim(units::time::second_t dt) {
  constexpr double kMaxRps = 100.0;
  auto simDrive = [&](hardware::TalonFX& motor, double cmd) {
    auto& sim = motor.GetSimState();
    const double rps = cmd * kMaxRps;
    sim.SetRotorVelocity(units::turns_per_second_t{rps});
    sim.AddRotorPosition(units::turn_t{rps * dt.value()});
  };
  simDrive(m_FL_Drive, 0.0);
  simDrive(m_FR_Drive, oldspeedFR / 2.0);
  simDrive(m_BL_Drive, -oldspeedBL / 2.0);
  simDrive(m_BR_Drive, oldspeedBR / 2.0);

  const double tFR = (angleFR / M_PI) / 2.0;
  const double tBL = (angleBL / M_PI) / 2.0;
  const double tBR = (angleBR / M_PI) / 2.0;

  m_FR_Steer.GetSimState().SetRawRotorPosition(units::turn_t{tFR * DC::rotorToSensorRatio});
  m_BL_Steer.GetSimState().SetRawRotorPosition(units::turn_t{tBL * DC::rotorToSensorRatio});
  m_BR_Steer.GetSimState().SetRawRotorPosition(units::turn_t{tBR * DC::rotorToSensorRatio});

  CANcoderFR.GetSimState().SetRawPosition(units::turn_t{tFR});
  CANcoderBL.GetSimState().SetRawPosition(units::turn_t{tBL});
  CANcoderBR.GetSimState().SetRawPosition(units::turn_t{tBR});
}
