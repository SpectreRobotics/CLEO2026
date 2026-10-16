// ============================================================================
// FRC Team 8753 - 2026 Swerve Drivetrain (Official WPILib Swerve Kinematics)
// ============================================================================

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <algorithm>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/time.h>
#include <units/velocity.h>

#include "Drivetrain.h"
#include "DrivetrainConstants.h"

using namespace ctre::phoenix6;
using namespace units::literals;
namespace DC = DrivetrainConstants;

Drivetrain::Drivetrain() {
  positionFWDField = units::length::meter_t(0);
  positionSTRField = units::length::meter_t(0);

  ConfigureMotors();
}

double Drivetrain::GetFLCANcoderTurns() {
  if (CANcoderFL.IsConnected()) {
    return CANcoderFL.GetPosition().GetValueAsDouble();
  }
  static hardware::CANcoder rioCANcoder32{DC::kFLEncoderId, CANBus{DC::kCanBus}};
  return rioCANcoder32.GetPosition().GetValueAsDouble();
}

void Drivetrain::ConfigureMotors() {
  // Drive Krakens: Brake mode, voltage ramp, stator current limit
  driveConfig.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
  driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.25_s;
  driveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.15_s;
  driveConfig.CurrentLimits.StatorCurrentLimit = 60.0_A;
  driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;

  m_FL_Drive.GetConfigurator().Apply(driveConfig);
  m_FR_Drive.GetConfigurator().Apply(driveConfig);
  m_BL_Drive.GetConfigurator().Apply(driveConfig);
  m_BR_Drive.GetConfigurator().Apply(driveConfig);

  // Steer Krakens: Coast mode, continuous wrap, and PID with derivative damping
  auto configureSteerMotor = [&](hardware::TalonFX& motor,
                                 hardware::CANcoder& encoder,
                                 bool useRemoteCANcoder) {
    configs::TalonFXConfiguration cfg{};
    cfg.MotorOutput.NeutralMode = signals::NeutralModeValue::Coast;
    cfg.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.01_s;
    cfg.ClosedLoopGeneral.ContinuousWrap = true;
    cfg.CurrentLimits.StatorCurrentLimit = 40.0_A;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;

    if (useRemoteCANcoder) {
      cfg.Feedback.FeedbackRemoteSensorID = encoder.GetDeviceID();
      cfg.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::RemoteCANcoder;
      cfg.Feedback.SensorToMechanismRatio = 1.0;
      cfg.Feedback.RotorToSensorRatio = 18.0;
    } else {
      // Use internal rotor sensor geared 18:1
      cfg.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::RotorSensor;
      cfg.Feedback.SensorToMechanismRatio = 18.0;
    }

    // Steer PID: kP = 1.5, kD = 0.05 for critical damping (stops high-frequency chatter/vibration)
    cfg.Slot0.kP = 1.5;
    cfg.Slot0.kI = 0.0;
    cfg.Slot0.kD = 0.05;

    motor.GetConfigurator().Apply(cfg);
  };

  // FR, BL, BR are on "rio" along with their CANcoders
  configureSteerMotor(m_FR_Steer, CANcoderFR, true);
  configureSteerMotor(m_BL_Steer, CANcoderBL, true);
  configureSteerMotor(m_BR_Steer, CANcoderBR, true);

  // FL steer (on "rio") is synced to CANcoder 32 (on "intakebus")
  configureSteerMotor(m_FL_Steer, CANcoderFL, false);

  // Seed FL steer position from CANcoder 32 absolute turns
  m_FL_Steer.SetPosition(units::angle::turn_t{GetFLCANcoderTurns()});
}

void Drivetrain::DisabledInit() {
  m_FL_Drive.Set(0.0);
  m_FR_Drive.Set(0.0);
  m_BL_Drive.Set(0.0);
  m_BR_Drive.Set(0.0);
  oldspeedFL = 0; oldspeedFR = 0; oldspeedBL = 0; oldspeedBR = 0;
}

void Drivetrain::DisabledPeriodic() {
  // Sync FL steer position to CANcoder 32 while disabled
  m_FL_Steer.SetPosition(units::angle::turn_t{GetFLCANcoderTurns()});

  frc::SmartDashboard::PutNumber("Swerve/FL_ActualTurns", m_FL_Steer.GetPosition().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Swerve/FR_ActualTurns", m_FR_Steer.GetPosition().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Swerve/BL_ActualTurns", m_BL_Steer.GetPosition().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Swerve/BR_ActualTurns", m_BR_Steer.GetPosition().GetValueAsDouble());

  frc::SmartDashboard::PutNumber("Swerve/FL_CANcoder", GetFLCANcoderTurns());
  frc::SmartDashboard::PutNumber("Swerve/FR_CANcoder", CANcoderFR.GetPosition().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Swerve/BL_CANcoder", CANcoderBL.GetPosition().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Swerve/BR_CANcoder", CANcoderBR.GetPosition().GetValueAsDouble());
}

void Drivetrain::TestZeroAzimuth() {
  controls::PositionDutyCycle steerPos{0_tr};
  steerPos.Slot = 0;
  m_FL_Steer.SetControl(steerPos.WithPosition(0_tr));
  m_FR_Steer.SetControl(steerPos.WithPosition(0_tr));
  m_BL_Steer.SetControl(steerPos.WithPosition(0_tr));
  m_BR_Steer.SetControl(steerPos.WithPosition(0_tr));
  m_FL_Drive.Set(0.0);
  m_FR_Drive.Set(0.0);
  m_BL_Drive.Set(0.0);
  m_BR_Drive.Set(0.0);
}

void Drivetrain::TestRotateModules(double speed) {
  controls::DutyCycleOut duty{speed};
  m_FL_Steer.SetControl(duty);
  m_FR_Steer.SetControl(duty);
  m_BL_Steer.SetControl(duty);
  m_BR_Steer.SetControl(duty);
  m_FL_Drive.Set(0.0);
  m_FR_Drive.Set(0.0);
  m_BL_Drive.Set(0.0);
  m_BR_Drive.Set(0.0);
}

// ============================================================================
// Main Swerve Update (WPILib SwerveDriveKinematics + SwerveModuleState Optimize)
// ============================================================================
void Drivetrain::Update(double x, double y, double x2, double GyroValue,
                        double triggerL, double triggerR, bool FieldCentric) {
  // 1. Deadzone check: When sticks are centered, zero drive outputs and HOLD steer
  // This completely eliminates hum, vibration, and motor stalling when stationary.
  if (std::abs(x) < 0.08 && std::abs(y) < 0.08 && std::abs(x2) < 0.08) {
    m_FL_Drive.Set(0.0);
    m_FR_Drive.Set(0.0);
    m_BL_Drive.Set(0.0);
    m_BR_Drive.Set(0.0);
    oldspeedFL = 0; oldspeedFR = 0; oldspeedBL = 0; oldspeedBR = 0;
    return;
  }

  // 2. Speed scaling with triggers (base 50%, R2 boosts up to 90%, L2 slows to 15%)
  const double speedScale = std::clamp(0.5 + 0.4 * triggerR - 0.35 * triggerL, 0.15, 1.0);
  const auto kMaxSpeed = 4.0_mps * speedScale;
  const auto kMaxAngularSpeed = units::radians_per_second_t{2.0 * M_PI * speedScale};

  // 3. WPILib ChassisSpeeds (NWU coordinate frame: +X forward, +Y left, +omega CCW)
  // Left stick Y: push up (+forward) = +vx
  // Left stick X: push right (+strafe right) = -vy (in WPILib, +Y is left)
  // Right stick X: push right (+CW rotate) = -omega (in WPILib, CCW is positive)
  const auto vx = y * kMaxSpeed;
  const auto vy = -x * kMaxSpeed;
  const auto omega = -x2 * kMaxAngularSpeed;

  frc::ChassisSpeeds chassisSpeeds;
  if (FieldCentric) {
    // GyroValue in Robot.cpp is CW-positive degrees; WPILib Rotation2d is CCW-positive
    chassisSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        vx, vy, omega, frc::Rotation2d{units::degree_t{-GyroValue}});
  } else {
    chassisSpeeds = frc::ChassisSpeeds{vx, vy, omega};
  }

  // 4. WPILib Kinematics conversion to 4 module states [FL, FR, BL, BR]
  auto [stateFL, stateFR, stateBL, stateBR] = m_kinematics.ToSwerveModuleStates(chassisSpeeds);
  wpi::array<frc::SwerveModuleState, 4> states{stateFL, stateFR, stateBL, stateBR};
  m_kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);
  stateFL = states[0];
  stateFR = states[1];
  stateBL = states[2];
  stateBR = states[3];

  // 5. Read current steer angles
  auto getAngle = [](hardware::TalonFX& motor) {
    return frc::Rotation2d{units::turn_t{motor.GetPosition().GetValueAsDouble()}};
  };
  frc::Rotation2d currentAngleFL = getAngle(m_FL_Steer);
  frc::Rotation2d currentAngleFR = getAngle(m_FR_Steer);
  frc::Rotation2d currentAngleBL = getAngle(m_BL_Steer);
  frc::Rotation2d currentAngleBR = getAngle(m_BR_Steer);

  // 6. WPILib SwerveModuleState Optimize & CosineScale (the standard WPILib algorithm)
  stateFL.Optimize(currentAngleFL);
  stateFR.Optimize(currentAngleFR);
  stateBL.Optimize(currentAngleBL);
  stateBR.Optimize(currentAngleBR);

  stateFL.CosineScale(currentAngleFL);
  stateFR.CosineScale(currentAngleFR);
  stateBL.CosineScale(currentAngleBL);
  stateBR.CosineScale(currentAngleBR);

  // 7. Apply steer setpoints with encoder deadband (ContinuousWrap handles shortest path)
  controls::PositionDutyCycle steerCmd{0_tr};
  steerCmd.Slot = 0;

  auto applySteerDeadband = [&](hardware::TalonFX& motor,
                                const frc::SwerveModuleState& targetState,
                                const frc::Rotation2d& currentAngle) {
    const double targetTurns = targetState.angle.Degrees() / 360_deg;
    const double currentTurns = currentAngle.Degrees() / 360_deg;
    const double errorTurns = std::remainder(targetTurns - currentTurns, 1.0);
    // Only command steer motor if error exceeds encoder deadband
    if (std::abs(errorTurns) > DC::kSteerDeadbandTurns) {
      motor.SetControl(steerCmd.WithPosition(units::turn_t{targetTurns}));
    }
  };

  applySteerDeadband(m_FL_Steer, stateFL, currentAngleFL);
  applySteerDeadband(m_FR_Steer, stateFR, currentAngleFR);
  applySteerDeadband(m_BL_Steer, stateBL, currentAngleBL);
  applySteerDeadband(m_BR_Steer, stateBR, currentAngleBR);

  // 8. Apply drive speeds (Left side inverted, right side positive on this chassis)
  const double outFL = (stateFL.speed / kMaxSpeed).value();
  const double outFR = (stateFR.speed / kMaxSpeed).value();
  const double outBL = (stateBL.speed / kMaxSpeed).value();
  const double outBR = (stateBR.speed / kMaxSpeed).value();

  m_FL_Drive.Set(std::clamp(-outFL, -1.0, 1.0));
  m_FR_Drive.Set(std::clamp(outFR, -1.0, 1.0));
  m_BL_Drive.Set(std::clamp(-outBL, -1.0, 1.0));
  m_BR_Drive.Set(std::clamp(outBR, -1.0, 1.0));

  // 9. Odometry update
  auto getModPos = [](hardware::TalonFX& steer, hardware::TalonFX& drive) {
    const double wheelRotations = drive.GetPosition().GetValueAsDouble() / 6.0375;
    const units::meter_t dist{wheelRotations * (M_PI * 0.0889)};
    return frc::SwerveModulePosition{dist, frc::Rotation2d{units::turn_t{steer.GetPosition().GetValueAsDouble()}}};
  };

  m_odometry.Update(frc::Rotation2d{units::degree_t{-GyroValue}},
                    {getModPos(m_FL_Steer, m_FL_Drive),
                     getModPos(m_FR_Steer, m_FR_Drive),
                     getModPos(m_BL_Steer, m_BL_Drive),
                     getModPos(m_BR_Steer, m_BR_Drive)});

  positionFWDField = m_odometry.GetPose().X();
  positionSTRField = -m_odometry.GetPose().Y();
  ROTField = m_odometry.GetPose().Rotation();
}

// ============================================================================
// Math & Testing Helpers
// ============================================================================
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
  units::time::second_t odoCurrentTime = frc::Timer::GetFPGATimestamp();
  odoDeltaTime = (odoCurrentTime - odoLastTime).value();
  odoLastTime = odoCurrentTime;

  double B_FL = std::sin(angleFL) * wheelSpeedFL;
  double B_FR = std::sin(angleFR) * wheelSpeedFR;
  double A_BL = std::sin(angleBL) * wheelSpeedBL;
  double A_BR = std::sin(angleBR) * wheelSpeedBR;

  double D_FL = std::cos(angleFL) * wheelSpeedFL;
  double C_FR = std::cos(angleFR) * wheelSpeedFR;
  double D_BL = std::cos(angleBL) * wheelSpeedBL;
  double C_BR = std::cos(angleBR) * wheelSpeedBR;

  double A = (A_BL + A_BR) / 2.0;
  double B = (B_FL + B_FR) / 2.0;
  double C = (C_FR + C_BR) / 2.0;
  double D = (D_FL + D_BL) / 2.0;

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
}

void Drivetrain::UpdateSim(units::time::second_t dt) {
  constexpr double kMaxRps = 100.0;
  auto simDrive = [&](hardware::TalonFX& motor, double cmd) {
    auto& sim = motor.GetSimState();
    const double rps = cmd * kMaxRps;
    sim.SetRotorVelocity(units::turns_per_second_t{rps});
    sim.AddRotorPosition(units::turn_t{rps * dt.value()});
  };
  simDrive(m_FL_Drive, -oldspeedFL / 2.0);
  simDrive(m_FR_Drive, oldspeedFR / 2.0);
  simDrive(m_BL_Drive, -oldspeedBL / 2.0);
  simDrive(m_BR_Drive, oldspeedBR / 2.0);

  const double tFL = (angleFL / M_PI) / 2.0;
  const double tFR = (angleFR / M_PI) / 2.0;
  const double tBL = (angleBL / M_PI) / 2.0;
  const double tBR = (angleBR / M_PI) / 2.0;

  m_FL_Steer.GetSimState().SetRawRotorPosition(units::turn_t{tFL * DC::rotorToSensorRatio});
  m_FR_Steer.GetSimState().SetRawRotorPosition(units::turn_t{tFR * DC::rotorToSensorRatio});
  m_BL_Steer.GetSimState().SetRawRotorPosition(units::turn_t{tBL * DC::rotorToSensorRatio});
  m_BR_Steer.GetSimState().SetRawRotorPosition(units::turn_t{tBR * DC::rotorToSensorRatio});

  CANcoderFL.GetSimState().SetRawPosition(units::turn_t{tFL});
  CANcoderFR.GetSimState().SetRawPosition(units::turn_t{tFR});
  CANcoderBL.GetSimState().SetRawPosition(units::turn_t{tBL});
  CANcoderBR.GetSimState().SetRawPosition(units::turn_t{tBR});
}
