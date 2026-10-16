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

  // Enable continuous wrap [-180 deg, +180 deg] on roboRIO turning PID controllers (Jagwires7443/Swerve)
  m_turningPID_FL.EnableContinuousInput(-180.0_deg, 180.0_deg);
  m_turningPID_FR.EnableContinuousInput(-180.0_deg, 180.0_deg);
  m_turningPID_BL.EnableContinuousInput(-180.0_deg, 180.0_deg);
  m_turningPID_BR.EnableContinuousInput(-180.0_deg, 180.0_deg);

  ConfigureMotors();
}

double Drivetrain::GetFLCANcoderTurns() {
  if (CANcoderFL.IsConnected()) {
    return CANcoderFL.GetPosition().GetValueAsDouble();
  }
  static hardware::CANcoder rioCANcoder32{DC::kFLEncoderId, CANBus{DC::kCanBus}};
  if (rioCANcoder32.IsConnected()) {
    return rioCANcoder32.GetPosition().GetValueAsDouble();
  }
  return CANcoderFL.GetPosition().GetValueAsDouble();
}

double Drivetrain::GetModuleTurns(hardware::CANcoder& encoder,
                                  hardware::TalonFX& motor,
                                  bool isFL) {
  if (isFL) {
    return GetFLCANcoderTurns();
  }
  if (encoder.IsConnected()) {
    return encoder.GetPosition().GetValueAsDouble();
  }
  return motor.GetPosition().GetValueAsDouble();
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

  // Steer Krakens: Brake mode so wheels stop cleanly when Set(0.0) is commanded inside the 0.05-turn deadband.
  // kD = 0.0 (no derivative kick on 100Hz CANcoder frames, matching Jagwires7443/Swerve).
  auto configureSteerMotor = [&](hardware::TalonFX& motor,
                                 hardware::CANcoder& encoder,
                                 bool useRemoteCANcoder) {
    configs::TalonFXConfiguration cfg{};
    cfg.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
    cfg.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.02_s;
    cfg.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.02_s;
    cfg.ClosedLoopGeneral.ContinuousWrap = true;
    cfg.CurrentLimits.StatorCurrentLimit = 40.0_A;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;

    if (useRemoteCANcoder) {
      cfg.Feedback.FeedbackRemoteSensorID = encoder.GetDeviceID();
      cfg.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::RemoteCANcoder;
      cfg.Feedback.SensorToMechanismRatio = 1.0;
      cfg.Feedback.RotorToSensorRatio = 18.0;
    } else {
      cfg.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::RotorSensor;
      cfg.Feedback.SensorToMechanismRatio = 18.0;
    }

    // Zero kD (kD = 0.0) to eliminate 100Hz RemoteCANcoder derivative chatter
    cfg.Slot0.kP = 1.8;
    cfg.Slot0.kI = 0.0;
    cfg.Slot0.kD = 0.0;

    motor.GetConfigurator().Apply(cfg);
  };

  // FR, BL, BR are on "rio" along with their CANcoders
  configureSteerMotor(m_FR_Steer, CANcoderFR, true);
  configureSteerMotor(m_BL_Steer, CANcoderBL, true);
  configureSteerMotor(m_BR_Steer, CANcoderBR, true);

  // FL steer (on "rio") is synced to CANcoder 32 (on "intakebus")
  configureSteerMotor(m_FL_Steer, CANcoderFL, false);

  // Seed FL steer position and reset all 4 roboRIO ProfiledPIDControllers to current CANcoder angles
  const double turnsFL = GetModuleTurns(CANcoderFL, m_FL_Steer, true);
  const double turnsFR = GetModuleTurns(CANcoderFR, m_FR_Steer, false);
  const double turnsBL = GetModuleTurns(CANcoderBL, m_BL_Steer, false);
  const double turnsBR = GetModuleTurns(CANcoderBR, m_BR_Steer, false);

  m_FL_Steer.SetPosition(units::angle::turn_t{turnsFL});

  m_turningPID_FL.Reset(units::degree_t{std::remainder(turnsFL, 1.0) * 360.0});
  m_turningPID_FR.Reset(units::degree_t{std::remainder(turnsFR, 1.0) * 360.0});
  m_turningPID_BL.Reset(units::degree_t{std::remainder(turnsBL, 1.0) * 360.0});
  m_turningPID_BR.Reset(units::degree_t{std::remainder(turnsBR, 1.0) * 360.0});
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
  const double turnsFL = GetModuleTurns(CANcoderFL, m_FL_Steer, true);
  const double turnsFR = GetModuleTurns(CANcoderFR, m_FR_Steer, false);
  const double turnsBL = GetModuleTurns(CANcoderBL, m_BL_Steer, false);
  const double turnsBR = GetModuleTurns(CANcoderBR, m_BR_Steer, false);

  // Sync FL steer position to CANcoder 32 and keep roboRIO ProfiledPIDControllers synced while disabled
  m_FL_Steer.SetPosition(units::angle::turn_t{turnsFL});
  m_turningPID_FL.Reset(units::degree_t{std::remainder(turnsFL, 1.0) * 360.0});
  m_turningPID_FR.Reset(units::degree_t{std::remainder(turnsFR, 1.0) * 360.0});
  m_turningPID_BL.Reset(units::degree_t{std::remainder(turnsBL, 1.0) * 360.0});
  m_turningPID_BR.Reset(units::degree_t{std::remainder(turnsBR, 1.0) * 360.0});

  frc::SmartDashboard::PutNumber("Swerve/FL_ActualTurns", turnsFL);
  frc::SmartDashboard::PutNumber("Swerve/FR_ActualTurns", turnsFR);
  frc::SmartDashboard::PutNumber("Swerve/BL_ActualTurns", turnsBL);
  frc::SmartDashboard::PutNumber("Swerve/BR_ActualTurns", turnsBR);

  frc::SmartDashboard::PutNumber("Swerve/FL_CANcoder", turnsFL);
  frc::SmartDashboard::PutNumber("Swerve/FR_CANcoder", CANcoderFR.GetPosition().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Swerve/BL_CANcoder", CANcoderBL.GetPosition().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Swerve/BR_CANcoder", CANcoderBR.GetPosition().GetValueAsDouble());
}

void Drivetrain::TestZeroAzimuth() {
  auto steerToZero = [&](hardware::TalonFX& motor,
                         hardware::CANcoder& encoder,
                         frc::ProfiledPIDController<units::angle::degrees>& pid,
                         bool isFL) {
    const double turns = GetModuleTurns(encoder, motor, isFL);
    const units::degree_t currentDeg{std::remainder(turns, 1.0) * 360.0};
    const double errorTurns = std::remainder(-turns, 1.0);
    if (std::abs(errorTurns) <= DC::kSteerDeadbandTurns) {
      pid.Reset(currentDeg);
      motor.Set(0.0);
    } else {
      const double out = std::clamp(pid.Calculate(currentDeg, 0.0_deg),
                                    -DC::kSteerPeakOutput, DC::kSteerPeakOutput);
      motor.Set(out);
    }
  };
  steerToZero(m_FL_Steer, CANcoderFL, m_turningPID_FL, true);
  steerToZero(m_FR_Steer, CANcoderFR, m_turningPID_FR, false);
  steerToZero(m_BL_Steer, CANcoderBL, m_turningPID_BL, false);
  steerToZero(m_BR_Steer, CANcoderBR, m_turningPID_BR, false);

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
// Main Swerve Update (Jagwires7443/Swerve roboRIO ProfiledPIDController + WPILib Kinematics)
// ============================================================================
void Drivetrain::Update(double x, double y, double x2, double GyroValue,
                        double triggerL, double triggerR, bool FieldCentric) {
  // Read current absolute module angles directly from CANcoders (wrapped to [-180 deg, +180 deg])
  const double turnsFL = GetModuleTurns(CANcoderFL, m_FL_Steer, true);
  const double turnsFR = GetModuleTurns(CANcoderFR, m_FR_Steer, false);
  const double turnsBL = GetModuleTurns(CANcoderBL, m_BL_Steer, false);
  const double turnsBR = GetModuleTurns(CANcoderBR, m_BR_Steer, false);

  const units::degree_t currentDegFL{std::remainder(turnsFL, 1.0) * 360.0};
  const units::degree_t currentDegFR{std::remainder(turnsFR, 1.0) * 360.0};
  const units::degree_t currentDegBL{std::remainder(turnsBL, 1.0) * 360.0};
  const units::degree_t currentDegBR{std::remainder(turnsBR, 1.0) * 360.0};

  // 1. Deadzone check: When sticks are centered, explicitly zero BOTH drive and steer motors
  // and keep ProfiledPIDControllers synced to current module angles (Jagwires7443/Swerve).
  if (std::abs(x) < 0.08 && std::abs(y) < 0.08 && std::abs(x2) < 0.08) {
    m_FL_Drive.Set(0.0);
    m_FR_Drive.Set(0.0);
    m_BL_Drive.Set(0.0);
    m_BR_Drive.Set(0.0);
    m_FL_Steer.Set(0.0);
    m_FR_Steer.Set(0.0);
    m_BL_Steer.Set(0.0);
    m_BR_Steer.Set(0.0);
    m_turningPID_FL.Reset(currentDegFL);
    m_turningPID_FR.Reset(currentDegFR);
    m_turningPID_BL.Reset(currentDegBL);
    m_turningPID_BR.Reset(currentDegBR);
    oldspeedFL = 0; oldspeedFR = 0; oldspeedBL = 0; oldspeedBR = 0;
    return;
  }

  // 2. Speed scaling with triggers (base 50%, R2 boosts up to 90%, L2 slows to 15%)
  const double speedScale = std::clamp(0.5 + 0.4 * triggerR - 0.35 * triggerL, 0.15, 1.0);
  const auto kMaxSpeed = 4.0_mps * speedScale;
  const auto kMaxAngularSpeed = units::radians_per_second_t{2.0 * M_PI * speedScale};

  // 3. WPILib ChassisSpeeds (NWU coordinate frame: +X forward, +Y left, +omega CCW)
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

  // 5. WPILib SwerveModuleState::Optimize using direct CANcoder angles (never rotate > 90 deg)
  const frc::Rotation2d currentAngleFL{currentDegFL};
  const frc::Rotation2d currentAngleFR{currentDegFR};
  const frc::Rotation2d currentAngleBL{currentDegBL};
  const frc::Rotation2d currentAngleBR{currentDegBR};

  stateFL.Optimize(currentAngleFL);
  stateFR.Optimize(currentAngleFR);
  stateBL.Optimize(currentAngleBL);
  stateBR.Optimize(currentAngleBR);

  // 6. Steer control via roboRIO ProfiledPIDController (Jagwires7443/Swerve)
  //    with explicit Set(0.0) cutoff inside DC::kSteerDeadbandTurns (0.05 turns = 18 deg)
  auto steerModule = [&](hardware::TalonFX& motor,
                         frc::ProfiledPIDController<units::angle::degrees>& pid,
                         const frc::SwerveModuleState& targetState,
                         units::degree_t currentDeg) -> double {
    const units::degree_t targetDeg = targetState.angle.Degrees();
    const double errorDeg = std::remainder((targetDeg - currentDeg).value(), 360.0);
    const double errorTurns = errorDeg / 360.0;

    // Inside the 0.05-turn (18 deg) deadband: explicitly stop the steer motor (Set(0.0))
    // so the TalonFX cannot hunt or oscillate back-and-forth around the setpoint.
    if (std::abs(errorTurns) <= DC::kSteerDeadbandTurns) {
      pid.Reset(currentDeg);
      motor.Set(0.0);
      return std::abs(errorDeg);
    }

    const double out = std::clamp(pid.Calculate(currentDeg, targetDeg),
                                  -DC::kSteerPeakOutput, DC::kSteerPeakOutput);
    motor.Set(out);
    return std::abs(errorDeg);
  };

  const double errDegFL = steerModule(m_FL_Steer, m_turningPID_FL, stateFL, currentDegFL);
  const double errDegFR = steerModule(m_FR_Steer, m_turningPID_FR, stateFR, currentDegFR);
  const double errDegBL = steerModule(m_BL_Steer, m_turningPID_BL, stateBL, currentDegBL);
  const double errDegBR = steerModule(m_BR_Steer, m_turningPID_BR, stateBR, currentDegBR);

  // 7. Drive velocity vector alignment (std::cos(angleError)) & turning-error power reduction
  //    (modeled after Jagwires7443/Swerve SwerveModule::SetDriveVelocity & DriveSubsystem::SetModuleStates)
  stateFL.CosineScale(currentAngleFL);
  stateFR.CosineScale(currentAngleFR);
  stateBL.CosineScale(currentAngleBL);
  stateBR.CosineScale(currentAngleBR);

  const double totalTurningError = errDegFL + errDegFR + errDegBL + errDegBR;
  const double alignScale = (totalTurningError > 40.0) ? 0.5 : 1.0;

  // 8. Apply drive speeds (Left side inverted, right side positive on this chassis)
  const double outFL = (stateFL.speed / kMaxSpeed).value() * alignScale;
  const double outFR = (stateFR.speed / kMaxSpeed).value() * alignScale;
  const double outBL = (stateBL.speed / kMaxSpeed).value() * alignScale;
  const double outBR = (stateBR.speed / kMaxSpeed).value() * alignScale;

  m_FL_Drive.Set(std::clamp(-outFL, -1.0, 1.0));
  m_FR_Drive.Set(std::clamp(outFR, -1.0, 1.0));
  m_BL_Drive.Set(std::clamp(-outBL, -1.0, 1.0));
  m_BR_Drive.Set(std::clamp(outBR, -1.0, 1.0));

  // Update internal state variables for simulation & telemetry
  angleFL = stateFL.angle.Radians().value();
  angleFR = stateFR.angle.Radians().value();
  angleBL = stateBL.angle.Radians().value();
  angleBR = stateBR.angle.Radians().value();
  oldspeedFL = outFL * 2.0;
  oldspeedFR = outFR * 2.0;
  oldspeedBL = outBL * 2.0;
  oldspeedBR = outBR * 2.0;

  // 9. Odometry update
  auto getModPos = [&](double moduleTurns, hardware::TalonFX& drive) {
    const double wheelRotations = drive.GetPosition().GetValueAsDouble() / 6.0375;
    const units::meter_t dist{wheelRotations * (M_PI * 0.0889)};
    return frc::SwerveModulePosition{dist, frc::Rotation2d{units::turn_t{moduleTurns}}};
  };

  m_odometry.Update(frc::Rotation2d{units::degree_t{-GyroValue}},
                    {getModPos(turnsFL, m_FL_Drive),
                     getModPos(turnsFR, m_FR_Drive),
                     getModPos(turnsBL, m_BL_Drive),
                     getModPos(turnsBR, m_BR_Drive)});

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
