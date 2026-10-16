// ============================================================================
// FRC Team 8753 - 2026 Swerve Drivetrain Implementation (swerve-base / 2026_Bot)
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

void Drivetrain::ConfigureMotors() {
  // Drive Krakens configurations
  driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = units::time::second_t(0.5);
  driveConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

  m_FL_Drive.GetConfigurator().Apply(driveConfig);
  m_FR_Drive.GetConfigurator().Apply(driveConfig);
  m_BL_Drive.GetConfigurator().Apply(driveConfig);
  m_BR_Drive.GetConfigurator().Apply(driveConfig);

  // Steer Krakens configurations // fusing CANcoder to Kraken encoder for higher accuracy
  configFL.Feedback.FeedbackRemoteSensorID = CANcoderFL.GetDeviceID();
  configFL.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
  configFL.Feedback.SensorToMechanismRatio = 1.0;
  configFL.Feedback.RotorToSensorRatio = 18;
  configFL.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = units::time::second_t(0.01);
  configFL.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
  configFL.ClosedLoopGeneral.ContinuousWrap = true;

  configFR.Feedback.FeedbackRemoteSensorID = CANcoderFR.GetDeviceID();
  configFR.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
  configFR.Feedback.SensorToMechanismRatio = 1.0;
  configFR.Feedback.RotorToSensorRatio = 18;
  configFR.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = units::time::second_t(0.01);
  configFR.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
  configFR.ClosedLoopGeneral.ContinuousWrap = true;

  configBL.Feedback.FeedbackRemoteSensorID = CANcoderBL.GetDeviceID();
  configBL.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
  configBL.Feedback.SensorToMechanismRatio = 1.0;
  configBL.Feedback.RotorToSensorRatio = 18;
  configBL.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = units::time::second_t(0.01);
  configBL.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
  configBL.ClosedLoopGeneral.ContinuousWrap = true;

  configBR.Feedback.FeedbackRemoteSensorID = CANcoderBR.GetDeviceID();
  configBR.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
  configBR.Feedback.SensorToMechanismRatio = 1.0;
  configBR.Feedback.RotorToSensorRatio = 18;
  configBR.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = units::time::second_t(0.01);
  configBR.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
  configBR.ClosedLoopGeneral.ContinuousWrap = true;

  Posconfig.kP = ModuleP;
  Posconfig.kI = ModuleI;
  Posconfig.kD = ModuleD;

  m_FL_Steer.GetConfigurator().Apply(configFL);
  m_FL_Steer.GetConfigurator().Apply(Posconfig);
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
  oldspeedFL = 0; oldspeedFR = 0; oldspeedBL = 0; oldspeedBR = 0;
}

void Drivetrain::DisabledPeriodic() {
  frc::SmartDashboard::PutNumber("Swerve/FL_ActualTurns", m_FL_Steer.GetPosition().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Swerve/FR_ActualTurns", m_FR_Steer.GetPosition().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Swerve/BL_ActualTurns", m_BL_Steer.GetPosition().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Swerve/BR_ActualTurns", m_BR_Steer.GetPosition().GetValueAsDouble());
}

void Drivetrain::TestZeroAzimuth() {
  ctre::phoenix6::controls::PositionDutyCycle steerPos{0_tr};
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
  ctre::phoenix6::controls::DutyCycleOut duty{speed};
  m_FL_Steer.SetControl(duty);
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
  // Get wheel angle with fused motor encoder and CANcoder
  double FL_pos = std::fmod(m_FL_Steer.GetPosition().GetValueAsDouble(), 1.0) * (2.0 * M_PI);
  double FR_pos = std::fmod(m_FR_Steer.GetPosition().GetValueAsDouble(), 1.0) * (2.0 * M_PI);
  double BL_pos = std::fmod(m_BL_Steer.GetPosition().GetValueAsDouble(), 1.0) * (2.0 * M_PI);
  double BR_pos = std::fmod(m_BR_Steer.GetPosition().GetValueAsDouble(), 1.0) * (2.0 * M_PI);

  units::time::second_t currentTime = frc::Timer::GetFPGATimestamp();
  units::time::second_t deltaTime = currentTime - lastTime;
  lastTime = currentTime;

  double STR = x;
  double FWD = y;

  // Field Centric // offsetting vectors by the gyro angle
  if (FieldCentric == true) {
    double temp = FWD * std::cos(GyroValue * M_PI / 180.0) + STR * std::sin(GyroValue * M_PI / 180.0);
    STR = -FWD * std::sin(GyroValue * M_PI / 180.0) + STR * std::cos(GyroValue * M_PI / 180.0);
    FWD = temp;
  } else {
    STR = x;
    FWD = y;
  }

  // PID controller for keeping robot straight
  double intDeltaTime = deltaTime.value();
  if (intDeltaTime <= 0.0 || intDeltaTime > 0.1) intDeltaTime = 0.02;
  double error = std::remainder(targetAngle - GyroValue, 360.0);
  iError += error * intDeltaTime;
  double dError = (error - lastError) / intDeltaTime;

  double output = straightP * error + straightI * iError + dError * straightD;
  output = std::clamp(output, -0.7, 0.7);
  lastError = error;

  if (x2 != 0.0) {
    targetAngle = GyroValue;
    ROT = x2;
  } else {
    ROT = output;
  }

  // Calculate variables A, B, C, D
  double A = (STR - ROT * L / 2.0);
  double B = (STR + ROT * L / 2.0);
  double C = (FWD - ROT * W / 2.0);
  double D = (FWD + ROT * W / 2.0);

  // Calculate wheel speeds
  if (x != 0.0 || y != 0.0 || x2 != 0.0) {
    speedFL = std::sqrt((B * B) + (D * D));
    speedFR = std::sqrt((B * B) + (C * C));
    speedBL = std::sqrt((A * A) + (D * D));
    speedBR = std::sqrt((A * A) + (C * C));
  } else {
    speedFL = 0.0;
    speedFR = 0.0;
    speedBL = 0.0;
    speedBR = 0.0;
  }

  std::vector<double> wheelSpeeds = {speedFL, speedFR, speedBL, speedBR};
  double maxSpeed = *std::max_element(wheelSpeeds.begin(), wheelSpeeds.end());
  if (maxSpeed >= 1.0) {
    speedFL /= maxSpeed;
    speedFR /= maxSpeed;
    speedBL /= maxSpeed;
    speedBR /= maxSpeed;
  }

  // Calculate final angle for each wheel
  if (x != 0.0 || y != 0.0 || x2 != 0.0) {
    tempangleFL = std::atan2(B, D);
    tempangleFR = std::atan2(B, C);
    tempangleBL = std::atan2(A, D);
    tempangleBR = std::atan2(A, C);
  }

  // Adjust angles to ensure minimal rotation
  angleFL = MinimizeRotation(tempangleFL, FL_pos);
  angleFR = MinimizeRotation(tempangleFR, FR_pos);
  angleBL = MinimizeRotation(tempangleBL, BL_pos);
  angleBR = MinimizeRotation(tempangleBR, BR_pos);

  double errorFL = WrapError(tempangleFL, FL_pos);
  double errorFR = WrapError(tempangleFR, FR_pos);
  double errorBL = WrapError(tempangleBL, BL_pos);
  double errorBR = WrapError(tempangleBR, BR_pos);

  speedFL *= std::cos(errorFL);
  speedFR *= std::cos(errorFR);
  speedBL *= std::cos(errorBL);
  speedBR *= std::cos(errorBR);

  ctre::phoenix6::controls::PositionDutyCycle steerm_speedFL{0_tr};
  ctre::phoenix6::controls::PositionDutyCycle steerm_speedFR{0_tr};
  ctre::phoenix6::controls::PositionDutyCycle steerm_speedBL{0_tr};
  ctre::phoenix6::controls::PositionDutyCycle steerm_speedBR{0_tr};

  steerm_speedFL.Slot = 0;
  steerm_speedFR.Slot = 0;
  steerm_speedBL.Slot = 0;
  steerm_speedBR.Slot = 0;

  m_FL_Steer.SetControl(steerm_speedFL.WithPosition(units::angle::turn_t((angleFL / M_PI) / 2.0)));
  m_FR_Steer.SetControl(steerm_speedFR.WithPosition(units::angle::turn_t((angleFR / M_PI) / 2.0)));
  m_BL_Steer.SetControl(steerm_speedBL.WithPosition(units::angle::turn_t((angleBL / M_PI) / 2.0)));
  m_BR_Steer.SetControl(steerm_speedBR.WithPosition(units::angle::turn_t((angleBR / M_PI) / 2.0)));

  double speedFLEr = speedFL - oldspeedFL;
  oldspeedFL += (speedFLEr >= 0.07) ? 0.07 : (speedFLEr <= -0.07) ? -0.07 : speedFLEr;
  double speedFREr = speedFR - oldspeedFR;
  oldspeedFR += (speedFREr >= 0.07) ? 0.07 : (speedFREr <= -0.07) ? -0.07 : speedFREr;
  double speedBLEr = speedBL - oldspeedBL;
  oldspeedBL += (speedBLEr >= 0.07) ? 0.07 : (speedBLEr <= -0.07) ? -0.07 : speedBLEr;
  double speedBREr = speedBR - oldspeedBR;
  oldspeedBR += (speedBREr >= 0.07) ? 0.07 : (speedBREr <= -0.07) ? -0.07 : speedBREr;

  // Control Driving motors: FL and BL are negated (-oldspeed), FR and BR are positive
  if (std::isnan(speedFL) || (x == 0.0 && y == 0.0 && x2 == 0.0)) {
    oldspeedFL = 0; oldspeedFR = 0; oldspeedBL = 0; oldspeedBR = 0;
    m_FL_Drive.Set(0.0);
    m_FR_Drive.Set(0.0);
    m_BL_Drive.Set(0.0);
    m_BR_Drive.Set(0.0);
  } else {
    // dividing by 2 for a max base speed of 50%; triggers add boost or subtract for precision
    const double cap = std::clamp(0.5 + 0.4 * triggerR - 0.35 * triggerL, 0.1, 1.0);
    m_FL_Drive.Set(std::clamp(-oldspeedFL / 2.0, -cap, cap));
    m_FR_Drive.Set(std::clamp(oldspeedFR / 2.0, -cap, cap));
    m_BL_Drive.Set(std::clamp(-oldspeedBL / 2.0, -cap, cap));
    m_BR_Drive.Set(std::clamp(oldspeedBR / 2.0, -cap, cap));
  }

  double wheelSpeedFL = (m_FL_Drive.GetVelocity().GetValueAsDouble() / 6.0375) * (M_PI * 0.0889);
  double wheelSpeedFR = (m_FR_Drive.GetVelocity().GetValueAsDouble() / 6.0375) * (M_PI * 0.0889);
  double wheelSpeedBL = (m_BL_Drive.GetVelocity().GetValueAsDouble() / 6.0375) * (M_PI * 0.0889);
  double wheelSpeedBR = (m_BR_Drive.GetVelocity().GetValueAsDouble() / 6.0375) * (M_PI * 0.0889);

  odometryUpdate(FL_pos, FR_pos, BL_pos, BR_pos, wheelSpeedFL, wheelSpeedFR, wheelSpeedBL, wheelSpeedBR, GyroValue);
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
  if (std::abs(errorTurns) < DC::kSteerDeadbandTurns) {
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
