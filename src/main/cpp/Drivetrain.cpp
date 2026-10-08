// ============================================================================
// FRC Team 8753 - 2026 Swerve Drivetrain Implementation
// ============================================================================

#define _USE_MATH_DEFINES
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <algorithm>
#include <vector>

#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <units/angle.h>
#include <units/time.h>
#include <units/angular_velocity.h>

#include "Drivetrain.h"
#include "DrivetrainConstants.h"

using namespace units::literals;

Drivetrain::Drivetrain() {
}

void Drivetrain::ConfigureMotors() {
  // Drive Krakens configurations
  driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
      units::time::second_t(DrivetrainConstants::DriveRampRateSeconds);
  driveConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

  // Steer Krakens configurations // fusing CANcoder to Krakens encoder for high accuracy
  auto setupSteerConfig = [this](ctre::phoenix6::configs::TalonFXConfiguration& cfg, int canCoderId) {
    cfg.Feedback.FeedbackRemoteSensorID = canCoderId;
    cfg.Feedback.FeedbackSensorSource =
        ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
    cfg.Feedback.SensorToMechanismRatio = DrivetrainConstants::sensorToMechanismRatio;
    cfg.Feedback.RotorToSensorRatio = DrivetrainConstants::rotorToSensorRatio;
    cfg.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod =
        units::time::second_t(DrivetrainConstants::SteeringRampRateSeconds);
    cfg.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    cfg.ClosedLoopGeneral.ContinuousWrap = true;
    cfg.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
    cfg.Slot0.kP = DrivetrainConstants::ModuleP;
    cfg.Slot0.kI = DrivetrainConstants::ModuleI;
    cfg.Slot0.kD = DrivetrainConstants::ModuleD;
  };

  setupSteerConfig(configFL, CANcoderFL.GetDeviceID());
  setupSteerConfig(configFR, CANcoderFR.GetDeviceID());
  setupSteerConfig(configBL, CANcoderBL.GetDeviceID());
  setupSteerConfig(configBR, CANcoderBR.GetDeviceID());

  auto applyWithRetry = [](ctre::phoenix6::hardware::TalonFX& motor,
                           const ctre::phoenix6::configs::TalonFXConfiguration& cfg,
                           const char* name) -> bool {
    auto status = motor.GetConfigurator().Apply(cfg, 100_ms);
    for (int attempt = 1; attempt <= 5; ++attempt) {
      if (status.IsOK()) {
        frc::SmartDashboard::PutString(std::string("Swerve/Config_") + name, "OK");
        return true;
      }
      status = motor.GetConfigurator().Apply(cfg, 100_ms);
    }
    frc::SmartDashboard::PutString(std::string("Swerve/Config_") + name, status.GetName());
    return false;
  };

  bool ok = true;
  ok &= applyWithRetry(m_FL_Drive, driveConfig, "FL_Drive");
  ok &= applyWithRetry(m_FR_Drive, driveConfig, "FR_Drive");
  ok &= applyWithRetry(m_BL_Drive, driveConfig, "BL_Drive");
  ok &= applyWithRetry(m_BR_Drive, driveConfig, "BR_Drive");

  ok &= applyWithRetry(m_FL_Steer, configFL, "FL_Steer");
  ok &= applyWithRetry(m_FR_Steer, configFR, "FR_Steer");
  ok &= applyWithRetry(m_BL_Steer, configBL, "BL_Steer");
  ok &= applyWithRetry(m_BR_Steer, configBR, "BR_Steer");

  m_configsApplied = ok;
  frc::SmartDashboard::PutBoolean("Swerve/AllConfigsApplied", m_configsApplied);
}

void Drivetrain::DisabledInit() {
  m_FL_Drive.Set(0.0);
  m_FR_Drive.Set(0.0);
  m_BL_Drive.Set(0.0);
  m_BR_Drive.Set(0.0);
}

void Drivetrain::DisabledPeriodic() {
  if (!m_configsApplied) {
    static int retryCount = 0;
    if (++retryCount % 50 == 0) {  // Retry once every 50 loops (1s) while disabled until all configs succeed
      ConfigureMotors();
    }
  }

  double fl = m_FL_Steer.GetPosition().GetValueAsDouble();
  double fr = m_FR_Steer.GetPosition().GetValueAsDouble();
  double bl = m_BL_Steer.GetPosition().GetValueAsDouble();
  double br = m_BR_Steer.GetPosition().GetValueAsDouble();

  frc::SmartDashboard::PutNumber("Swerve/FL_PositionTurns", fl);
  frc::SmartDashboard::PutNumber("Swerve/FR_PositionTurns", fr);
  frc::SmartDashboard::PutNumber("Swerve/BL_PositionTurns", bl);
  frc::SmartDashboard::PutNumber("Swerve/BR_PositionTurns", br);
}

void Drivetrain::Update(double x, double y, double x2, double GyroValue,
                        double triggerL, double triggerR, bool FieldCentric) {

  // Get wheel angle with fused motor encoder and CANcoder
  double FL_pos = std::fmod(m_FL_Steer.GetPosition().GetValueAsDouble(), 1.0) * (2.0 * M_PI);
  double FR_pos = std::fmod(m_FR_Steer.GetPosition().GetValueAsDouble(), 1.0) * (2.0 * M_PI);
  double BL_pos = std::fmod(m_BL_Steer.GetPosition().GetValueAsDouble(), 1.0) * (2.0 * M_PI);
  double BR_pos = std::fmod(m_BR_Steer.GetPosition().GetValueAsDouble(), 1.0) * (2.0 * M_PI);

  // Calculate time difference since last loop
  units::time::second_t currentTime = frc::Timer::GetFPGATimestamp();
  units::time::second_t deltaTime = currentTime - lastTime;
  double doubleDeltaTime = double(deltaTime);
  lastTime = currentTime;

  if (doubleDeltaTime <= 0.0001 || doubleDeltaTime > 0.5) {
    doubleDeltaTime = 0.02;
  }

  double STR = x;
  double FWD = y;

  // Field Centric // offsetting vectors by the gyro angle
  double temp;
  if (FieldCentric) {
    temp = FWD * std::cos(GyroValue * M_PI / 180.0) + STR * std::sin(GyroValue * M_PI / 180.0);
    STR = -FWD * std::sin(GyroValue * M_PI / 180.0) + STR * std::cos(GyroValue * M_PI / 180.0);
    FWD = temp;
  } else {
    STR = x;
    FWD = y;
  }

  // PID controller for keeping robot straight to combat drag
  double intDeltaTime = doubleDeltaTime;
  double error = remainderf((targetAngle - GyroValue), 360.0);
  m_headingIError += error * intDeltaTime;
  double dError = (error - m_headingLastError) / intDeltaTime;

  double output = DrivetrainConstants::straightP * error +
                  DrivetrainConstants::straightI * m_headingIError +
                  dError * DrivetrainConstants::straightD;
  output = std::clamp(output, -1.0 * (DrivetrainConstants::outputClamp), DrivetrainConstants::outputClamp);
  m_headingLastError = error;

  if (x2 != 0.0) {
    targetAngle = GyroValue;
    ROT = x2;
  } else {
    ROT = output;
  }

  bool isDriving = (std::abs(x) > 0.001 || std::abs(y) > 0.001 || std::abs(x2) > 0.001);

  if (!isDriving) {
    // When stationary, stop all drive motors immediately and prevent steer jitter
    speedFL = 0.0;
    speedFR = 0.0;
    speedBL = 0.0;
    speedBR = 0.0;
    m_FL_Drive.Set(0.0);
    m_FR_Drive.Set(0.0);
    m_BL_Drive.Set(0.0);
    m_BR_Drive.Set(0.0);

    targetAngle = GyroValue;
    m_headingIError = 0.0;
    m_headingLastError = 0.0;

    odometryUpdate(
        FL_pos,
        FR_pos,
        BL_pos,
        BR_pos,
        0.0,
        0.0,
        0.0,
        0.0,
        GyroValue);
    return;
  }

  // Calculate variables A, B, C, D
  double A = (STR - ROT * DrivetrainConstants::L / 2.0);
  double B = (STR + ROT * DrivetrainConstants::L / 2.0);
  double C = (FWD - ROT * DrivetrainConstants::W / 2.0);
  double D = (FWD + ROT * DrivetrainConstants::W / 2.0);

  // Calculate wheel speeds
  speedFL = std::hypot(B, D);
  speedFR = std::hypot(B, C);
  speedBL = std::hypot(A, D);
  speedBR = std::hypot(A, C);

  std::vector<double> wheelSpeeds = {speedFL, speedFR, speedBL, speedBR};

  // Find the maximum value among the wheel speeds
  double maxSpeed = *std::max_element(wheelSpeeds.begin(), wheelSpeeds.end());

  // If over one, divide all wheel speeds by the max speed
  if (maxSpeed >= 1.0) {
    speedFL = speedFL / maxSpeed;
    speedFR = speedFR / maxSpeed;
    speedBL = speedBL / maxSpeed;
    speedBR = speedBR / maxSpeed;
  }

  // Calculate final angle for each wheel
  tempangleFL = std::atan2(B, D);
  tempangleFR = std::atan2(B, C);
  tempangleBL = std::atan2(A, D);
  tempangleBR = std::atan2(A, C);

  // Adjust angles to ensure minimal rotation
  angleFL = MinimizeRotation(tempangleFL, FL_pos, speedFL);
  angleFR = MinimizeRotation(tempangleFR, FR_pos, speedFR);
  angleBL = MinimizeRotation(tempangleBL, BL_pos, speedBL);
  angleBR = MinimizeRotation(tempangleBR, BR_pos, speedBR);

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

  // Calculate Speed slowdown constant
  double driveParam = DrivetrainConstants::DefaultDriveSpeed +
                      (-triggerL * DrivetrainConstants::TriggerConstant) +
                      (triggerR * DrivetrainConstants::TriggerConstant);
  double speedConst = 100.0 / std::max(5.0, driveParam);

  // Set Motors to direct duty cycle
  if (std::isnan(speedFL) || std::isnan(speedFR) || std::isnan(speedBL) || std::isnan(speedBR)) {
    speedFL = 0.0;
    speedFR = 0.0;
    speedBL = 0.0;
    speedBR = 0.0;
    m_FL_Drive.Set(0.0);
    m_FR_Drive.Set(0.0);
    m_BL_Drive.Set(0.0);
    m_BR_Drive.Set(0.0);
  } else {
    m_FL_Drive.Set(std::clamp(speedFL / speedConst, -DrivetrainConstants::DriveMotorsHardLimit, DrivetrainConstants::DriveMotorsHardLimit));
    m_FR_Drive.Set(std::clamp(speedFR / speedConst, -DrivetrainConstants::DriveMotorsHardLimit, DrivetrainConstants::DriveMotorsHardLimit));
    m_BL_Drive.Set(std::clamp(speedBL / speedConst, -DrivetrainConstants::DriveMotorsHardLimit, DrivetrainConstants::DriveMotorsHardLimit));
    m_BR_Drive.Set(std::clamp(-speedBR / speedConst, -DrivetrainConstants::DriveMotorsHardLimit, DrivetrainConstants::DriveMotorsHardLimit));
  }

  frc::SmartDashboard::PutNumber("Swerve/FL_DriveCmd", m_FL_Drive.Get());
  frc::SmartDashboard::PutNumber("Swerve/FR_DriveCmd", m_FR_Drive.Get());
  frc::SmartDashboard::PutNumber("Swerve/BL_DriveCmd", m_BL_Drive.Get());
  frc::SmartDashboard::PutNumber("Swerve/BR_DriveCmd", m_BR_Drive.Get());

  // Find Wheel Speeds in MetersPerSecond
  wheelSpeedFL = (m_FL_Drive.GetVelocity().GetValueAsDouble() / DrivetrainConstants::DriveGearRatio) * (M_PI * DrivetrainConstants::WheelCircumference);
  wheelSpeedFR = (m_FR_Drive.GetVelocity().GetValueAsDouble() / DrivetrainConstants::DriveGearRatio) * (M_PI * DrivetrainConstants::WheelCircumference);
  wheelSpeedBL = (m_BL_Drive.GetVelocity().GetValueAsDouble() / DrivetrainConstants::DriveGearRatio) * (M_PI * DrivetrainConstants::WheelCircumference);
  wheelSpeedBR = (-m_BR_Drive.GetVelocity().GetValueAsDouble() / DrivetrainConstants::DriveGearRatio) * (M_PI * DrivetrainConstants::WheelCircumference);

  odometryUpdate(
      FL_pos,
      FR_pos,
      BL_pos,
      BR_pos,
      wheelSpeedFL,
      wheelSpeedFR,
      wheelSpeedBL,
      wheelSpeedBR,
      GyroValue);
}

double Drivetrain::MinimizeRotation(double targetAngleRad, double currentAngleRad, double& speedPercent) {
  double errorRad = std::remainder(targetAngleRad - currentAngleRad, 2.0 * M_PI);

  if (std::fabs(speedPercent) > 0.01 && std::fabs(errorRad) > M_PI / 2.0) {
    errorRad -= std::copysign(M_PI, errorRad);
    speedPercent = -speedPercent;
  }

  return std::remainder(currentAngleRad + errorRad, 2.0 * M_PI);
}

void Drivetrain::odometryUpdate(
    double angleFL,
    double angleFR,
    double angleBL,
    double angleBR,
    double wheelSpeedFL,
    double wheelSpeedFR,
    double wheelSpeedBL,
    double wheelSpeedBR,
    double GyroValue) {

  units::time::second_t odoCurrentTime = frc::Timer::GetFPGATimestamp();
  odoDeltaTime = double(odoCurrentTime) - double(odoLastTime);
  odoLastTime = odoCurrentTime;

  if (odoDeltaTime <= 0.0 || odoDeltaTime > 0.5) {
    return;
  }

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

  double odoROT = (GyroValue * M_PI / 180.0);

  double FWD1 = odoROT * (DrivetrainConstants::L / 2.0) + A;
  double FWD2 = -odoROT * (DrivetrainConstants::L / 2.0) + B;
  odoSTR = ((FWD1 + FWD2) / 2.0);

  double STR1 = odoROT * (DrivetrainConstants::W / 2.0) + C;
  double STR2 = -odoROT * (DrivetrainConstants::W / 2.0) + D;
  odoFWD = (STR1 + STR2) / 2.0;

  // Normal odometry update when enabled
  positionFWDField -= units::length::meter_t(odoFWD * odoDeltaTime);
  positionSTRField += units::length::meter_t(odoSTR * odoDeltaTime);
  ROTField = units::angle::radian_t(odoROT);

  frc::SmartDashboard::PutNumber("PositionForwardField", double(positionFWDField));
  frc::SmartDashboard::PutNumber("PositionStrafeField", double(positionSTRField));
  frc::SmartDashboard::PutNumber("FwdVelocity", double(odoFWD));
  frc::SmartDashboard::PutNumber("StrVelocity", double(odoSTR));
}

// ============================================================================
// UpdateSim - Simulation physics update for desktop simulation
// ============================================================================
void Drivetrain::UpdateSim(units::time::second_t dt) {
  constexpr double kMaxRps = 100.0;

  double rpsFL = m_FL_Drive.Get() * kMaxRps;
  double rpsFR = m_FR_Drive.Get() * kMaxRps;
  double rpsBL = m_BL_Drive.Get() * kMaxRps;
  double rpsBR = -m_BR_Drive.Get() * kMaxRps;

  m_FL_Drive.GetSimState().SetRotorVelocity(units::angular_velocity::turns_per_second_t(rpsFL));
  m_FR_Drive.GetSimState().SetRotorVelocity(units::angular_velocity::turns_per_second_t(rpsFR));
  m_BL_Drive.GetSimState().SetRotorVelocity(units::angular_velocity::turns_per_second_t(rpsBL));
  m_BR_Drive.GetSimState().SetRotorVelocity(units::angular_velocity::turns_per_second_t(rpsBR));

  m_FL_Drive.GetSimState().AddRotorPosition(units::angle::turn_t(rpsFL * dt.value()));
  m_FR_Drive.GetSimState().AddRotorPosition(units::angle::turn_t(rpsFR * dt.value()));
  m_BL_Drive.GetSimState().AddRotorPosition(units::angle::turn_t(rpsBL * dt.value()));
  m_BR_Drive.GetSimState().AddRotorPosition(units::angle::turn_t(rpsBR * dt.value()));

  double turnsFL = (angleFL / (2.0 * M_PI)) * DrivetrainConstants::rotorToSensorRatio;
  double turnsFR = (angleFR / (2.0 * M_PI)) * DrivetrainConstants::rotorToSensorRatio;
  double turnsBL = (angleBL / (2.0 * M_PI)) * DrivetrainConstants::rotorToSensorRatio;
  double turnsBR = (angleBR / (2.0 * M_PI)) * DrivetrainConstants::rotorToSensorRatio;

  m_FL_Steer.GetSimState().SetRawRotorPosition(units::angle::turn_t(turnsFL));
  m_FR_Steer.GetSimState().SetRawRotorPosition(units::angle::turn_t(turnsFR));
  m_BL_Steer.GetSimState().SetRawRotorPosition(units::angle::turn_t(turnsBL));
  m_BR_Steer.GetSimState().SetRawRotorPosition(units::angle::turn_t(turnsBR));

  CANcoderFL.GetSimState().SetRawPosition(units::angle::turn_t(angleFL / (2.0 * M_PI)));
  CANcoderFR.GetSimState().SetRawPosition(units::angle::turn_t(angleFR / (2.0 * M_PI)));
  CANcoderBL.GetSimState().SetRawPosition(units::angle::turn_t(angleBL / (2.0 * M_PI)));
  CANcoderBR.GetSimState().SetRawPosition(units::angle::turn_t(angleBR / (2.0 * M_PI)));
}

