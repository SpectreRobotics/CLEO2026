// ============================================================================
// FRC Team 8753 - 2026 Swerve Drivetrain Implementation
// ============================================================================

#define _USE_MATH_DEFINES
#include <cmath>

#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <units/angle.h>
#include <units/time.h>

#include "Drivetrain.h"
#include "DrivetrainConstants.h"

using namespace units::literals;

/**
 * @brief Constructor - initializes drivetrain object
 */
Drivetrain::Drivetrain() {
  // Hardware initialization handled by member initializers
}

/**
 * @brief Configure all swerve drive motors with PID, ramp rates, and sensor fusion
 *
 * Drive motors: Brake mode with voltage ramp
 * Steer motors: Coast mode with CANcoder fusion and continuous wrap
 */
void Drivetrain::ConfigureMotors() {
  // ========== Drive Motor Configuration ==========
  // Configure voltage ramp rate to prevent wheel slip
  driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
      units::time::second_t(DrivetrainConstants::DriveRampRateSeconds);
  driveConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

  // Apply drive config to all 4 drive motors
  m_FL_Drive.GetConfigurator().Apply(driveConfig);
  m_FR_Drive.GetConfigurator().Apply(driveConfig);
  m_BL_Drive.GetConfigurator().Apply(driveConfig);
  m_BR_Drive.GetConfigurator().Apply(driveConfig);

  // ========== Steering Motor Configuration ==========
  // Fuse CANcoder with TalonFX internal encoder for absolute position + high resolution

  // Front-Left Steering Module
  configFL.Feedback.FeedbackRemoteSensorID = CANcoderFL.GetDeviceID();
  configFL.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
  configFL.Feedback.SensorToMechanismRatio = DrivetrainConstants::sensorToMechanismRatio;  // CANcoder:wheel = 1:1
  configFL.Feedback.RotorToSensorRatio = DrivetrainConstants::rotorToSensorRatio;          // Motor:CANcoder = 18:1
  configFL.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod =
      units::time::second_t(DrivetrainConstants::SteeringRampRateSeconds);
  configFL.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;  // Coast for free spinning
  configFL.ClosedLoopGeneral.ContinuousWrap = true;  // Enable 0°-360° wraparound
  configFL.MotorOutput.Inverted = false;

  // Front-Right Steering Module
  configFR.Feedback.FeedbackRemoteSensorID = CANcoderFR.GetDeviceID();
  configFR.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
  configFR.Feedback.SensorToMechanismRatio = DrivetrainConstants::sensorToMechanismRatio;
  configFR.Feedback.RotorToSensorRatio = DrivetrainConstants::rotorToSensorRatio;
  configFR.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod =
      units::time::second_t(DrivetrainConstants::SteeringRampRateSeconds);
  configFR.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
  configFR.ClosedLoopGeneral.ContinuousWrap = true;
  configFR.MotorOutput.Inverted = false;

  // Back-Left Steering Module
  configBL.Feedback.FeedbackRemoteSensorID = CANcoderBL.GetDeviceID();
  configBL.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
  configBL.Feedback.SensorToMechanismRatio = DrivetrainConstants::sensorToMechanismRatio;
  configBL.Feedback.RotorToSensorRatio = DrivetrainConstants::rotorToSensorRatio;
  configBL.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod =
      units::time::second_t(DrivetrainConstants::SteeringRampRateSeconds);
  configBL.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
  configBL.ClosedLoopGeneral.ContinuousWrap = true;
  configBL.MotorOutput.Inverted = false;

  // Back-Right Steering Module
  configBR.Feedback.FeedbackRemoteSensorID = CANcoderBR.GetDeviceID();
  configBR.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
  configBR.Feedback.SensorToMechanismRatio = DrivetrainConstants::sensorToMechanismRatio;
  configBR.Feedback.RotorToSensorRatio = DrivetrainConstants::rotorToSensorRatio;
  configBR.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod =
      units::time::second_t(DrivetrainConstants::SteeringRampRateSeconds);
  configBR.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
  configBR.ClosedLoopGeneral.ContinuousWrap = true;
  configBR.MotorOutput.Inverted = false;

  // ========== PID Configuration (Slot 0) ==========
  Posconfig.kP = DrivetrainConstants::ModuleP;
  Posconfig.kI = DrivetrainConstants::ModuleI;
  Posconfig.kD = DrivetrainConstants::ModuleD;

  // Apply all configurations to steering motors
  m_FL_Steer.GetConfigurator().Apply(configFL);
  m_FL_Steer.GetConfigurator().Apply(Posconfig);
  m_FR_Steer.GetConfigurator().Apply(configFR);
  m_FR_Steer.GetConfigurator().Apply(Posconfig);
  m_BL_Steer.GetConfigurator().Apply(configBL);
  m_BL_Steer.GetConfigurator().Apply(Posconfig);
  m_BR_Steer.GetConfigurator().Apply(configBR);
  m_BR_Steer.GetConfigurator().Apply(Posconfig);
}

/**
 * @brief Update - Main swerve drive control loop
 *
 * @param x - Strafe input (-1 to 1, left/right)
 * @param y - Forward input (-1 to 1, forward/back)
 * @param x2 - Rotation input (-1 to 1, CCW/CW)
 * @param GyroValue - Current robot heading in degrees
 * @param triggerL - Left trigger (speed modifier)
 * @param triggerR - Right trigger (speed modifier)
 * @param FieldCentric - True for field-centric, false for robot-centric
 */
void Drivetrain::Update(double x, double y, double x2, double GyroValue, double triggerL, double triggerR, bool FieldCentric) {

  // ========== Read Wheel Angles ==========
  // Get current wheel angles in radians (0 to 2π)
  double FL_pos = std::fmod(m_FL_Steer.GetPosition().GetValueAsDouble(), 1.0) * (2.0 * M_PI);
  double FR_pos = std::fmod(m_FR_Steer.GetPosition().GetValueAsDouble(), 1.0) * (2.0 * M_PI);
  double BL_pos = std::fmod(m_BL_Steer.GetPosition().GetValueAsDouble(), 1.0) * (2.0 * M_PI);
  double BR_pos = std::fmod(m_BR_Steer.GetPosition().GetValueAsDouble(), 1.0) * (2.0 * M_PI);

  // ========== Calculate Delta Time ==========
  units::time::second_t currentTime = frc::Timer::GetFPGATimestamp();
  units::time::second_t deltaTime = currentTime - lastTime;
  lastTime = currentTime;

  // ========== Initialize Drive Vectors ==========
  double STR = x;  // Strafe (left/right)
  double FWD = y;  // Forward (forward/back)
  double iError = 0;
  double lastError = 0;

  // ========== Field-Centric Transformation ==========
  if (FieldCentric) {
    double temp = FWD * std::cos(GyroValue * M_PI / 180.0) + STR * std::sin(GyroValue * M_PI / 180.0);
    STR = -FWD * std::sin(GyroValue * M_PI / 180.0) + STR * std::cos(GyroValue * M_PI / 180.0);
    FWD = temp;
  } else {
    STR = x;
    FWD = y;
  }

  // ========== PID Heading Lock ==========
  double intDeltaTime = deltaTime.value();
  if (intDeltaTime <= 0.0) intDeltaTime = 0.02;
  double error = std::remainder(targetAngle - GyroValue, 360.0);
  iError += error * intDeltaTime;
  double dError = (error - lastError) / intDeltaTime;

  double output = DrivetrainConstants::straightP * error +
                  DrivetrainConstants::straightI * iError +
                  dError * DrivetrainConstants::straightD;
  output = std::clamp(output, -1.0 * DrivetrainConstants::outputClamp, DrivetrainConstants::outputClamp);
  lastError = error;

  if (x2 != 0) {
    targetAngle = GyroValue;
    ROT = x2;
  } else {
    ROT = output;
  }

  // ========== Swerve Drive Kinematics ==========
  double A = (STR - ROT * DrivetrainConstants::L / 2.0);
  double B = (STR + ROT * DrivetrainConstants::L / 2.0);
  double C = (FWD - ROT * DrivetrainConstants::W / 2.0);
  double D = (FWD + ROT * DrivetrainConstants::W / 2.0);

  // ========== Calculate Wheel Speeds ==========
  if (x != 0 || y != 0 || x2 != 0) {
    speedFL = std::sqrt((B * B) + (D * D));
    speedFR = std::sqrt((B * B) + (C * C));
    speedBL = std::sqrt((A * A) + (D * D));
    speedBR = std::sqrt((A * A) + (C * C));
  } else {
    speedFL = 0;
    speedFR = 0;
    speedBL = 0;
    speedBR = 0;
  }

  // ========== Normalize Wheel Speeds ==========
  std::vector<double> wheelSpeeds = {speedFL, speedFR, speedBL, speedBR};
  double maxSpeed = *std::max_element(wheelSpeeds.begin(), wheelSpeeds.end());

  if (maxSpeed >= 1.0) {
    speedFL = speedFL / maxSpeed;
    speedFR = speedFR / maxSpeed;
    speedBL = speedBL / maxSpeed;
    speedBR = speedBR / maxSpeed;
  }

  // ========== Calculate Wheel Angles ==========
  if (x != 0 || y != 0 || x2 != 0) {
    tempangleFL = std::atan2(B, D);
    tempangleFR = std::atan2(B, C);
    tempangleBL = std::atan2(A, D);
    tempangleBR = std::atan2(A, C);
  }

  // ========== Minimize Rotation ==========
  angleFL = MinimizeRotation(tempangleFL, FL_pos, speedFL);
  angleFR = MinimizeRotation(tempangleFR, FR_pos, speedFR);
  angleBL = MinimizeRotation(tempangleBL, BL_pos, speedBL);
  angleBR = MinimizeRotation(tempangleBR, BR_pos, speedBR);

  // ========== Send Steering Commands ==========
  ctre::phoenix6::controls::PositionDutyCycle steerm_speedFL = ctre::phoenix6::controls::PositionDutyCycle{0_tr}.WithSlot(0);
  ctre::phoenix6::controls::PositionDutyCycle steerm_speedFR = ctre::phoenix6::controls::PositionDutyCycle{0_tr}.WithSlot(0);
  ctre::phoenix6::controls::PositionDutyCycle steerm_speedBL = ctre::phoenix6::controls::PositionDutyCycle{0_tr}.WithSlot(0);
  ctre::phoenix6::controls::PositionDutyCycle steerm_speedBR = ctre::phoenix6::controls::PositionDutyCycle{0_tr}.WithSlot(0);

  m_FL_Steer.SetControl(steerm_speedFL.WithPosition(units::angle::turn_t((angleFL / M_PI) / 2.0)));
  m_FR_Steer.SetControl(steerm_speedFR.WithPosition(units::angle::turn_t((angleFR / M_PI) / 2.0)));
  m_BL_Steer.SetControl(steerm_speedBL.WithPosition(units::angle::turn_t((angleBL / M_PI) / 2.0)));
  m_BR_Steer.SetControl(steerm_speedBR.WithPosition(units::angle::turn_t((angleBR / M_PI) / 2.0)));

  // ========== Calculate Speed Scaling ==========
  double speedConst = 100.0 / (DrivetrainConstants::DefaultDriveSpeed +
                              (-triggerL * DrivetrainConstants::TriggerConstant) +
                              (triggerR * DrivetrainConstants::TriggerConstant));

  // ========== Send Drive Motor Commands ==========
  if (std::isnan(speedFL)) {
    speedFL = 0;
    speedFR = 0;
    speedBL = 0;
    speedBR = 0;
    m_FL_Drive.Set(0);
    m_FR_Drive.Set(0);
    m_BL_Drive.Set(0);
    m_BR_Drive.Set(0);
  } else {
    double cmdFL = std::clamp(speedFL / speedConst, -DrivetrainConstants::DriveMotorsHardLimit, DrivetrainConstants::DriveMotorsHardLimit);
    double cmdFR = std::clamp(speedFR / speedConst, -DrivetrainConstants::DriveMotorsHardLimit, DrivetrainConstants::DriveMotorsHardLimit);
    double cmdBL = std::clamp(speedBL / speedConst, -DrivetrainConstants::DriveMotorsHardLimit, DrivetrainConstants::DriveMotorsHardLimit);
    double cmdBR = std::clamp(-speedBR / speedConst, -DrivetrainConstants::DriveMotorsHardLimit, DrivetrainConstants::DriveMotorsHardLimit);
    m_FL_Drive.Set(cmdFL);
    m_FR_Drive.Set(cmdFR);
    m_BL_Drive.Set(cmdBL);
    m_BR_Drive.Set(cmdBR);
    frc::SmartDashboard::PutNumber("DBG_cmdFL", cmdFL);
    frc::SmartDashboard::PutNumber("DBG_cmdBR", cmdBR);
    frc::SmartDashboard::PutNumber("DBG_speedConst", speedConst);
  }

  // ========== Calculate Wheel Velocities for Odometry ==========
  wheelSpeedFL = (m_FL_Drive.GetVelocity().GetValueAsDouble() / DrivetrainConstants::DriveGearRatio) * (M_PI * DrivetrainConstants::WheelCircumference);
  wheelSpeedFR = (m_FR_Drive.GetVelocity().GetValueAsDouble() / DrivetrainConstants::DriveGearRatio) * (M_PI * DrivetrainConstants::WheelCircumference);
  wheelSpeedBL = (m_BL_Drive.GetVelocity().GetValueAsDouble() / DrivetrainConstants::DriveGearRatio) * (M_PI * DrivetrainConstants::WheelCircumference);
  wheelSpeedBR = (-m_BR_Drive.GetVelocity().GetValueAsDouble() / DrivetrainConstants::DriveGearRatio) * (M_PI * DrivetrainConstants::WheelCircumference);
}

double Drivetrain::MinimizeRotation(double targetAngleRad, double currentAngleRad, double& speedPercent) {
  double errorRad = std::remainder(targetAngleRad - currentAngleRad, 2.0 * M_PI);

  if (std::fabs(errorRad) > M_PI / 2.0) {
    errorRad -= std::copysign(M_PI, errorRad);
    speedPercent = -speedPercent;
  }

  return std::remainder(currentAngleRad + errorRad, 2.0 * M_PI);
}

double Drivetrain::WrapError(double targetAngle, double currentAngle) {
  return std::remainder(targetAngle - currentAngle, 2.0 * M_PI);
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

  double odoROT = (GyroValue * M_PI / 180.0);

  double FWD1 = odoROT * (DrivetrainConstants::L / 2.0) + A;
  double FWD2 = -odoROT * (DrivetrainConstants::L / 2.0) + B;
  odoSTR = ((FWD1 + FWD2) / 2.0);

  double STR1 = odoROT * (DrivetrainConstants::W / 2.0) + C;
  double STR2 = -odoROT * (DrivetrainConstants::W / 2.0) + D;
  odoFWD = (STR1 + STR2) / 2.0;

  double gyroRad = GyroValue * M_PI / 180.0;
  double fieldFWD = odoFWD * std::cos(gyroRad) - odoSTR * std::sin(gyroRad);
  double fieldSTR = odoFWD * std::sin(gyroRad) + odoSTR * std::cos(gyroRad);

  positionFWDField -= units::length::meter_t(fieldFWD * odoDeltaTime);
  positionSTRField += units::length::meter_t(fieldSTR * odoDeltaTime);
  ROTField = frc::Rotation2d(units::angle::radian_t(odoROT));

  frc::SmartDashboard::PutNumber("PositionForwardField", positionFWDField.value());
  frc::SmartDashboard::PutNumber("PositionStrafeField", positionSTRField.value());
}
