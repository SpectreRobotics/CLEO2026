// ============================================================================
// FRC Team 8753 - 2026 Swerve Drivetrain Implementation
// ============================================================================
// Complete ground-up implementation based 1:1 on ChimiSwerve (Team 1684) White Paper.
// Hardware: 4x MK3.5 Swerve Modules with Kraken X60 (TalonFX) & CTRE CANcoders.
// ============================================================================

#define _USE_MATH_DEFINES
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <algorithm>
#include <vector>

#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>
#include <units/time.h>
#include <units/current.h>
#include <units/angular_velocity.h>

#include "Drivetrain.h"
#include "DrivetrainConstants.h"

using namespace units::literals;

Drivetrain::Drivetrain() {
}

// ============================================================================
// ConfigureMotors - Set up Kraken TalonFX and CANcoder feedback
// ============================================================================
void Drivetrain::ConfigureMotors() {
  // ========== Drive Motor Configuration ==========
  auto setupDrive = [](ctre::phoenix6::configs::TalonFXConfiguration& cfg, bool inverted) {
    cfg.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        units::time::second_t(DrivetrainConstants::DriveRampRateSeconds);
    cfg.OpenLoopRamps.DutyCycleOpenLoopRampPeriod =
        units::time::second_t(DrivetrainConstants::DriveOpenLoopRamp);
    cfg.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    cfg.CurrentLimits.StatorCurrentLimit =
        units::current::ampere_t(DrivetrainConstants::kDriveCurrentLimit);
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.MotorOutput.Inverted = inverted
        ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
        : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  };

  setupDrive(driveConfigFL, DrivetrainConstants::kFLDriveInverted);
  setupDrive(driveConfigFR, DrivetrainConstants::kFRDriveInverted);
  setupDrive(driveConfigBL, DrivetrainConstants::kBLDriveInverted);
  setupDrive(driveConfigBR, DrivetrainConstants::kBRDriveInverted);

  auto applyWithRetry = [](ctre::phoenix6::hardware::TalonFX& motor, const auto& cfg) {
    ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    for (int i = 0; i < 2 && !status.IsOK(); ++i) {
      status = motor.GetConfigurator().Apply(cfg, 0.02_s);
    }
  };

  applyWithRetry(m_FL_Drive, driveConfigFL);
  applyWithRetry(m_FR_Drive, driveConfigFR);
  applyWithRetry(m_BL_Drive, driveConfigBL);
  applyWithRetry(m_BR_Drive, driveConfigBR);

  // ========== CANcoder Configuration (Magnet Offsets) ==========
  auto setupCANcoder = [](ctre::phoenix6::hardware::CANcoder& cc, double magnetOffset) {
    ctre::phoenix6::configs::CANcoderConfiguration cfg{};
    cfg.MagnetSensor.MagnetOffset = units::angle::turn_t(magnetOffset);
    cfg.MagnetSensor.SensorDirection =
        ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
    cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = units::angle::turn_t(0.5);
    ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    for (int i = 0; i < 2 && !status.IsOK(); ++i) {
      status = cc.GetConfigurator().Apply(cfg, 0.02_s);
    }
  };

  setupCANcoder(CANcoderFL, DrivetrainConstants::kFLMagnetOffset);
  setupCANcoder(CANcoderFR, DrivetrainConstants::kFRMagnetOffset);
  setupCANcoder(CANcoderBL, DrivetrainConstants::kBLMagnetOffset);
  setupCANcoder(CANcoderBR, DrivetrainConstants::kBRMagnetOffset);

  // ========== Steering Motor Configuration (RemoteCANcoder Fusion) ==========
  auto setupSteerConfig = [](ctre::phoenix6::configs::TalonFXConfiguration& cfg, int canCoderId) {
    cfg.Feedback.FeedbackRemoteSensorID = canCoderId;
    cfg.Feedback.FeedbackSensorSource =
        ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
    cfg.Feedback.SensorToMechanismRatio = DrivetrainConstants::sensorToMechanismRatio;
    cfg.Feedback.RotorToSensorRatio = DrivetrainConstants::rotorToSensorRatio;
    cfg.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod =
        units::time::second_t(DrivetrainConstants::SteeringRampRateSeconds);
    cfg.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    cfg.ClosedLoopGeneral.ContinuousWrap = true;
    cfg.MotorOutput.Inverted = DrivetrainConstants::kSteerMotorInverted
        ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
        : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
    cfg.MotorOutput.PeakForwardDutyCycle = DrivetrainConstants::kSteerPeakOutput;
    cfg.MotorOutput.PeakReverseDutyCycle = -DrivetrainConstants::kSteerPeakOutput;
  };

  setupSteerConfig(steerConfigFL, CANcoderFL.GetDeviceID());
  setupSteerConfig(steerConfigFR, CANcoderFR.GetDeviceID());
  setupSteerConfig(steerConfigBL, CANcoderBL.GetDeviceID());
  setupSteerConfig(steerConfigBR, CANcoderBR.GetDeviceID());

  steerSlot0.kP = DrivetrainConstants::ModuleP;
  steerSlot0.kI = DrivetrainConstants::ModuleI;
  steerSlot0.kD = DrivetrainConstants::ModuleD;

  applyWithRetry(m_FL_Steer, steerConfigFL);
  applyWithRetry(m_FL_Steer, steerSlot0);

  applyWithRetry(m_FR_Steer, steerConfigFR);
  applyWithRetry(m_FR_Steer, steerSlot0);

  applyWithRetry(m_BL_Steer, steerConfigBL);
  applyWithRetry(m_BL_Steer, steerSlot0);

  applyWithRetry(m_BR_Steer, steerConfigBR);
  applyWithRetry(m_BR_Steer, steerSlot0);

  // Initialize targets; hasBootstrapped stays false until DisabledPeriodic / Update reads live CAN
  hasBootstrapped = false;
}

// ============================================================================
// Disabled Periodic / Init - Keep setpoints synchronized to physical positions
// ============================================================================
void Drivetrain::DisabledInit() {
  m_FL_Drive.Set(0.0);
  m_FR_Drive.Set(0.0);
  m_BL_Drive.Set(0.0);
  m_BR_Drive.Set(0.0);
}

void Drivetrain::DisabledPeriodic() {
  // Read absolute positions directly from CANcoders (always online, prevents error spam if a steer motor is offline)
  double ccFL = CANcoderFL.GetPosition().GetValueAsDouble();
  double ccFR = CANcoderFR.GetPosition().GetValueAsDouble();
  double ccBL = CANcoderBL.GetPosition().GetValueAsDouble();
  double ccBR = CANcoderBR.GetPosition().GetValueAsDouble();

  // Read TalonFX positions only if device is communicating
  double fl = m_FL_Steer.IsConnected() ? m_FL_Steer.GetPosition().GetValueAsDouble() : ccFL;
  double fr = m_FR_Steer.IsConnected() ? m_FR_Steer.GetPosition().GetValueAsDouble() : ccFR;
  double bl = m_BL_Steer.IsConnected() ? m_BL_Steer.GetPosition().GetValueAsDouble() : ccBL;
  double br = m_BR_Steer.IsConnected() ? m_BR_Steer.GetPosition().GetValueAsDouble() : ccBR;

  // Continuously track current physical angles so enabling causes zero jump
  lastTargetTurnsFL = ccFL;
  lastTargetTurnsFR = ccFR;
  lastTargetTurnsBL = ccBL;
  lastTargetTurnsBR = ccBR;
  hasBootstrapped = true;

  // Telemetry: Display both TalonFX steer position and CANcoder position while disabled
  frc::SmartDashboard::PutNumber("Swerve/FL_PositionTurns", fl);
  frc::SmartDashboard::PutNumber("Swerve/FR_PositionTurns", fr);
  frc::SmartDashboard::PutNumber("Swerve/BL_PositionTurns", bl);
  frc::SmartDashboard::PutNumber("Swerve/BR_PositionTurns", br);

  frc::SmartDashboard::PutNumber("Swerve/FL_CANcoderTurns", ccFL);
  frc::SmartDashboard::PutNumber("Swerve/FR_CANcoderTurns", ccFR);
  frc::SmartDashboard::PutNumber("Swerve/BL_CANcoderTurns", ccBL);
  frc::SmartDashboard::PutNumber("Swerve/BR_CANcoderTurns", ccBR);

  // Degrees readout [-180, 180] for physical alignment checking on the cart
  frc::SmartDashboard::PutNumber("Swerve/FL_CANcoderDeg", std::remainder(ccFL, 1.0) * 360.0);
  frc::SmartDashboard::PutNumber("Swerve/FR_CANcoderDeg", std::remainder(ccFR, 1.0) * 360.0);
  frc::SmartDashboard::PutNumber("Swerve/BL_CANcoderDeg", std::remainder(ccBL, 1.0) * 360.0);
  frc::SmartDashboard::PutNumber("Swerve/BR_CANcoderDeg", std::remainder(ccBR, 1.0) * 360.0);

  // Live Drive Motor Inversion controls on Dashboard
  frc::SmartDashboard::SetDefaultBoolean("Swerve/Inv_FL_Drive", DrivetrainConstants::kFLDriveInverted);
  frc::SmartDashboard::SetDefaultBoolean("Swerve/Inv_FR_Drive", DrivetrainConstants::kFRDriveInverted);
  frc::SmartDashboard::SetDefaultBoolean("Swerve/Inv_BL_Drive", DrivetrainConstants::kBLDriveInverted);
  frc::SmartDashboard::SetDefaultBoolean("Swerve/Inv_BR_Drive", DrivetrainConstants::kBRDriveInverted);

  // Live re-application if user changes an inversion checkbox on dashboard
  bool dashFL = frc::SmartDashboard::GetBoolean("Swerve/Inv_FL_Drive", DrivetrainConstants::kFLDriveInverted);
  bool dashFR = frc::SmartDashboard::GetBoolean("Swerve/Inv_FR_Drive", DrivetrainConstants::kFRDriveInverted);
  bool dashBL = frc::SmartDashboard::GetBoolean("Swerve/Inv_BL_Drive", DrivetrainConstants::kBLDriveInverted);
  bool dashBR = frc::SmartDashboard::GetBoolean("Swerve/Inv_BR_Drive", DrivetrainConstants::kBRDriveInverted);

  static bool lastFL = DrivetrainConstants::kFLDriveInverted;
  static bool lastFR = DrivetrainConstants::kFRDriveInverted;
  static bool lastBL = DrivetrainConstants::kBLDriveInverted;
  static bool lastBR = DrivetrainConstants::kBRDriveInverted;

  if (dashFL != lastFL) {
    lastFL = dashFL;
    driveConfigFL.MotorOutput.Inverted = dashFL
        ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
        : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
    m_FL_Drive.GetConfigurator().Apply(driveConfigFL);
  }
  if (dashFR != lastFR) {
    lastFR = dashFR;
    driveConfigFR.MotorOutput.Inverted = dashFR
        ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
        : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
    m_FR_Drive.GetConfigurator().Apply(driveConfigFR);
  }
  if (dashBL != lastBL) {
    lastBL = dashBL;
    driveConfigBL.MotorOutput.Inverted = dashBL
        ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
        : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
    m_BL_Drive.GetConfigurator().Apply(driveConfigBL);
  }
  if (dashBR != lastBR) {
    lastBR = dashBR;
    driveConfigBR.MotorOutput.Inverted = dashBR
        ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
        : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
    m_BR_Drive.GetConfigurator().Apply(driveConfigBR);
  }
}

// ============================================================================
// ChimiOptimizeAzimuth - Inversion Awareness & Shortest Path (Pages 15-16)
// ============================================================================
double Drivetrain::ChimiOptimizeAzimuth(double waRad, double currentTurns, double& ws) {
  // Convert target angle from radians to turns [-0.5, +0.5]
  // Note: Kinematics (atan2(STR, FWD)) is Clockwise-positive.
  // Phoenix 6 CANcoder is CounterClockwise-positive (CCW+).
  // Negating waRad converts from CW kinematics space to CCW sensor space so strafe and turn point correctly:
  double targetTurns = -waRad / (2.0 * M_PI);

  // Shortest angular difference between target and current in turns [-0.5, +0.5]
  double errorTurns = std::remainder(targetTurns - currentTurns, 1.0);

  // ChimiSwerve Inversion Awareness (Page 16):
  // If angular error > 90° (0.25 turns), flip target by 180° (0.5 turns) and invert speed
  if (std::abs(errorTurns) > 0.25) {
    errorTurns -= std::copysign(0.5, errorTurns);
    ws = -ws;
  }

  // ChimiSwerve Cosine Scaling:
  // Scale wheel drive speed by cos(error). If wheel is 90 deg away from target,
  // speed is 0. If aligned, speed is 100%. Prevents scrubbing and motor stalling!
  double errorRad = errorTurns * (2.0 * M_PI);
  double cosScalar = std::max(0.0, std::cos(errorRad));
  ws *= cosScalar;

  // Continuous setpoint (never more than 0.25 turns away from current position!)
  return currentTurns + errorTurns;
}

// ============================================================================
// Update - Main Swerve Control Loop
// ============================================================================
void Drivetrain::Update(double x, double y, double x2, double GyroValue,
                        double triggerL, double triggerR, bool FieldCentric) {

  // Read current steering motor positions (unbounded turns), fallback to CANcoder if motor is offline
  double turnsFL = m_FL_Steer.IsConnected() ? m_FL_Steer.GetPosition().GetValueAsDouble() : CANcoderFL.GetPosition().GetValueAsDouble();
  double turnsFR = m_FR_Steer.IsConnected() ? m_FR_Steer.GetPosition().GetValueAsDouble() : CANcoderFR.GetPosition().GetValueAsDouble();
  double turnsBL = m_BL_Steer.IsConnected() ? m_BL_Steer.GetPosition().GetValueAsDouble() : CANcoderBL.GetPosition().GetValueAsDouble();
  double turnsBR = m_BR_Steer.IsConnected() ? m_BR_Steer.GetPosition().GetValueAsDouble() : CANcoderBR.GetPosition().GetValueAsDouble();

  // Initial bootstrap if not done yet
  if (!hasBootstrapped) {
    lastTargetTurnsFL = turnsFL;
    lastTargetTurnsFR = turnsFR;
    lastTargetTurnsBL = turnsBL;
    lastTargetTurnsBR = turnsBR;
    storedYaw = GyroValue;
    hasBootstrapped = true;
  }

  // Current physical wheel angles in radians for odometry [-pi, pi] (converted from CCW+ sensor to CW+ kinematics)
  double waFL_cur = -std::remainder(turnsFL, 1.0) * (2.0 * M_PI);
  double waFR_cur = -std::remainder(turnsFR, 1.0) * (2.0 * M_PI);
  double waBL_cur = -std::remainder(turnsBL, 1.0) * (2.0 * M_PI);
  double waBR_cur = -std::remainder(turnsBR, 1.0) * (2.0 * M_PI);

  // ========== Neutral / Stop Check (ChimiSwerve Section 2.3d, Page 36) ==========
  // When no translation or rotation is commanded, stop all drive motors immediately.
  // Hold steering at the last commanded position so wheels do NOT spin or twitch.
  constexpr double kDeadband = 0.05;
  bool isTranslating = (std::abs(x) >= kDeadband || std::abs(y) >= kDeadband);
  bool isRotating = (std::abs(x2) >= kDeadband);

  if (!isTranslating && !isRotating) {
    m_FL_Drive.Set(0.0);
    m_FR_Drive.Set(0.0);
    m_BL_Drive.Set(0.0);
    m_BR_Drive.Set(0.0);

    // Hold steering motors at current setpoint (no motion)
    m_FL_Steer.SetControl(steerRequest.WithPosition(units::angle::turn_t(lastTargetTurnsFL)));
    m_FR_Steer.SetControl(steerRequest.WithPosition(units::angle::turn_t(lastTargetTurnsFR)));
    m_BL_Steer.SetControl(steerRequest.WithPosition(units::angle::turn_t(lastTargetTurnsBL)));
    m_BR_Steer.SetControl(steerRequest.WithPosition(units::angle::turn_t(lastTargetTurnsBR)));

    odometryUpdate(waFL_cur, waFR_cur, waBL_cur, waBR_cur, 0.0, 0.0, 0.0, 0.0, GyroValue);

    // Neutral telemetry
    frc::SmartDashboard::PutNumber("Swerve/FL_TargetTurns", lastTargetTurnsFL);
    frc::SmartDashboard::PutNumber("Swerve/FL_ActualTurns", turnsFL);
    frc::SmartDashboard::PutNumber("Swerve/FR_TargetTurns", lastTargetTurnsFR);
    frc::SmartDashboard::PutNumber("Swerve/FR_ActualTurns", turnsFR);
    frc::SmartDashboard::PutNumber("Swerve/BL_TargetTurns", lastTargetTurnsBL);
    frc::SmartDashboard::PutNumber("Swerve/BL_ActualTurns", turnsBL);
    frc::SmartDashboard::PutNumber("Swerve/BR_TargetTurns", lastTargetTurnsBR);
    frc::SmartDashboard::PutNumber("Swerve/BR_ActualTurns", turnsBR);

    frc::SmartDashboard::PutNumber("Swerve/FL_DriveCmd", 0.0);
    frc::SmartDashboard::PutNumber("Swerve/FR_DriveCmd", 0.0);
    frc::SmartDashboard::PutNumber("Swerve/BL_DriveCmd", 0.0);
    frc::SmartDashboard::PutNumber("Swerve/BR_DriveCmd", 0.0);

    frc::SmartDashboard::PutNumber("Swerve/Joy_X", x);
    frc::SmartDashboard::PutNumber("Swerve/Joy_Y", y);
    frc::SmartDashboard::PutNumber("Swerve/Joy_X2", x2);
    return;
  }

  // ========== Field-Centric Transformation (ChimiSwerve Pages 11-13) ==========
  double FWD = y;
  double STR = x;
  if (FieldCentric) {
    double gyroRad = GyroValue * (M_PI / 180.0);
    double temp = FWD * std::cos(gyroRad) + STR * std::sin(gyroRad);
    STR = STR * std::cos(gyroRad) - FWD * std::sin(gyroRad);
    FWD = temp;
  }

  // ========== Heading Lock / Drive Straight PID (ChimiSwerve Pages 34-36) ==========
  double ROT = 0.0;
  if (isRotating) {
    storedYaw = GyroValue;
    ROT = x2;
  } else if (isTranslating) {
    double errorAngle = std::remainder(storedYaw - GyroValue, 360.0);
    double yawCorrection = errorAngle * DrivetrainConstants::straightP;
    ROT = std::clamp(yawCorrection, -DrivetrainConstants::outputClamp, DrivetrainConstants::outputClamp);
  }

  // ========== Inverse Kinematics (ChimiSwerve Pages 13-14) ==========
  double L = DrivetrainConstants::L;
  double W = DrivetrainConstants::W;
  double R = DrivetrainConstants::R;

  double A = STR - ROT * (L / R);
  double B = STR + ROT * (L / R);
  double C = FWD - ROT * (W / R);
  double D = FWD + ROT * (W / R);

  // Raw module speeds
  double wsFR = std::hypot(B, C);
  double wsFL = std::hypot(B, D);
  double wsRR = std::hypot(A, C);  // Back-Right (Module 1)
  double wsRL = std::hypot(A, D);  // Back-Left (Module 4)

  // Raw module angles in radians [-pi, pi], clockwise positive, 0 is straight forward
  double waFR = std::atan2(B, C);
  double waFL = std::atan2(B, D);
  double waRR = std::atan2(A, C);
  double waRL = std::atan2(A, D);

  // ========== Normalize Wheel Speeds (ChimiSwerve Page 14) ==========
  double maxSpeed = std::max({wsFL, wsFR, wsRL, wsRR});
  if (maxSpeed > 1.0) {
    wsFL /= maxSpeed;
    wsFR /= maxSpeed;
    wsRL /= maxSpeed;
    wsRR /= maxSpeed;
  }

  // ========== Inversion Awareness & Azimuth Optimization (Pages 15-16) ==========
  lastTargetTurnsFL = ChimiOptimizeAzimuth(waFL, turnsFL, wsFL);
  lastTargetTurnsFR = ChimiOptimizeAzimuth(waFR, turnsFR, wsFR);
  lastTargetTurnsBL = ChimiOptimizeAzimuth(waRL, turnsBL, wsRL);
  lastTargetTurnsBR = ChimiOptimizeAzimuth(waRR, turnsBR, wsRR);

  // ========== Send Steering Commands (Continuous Turns) ==========
  m_FL_Steer.SetControl(steerRequest.WithPosition(units::angle::turn_t(lastTargetTurnsFL)));
  m_FR_Steer.SetControl(steerRequest.WithPosition(units::angle::turn_t(lastTargetTurnsFR)));
  m_BL_Steer.SetControl(steerRequest.WithPosition(units::angle::turn_t(lastTargetTurnsBL)));
  m_BR_Steer.SetControl(steerRequest.WithPosition(units::angle::turn_t(lastTargetTurnsBR)));

  // ========== Speed Scaling (Triggers) ==========
  double driveSpeedParam = DrivetrainConstants::DefaultDriveSpeed +
                           (-triggerL * DrivetrainConstants::TriggerConstant) +
                           (triggerR * DrivetrainConstants::TriggerConstant);
  double speedConst = 100.0 / std::max(5.0, driveSpeedParam);

  // ========== Send Drive Commands ==========
  double cmdFL = 0.0, cmdFR = 0.0, cmdBL = 0.0, cmdBR = 0.0;
  if (!std::isnan(wsFL) && !std::isnan(wsFR) && !std::isnan(wsRL) && !std::isnan(wsRR)) {
    cmdFL = std::clamp(wsFL / speedConst, -DrivetrainConstants::DriveMotorsHardLimit, DrivetrainConstants::DriveMotorsHardLimit);
    cmdFR = std::clamp(wsFR / speedConst, -DrivetrainConstants::DriveMotorsHardLimit, DrivetrainConstants::DriveMotorsHardLimit);
    cmdBL = std::clamp(wsRL / speedConst, -DrivetrainConstants::DriveMotorsHardLimit, DrivetrainConstants::DriveMotorsHardLimit);
    cmdBR = std::clamp(wsRR / speedConst, -DrivetrainConstants::DriveMotorsHardLimit, DrivetrainConstants::DriveMotorsHardLimit);

    m_FL_Drive.Set(cmdFL);
    m_FR_Drive.Set(cmdFR);
    m_BL_Drive.Set(cmdBL);
    m_BR_Drive.Set(cmdBR);
  } else {
    m_FL_Drive.Set(0.0);
    m_FR_Drive.Set(0.0);
    m_BL_Drive.Set(0.0);
    m_BR_Drive.Set(0.0);
  }

  // ========== Calculate Wheel Velocities for Odometry (m/s) ==========
  // Motor inversion is handled in hardware configuration, so GetVelocity() is always positive forward
  double vFL = (m_FL_Drive.GetVelocity().GetValueAsDouble() / DrivetrainConstants::DriveGearRatio) * (M_PI * DrivetrainConstants::WheelCircumference);
  double vFR = (m_FR_Drive.GetVelocity().GetValueAsDouble() / DrivetrainConstants::DriveGearRatio) * (M_PI * DrivetrainConstants::WheelCircumference);
  double vBL = (m_BL_Drive.GetVelocity().GetValueAsDouble() / DrivetrainConstants::DriveGearRatio) * (M_PI * DrivetrainConstants::WheelCircumference);
  double vBR = (m_BR_Drive.GetVelocity().GetValueAsDouble() / DrivetrainConstants::DriveGearRatio) * (M_PI * DrivetrainConstants::WheelCircumference);

  odometryUpdate(waFL_cur, waFR_cur, waBL_cur, waBR_cur, vFL, vFR, vBL, vBR, GyroValue);

  // Drive motor current monitoring to detect any stall immediately
  frc::SmartDashboard::PutNumber("Swerve/FL_CurrentAmps", m_FL_Drive.GetStatorCurrent().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Swerve/FR_CurrentAmps", m_FR_Drive.GetStatorCurrent().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Swerve/BL_CurrentAmps", m_BL_Drive.GetStatorCurrent().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Swerve/BR_CurrentAmps", m_BR_Drive.GetStatorCurrent().GetValueAsDouble());

  // ========== Active Telemetry ==========
  frc::SmartDashboard::PutNumber("Swerve/FL_TargetTurns", lastTargetTurnsFL);
  frc::SmartDashboard::PutNumber("Swerve/FL_ActualTurns", turnsFL);
  frc::SmartDashboard::PutNumber("Swerve/FR_TargetTurns", lastTargetTurnsFR);
  frc::SmartDashboard::PutNumber("Swerve/FR_ActualTurns", turnsFR);
  frc::SmartDashboard::PutNumber("Swerve/BL_TargetTurns", lastTargetTurnsBL);
  frc::SmartDashboard::PutNumber("Swerve/BL_ActualTurns", turnsBL);
  frc::SmartDashboard::PutNumber("Swerve/BR_TargetTurns", lastTargetTurnsBR);
  frc::SmartDashboard::PutNumber("Swerve/BR_ActualTurns", turnsBR);

  frc::SmartDashboard::PutNumber("Swerve/FL_DriveCmd", cmdFL);
  frc::SmartDashboard::PutNumber("Swerve/FR_DriveCmd", cmdFR);
  frc::SmartDashboard::PutNumber("Swerve/BL_DriveCmd", cmdBL);
  frc::SmartDashboard::PutNumber("Swerve/BR_DriveCmd", cmdBR);

  frc::SmartDashboard::PutNumber("Swerve/Joy_X", x);
  frc::SmartDashboard::PutNumber("Swerve/Joy_Y", y);
  frc::SmartDashboard::PutNumber("Swerve/Joy_X2", x2);
  frc::SmartDashboard::PutBoolean("Swerve/FieldCentric", FieldCentric);
}

// ============================================================================
// odometryUpdate - ChimiSwerve Forward Kinematics & Odometry (Pages 17-19)
// ============================================================================
void Drivetrain::odometryUpdate(
    double waFL,
    double waFR,
    double waBL,
    double waBR,
    double wsFL,
    double wsFR,
    double wsBL,
    double wsBR,
    double GyroValue) {

  units::time::second_t odoCurrentTime = frc::Timer::GetFPGATimestamp();
  if (odoLastTime == units::time::second_t(0)) {
    odoLastTime = odoCurrentTime;
    return;
  }
  odoDeltaTime = double(odoCurrentTime - odoLastTime);
  odoLastTime = odoCurrentTime;

  if (odoDeltaTime <= 0.0 || odoDeltaTime > 0.5) {
    return;
  }

  // Calculate A, B, C, D from wheel speeds and angles (Page 18)
  double B_FL = std::sin(waFL) * wsFL;
  double D_FL = std::cos(waFL) * wsFL;

  double B_FR = std::sin(waFR) * wsFR;
  double C_FR = std::cos(waFR) * wsFR;

  double A_BL = std::sin(waBL) * wsBL;
  double D_BL = std::cos(waBL) * wsBL;

  double A_BR = std::sin(waBR) * wsBR;
  double C_BR = std::cos(waBR) * wsBR;

  // Average components (Page 18)
  double A = (A_BR + A_BL) / 2.0;
  double B = (B_FL + B_FR) / 2.0;
  double C = (C_FR + C_BR) / 2.0;
  double D = (D_FL + D_BL) / 2.0;

  double odoROT = (GyroValue * M_PI / 180.0);
  ROTField = units::angle::radian_t(odoROT);

  // Robot-relative velocities: average opposing sides (Section 2.2b, Page 19)
  odoFWD = (A + B) / 2.0;
  odoSTR = (C + D) / 2.0;

  // Field-centric transformation of velocities (Page 19)
  double gyroRad = GyroValue * (M_PI / 180.0);
  double fieldFWD = odoFWD * std::cos(gyroRad) + odoSTR * std::sin(gyroRad);
  double fieldSTR = odoSTR * std::cos(gyroRad) - odoFWD * std::sin(gyroRad);

  // Integrate into field position (Page 19)
  positionFWDField += units::length::meter_t(fieldFWD * odoDeltaTime);
  positionSTRField += units::length::meter_t(fieldSTR * odoDeltaTime);
  ROTField = units::angle::radian_t(odoROT);

  frc::SmartDashboard::PutNumber("PositionForwardField", double(positionFWDField));
  frc::SmartDashboard::PutNumber("PositionStrafeField", double(positionSTRField));
  frc::SmartDashboard::PutNumber("RobotHeadingDeg", GyroValue);
}

// ============================================================================
// UpdateSim - Simulation physics update for desktop simulation
// ============================================================================
void Drivetrain::UpdateSim(units::time::second_t dt) {
  constexpr double kMaxRps = 100.0;

  double rpsFL = m_FL_Drive.Get() * kMaxRps;
  double rpsFR = m_FR_Drive.Get() * kMaxRps;
  double rpsBL = m_BL_Drive.Get() * kMaxRps;
  double rpsBR = m_BR_Drive.Get() * kMaxRps;

  m_FL_Drive.GetSimState().SetRotorVelocity(units::angular_velocity::turns_per_second_t(rpsFL));
  m_FR_Drive.GetSimState().SetRotorVelocity(units::angular_velocity::turns_per_second_t(rpsFR));
  m_BL_Drive.GetSimState().SetRotorVelocity(units::angular_velocity::turns_per_second_t(rpsBL));
  m_BR_Drive.GetSimState().SetRotorVelocity(units::angular_velocity::turns_per_second_t(rpsBR));

  m_FL_Drive.GetSimState().AddRotorPosition(units::angle::turn_t(rpsFL * dt.value()));
  m_FR_Drive.GetSimState().AddRotorPosition(units::angle::turn_t(rpsFR * dt.value()));
  m_BL_Drive.GetSimState().AddRotorPosition(units::angle::turn_t(rpsBL * dt.value()));
  m_BR_Drive.GetSimState().AddRotorPosition(units::angle::turn_t(rpsBR * dt.value()));

  m_FL_Steer.GetSimState().SetRawRotorPosition(units::angle::turn_t(lastTargetTurnsFL * DrivetrainConstants::rotorToSensorRatio));
  m_FR_Steer.GetSimState().SetRawRotorPosition(units::angle::turn_t(lastTargetTurnsFR * DrivetrainConstants::rotorToSensorRatio));
  m_BL_Steer.GetSimState().SetRawRotorPosition(units::angle::turn_t(lastTargetTurnsBL * DrivetrainConstants::rotorToSensorRatio));
  m_BR_Steer.GetSimState().SetRawRotorPosition(units::angle::turn_t(lastTargetTurnsBR * DrivetrainConstants::rotorToSensorRatio));

  CANcoderFL.GetSimState().SetRawPosition(units::angle::turn_t(lastTargetTurnsFL));
  CANcoderFR.GetSimState().SetRawPosition(units::angle::turn_t(lastTargetTurnsFR));
  CANcoderBL.GetSimState().SetRawPosition(units::angle::turn_t(lastTargetTurnsBL));
  CANcoderBR.GetSimState().SetRawPosition(units::angle::turn_t(lastTargetTurnsBR));
}
