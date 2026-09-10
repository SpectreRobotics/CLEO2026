// ============================================================================
// FRC Team 8753 - 2026 Swerve Drivetrain
// ============================================================================
// Implements a 4-module swerve drive system with field-centric control,
// wheel odometry, and encoder-based position tracking.
// ============================================================================

#pragma once

// WPILib includes
#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/DriverStation.h>
#include <frc/kinematics/ChassisSpeeds.h>

// NetworkTables includes
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableValue.h>

// CTRE Phoenix 6 includes
#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>

// Standard library
#include <cmath>
#include <vector>
#include <algorithm>
#include "DrivetrainConstants.h"

#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

/**
 * @brief Swerve drivetrain class with 4 independent modules
 *
 * Each module consists of:
 * - 1x Kraken TalonFX drive motor
 * - 1x Kraken TalonFX steering motor
 * - 1x CANcoder absolute encoder (fused with steering motor)
 */
class Drivetrain {
    public:
        // ========== Lifecycle Methods ==========
        /**
         * @brief Constructor - initializes drivetrain
         */
        Drivetrain();

        /**
         * @brief Applies motor configurations (PID, ramp rates, sensor fusion)
         */
        void ConfigureMotors();

        // ========== Control Methods ==========
        /**
         * @brief Main swerve drive update - calculates module states and applies commands
         * @param x Strafe input (left/right, -1.0 to 1.0)
         * @param y Forward input (forward/back, -1.0 to 1.0)
         * @param x2 Rotation input (-1.0 to 1.0)
         * @param GyroValue Current heading in degrees
         * @param triggerL Left trigger value (brake/slow mode)
         * @param triggerR Right trigger value (boost mode)
         * @param FieldCentric true for field-centric, false for robot-centric
         */
        void Update(double x, double y, double x2, double GyroValue,
                    double triggerL, double triggerR, bool FieldCentric);

        /**
         * @brief Optimizes swerve module rotation to minimize travel
         * @param targetAngleRad Desired wheel angle (radians)
         * @param currentAngleRad Current wheel angle (radians)
         * @param speedPercent Reference to speed - reversed if wheel flips
         * @return Optimized target angle (radians)
         */
        double MinimizeRotation(double targetAngleRad, double currentAngleRad, double& speedPercent);

        /**
         * @brief Wraps angle error to -PI to PI range
         * @param targetAngle Target angle (radians)
         * @param currentAngle Current angle (radians)
         * @return Wrapped error (-PI to PI)
         */
        double WrapError(double targetAngle, double currentAngle);

        /**
         * @brief Updates robot field position using wheel odometry
         * @param angleFL Front-left wheel angle (radians)
         * @param angleFR Front-right wheel angle (radians)
         * @param angleBL Back-left wheel angle (radians)
         * @param angleBR Back-right wheel angle (radians)
         * @param wheelSpeedFL Front-left wheel velocity (m/s)
         * @param wheelSpeedFR Front-right wheel velocity (m/s)
         * @param wheelSpeedBL Back-left wheel velocity (m/s)
         * @param wheelSpeedBR Back-right wheel velocity (m/s)
         * @param GyroValue Current gyro heading (degrees)
         */
        void odometryUpdate(double angleFL, double angleFR, double angleBL, double angleBR,
                            double wheelSpeedFL, double wheelSpeedFR, double wheelSpeedBL,
                            double wheelSpeedBR, double GyroValue);

        // ========== Odometry State (Public for access from Robot) ==========
        units::length::meter_t positionFWDField = units::length::meter_t(0);  ///< Forward position (meters)
        units::length::meter_t positionSTRField = units::length::meter_t(0);  ///< Strafe position (meters)
        frc::Rotation2d ROTField = frc::Rotation2d(units::radian_t(0));       ///< Field heading (radians)
        double odoSTR = 0;                                                    ///< Strafe velocity (m/s)
        double odoFWD = 0;                                                    ///< Forward velocity (m/s)

        // ========== Odometry Timing ==========
        double odoDeltaTime = 0;                                              ///< Delta time for odometry (seconds)
        units::time::second_t odoLastTime = units::time::second_t(0);         ///< Last odometry update timestamp

        // ========== Hardware - Drive Motors ==========
        /**
         * Kraken TalonFX drive motors (power the wheels)
         * CAN ID pattern: X0 where X is module number (1-4)
         */
        ctre::phoenix6::hardware::TalonFX m_FL_Drive{30};  ///< Front-left drive (ID 30, module 3)
        ctre::phoenix6::hardware::TalonFX m_FR_Drive{20};  ///< Front-right drive (ID 20, module 2)
        ctre::phoenix6::hardware::TalonFX m_BL_Drive{40};  ///< Back-left drive (ID 40, module 4)
        ctre::phoenix6::hardware::TalonFX m_BR_Drive{10};  ///< Back-right drive (ID 10, module 1)

        // ========== Hardware - Steering Motors ==========
        /**
         * Kraken TalonFX steering motors (rotate the modules)
         * CAN ID pattern: X1 where X is module number (1-4)
         */
        ctre::phoenix6::hardware::TalonFX m_FL_Steer{31};  ///< Front-left steer (ID 31, module 3)
        ctre::phoenix6::hardware::TalonFX m_FR_Steer{21};  ///< Front-right steer (ID 21, module 2)
        ctre::phoenix6::hardware::TalonFX m_BL_Steer{41};  ///< Back-left steer (ID 41, module 4)
        ctre::phoenix6::hardware::TalonFX m_BR_Steer{11};  ///< Back-right steer (ID 11, module 1)

    private:
        // ========== Hardware - Absolute Encoders ==========
        /**
         * CANcoder absolute encoders (fused with steering motors for accuracy)
         * CAN ID pattern: X2 where X is module number (1-4)
         */
        ctre::phoenix6::hardware::CANcoder CANcoderFL{32};  ///< Front-left CANcoder (ID 32, module 3)
        ctre::phoenix6::hardware::CANcoder CANcoderFR{22};  ///< Front-right CANcoder (ID 22, module 2)
        ctre::phoenix6::hardware::CANcoder CANcoderBL{42};  ///< Back-left CANcoder (ID 42, module 4)
        ctre::phoenix6::hardware::CANcoder CANcoderBR{12};  ///< Back-right CANcoder (ID 12, module 1)

        // ========== Motor Configurations ==========
        ctre::phoenix6::configs::Slot0Configs Posconfig{};              ///< PID config for steering position
        ctre::phoenix6::configs::TalonFXConfiguration driveConfig{};    ///< Drive motor config (ramp, brake)
        ctre::phoenix6::configs::TalonFXConfiguration configFL{};       ///< Front-left steer config
        ctre::phoenix6::configs::TalonFXConfiguration configFR{};       ///< Front-right steer config
        ctre::phoenix6::configs::TalonFXConfiguration configBL{};       ///< Back-left steer config
        ctre::phoenix6::configs::TalonFXConfiguration configBR{};       ///< Back-right steer config
        ctre::phoenix6::configs::CANcoderConfiguration cc_cfg{};        ///< CANcoder config (unused)

        // ========== Swerve Module State ==========
        double tempangleFL = 0, tempangleFR = 0, tempangleBL = 0, tempangleBR = 0;  ///< Temporary angle calculations
        double angleFL = 0, angleFR = 0, angleBL = 0, angleBR = 0;                  ///< Optimized module target angles (rad)
        double speedFL = 0, speedFR = 0, speedBL = 0, speedBR = 0;                  ///< Module drive speeds (duty cycle)

        // ========== Timing ==========
        units::time::second_t lastTime = units::time::second_t(0);  ///< Last update timestamp

        // ========== Joystick Inputs (after field-centric transform) ==========
        double ROT = 0;         ///< Rotation command (rad/s)
        double FWD = 0;         ///< Forward command (m/s)
        double STR = 0;         ///< Strafe command (m/s)
        double targetAngle = 0; ///< Temporary target angle variable

        // ========== Wheel Velocities ==========
        double wheelSpeedFL = 0, wheelSpeedFR = 0, wheelSpeedBL = 0, wheelSpeedBR = 0;      ///< Linear wheel speeds (m/s)
        double angularSpeedFL = 0, angularSpeedFR = 0, angularSpeedBL = 0, angularSpeedBR = 0;  ///< Angular velocities (rad/s)
};

#endif  // DRIVETRAIN_H
