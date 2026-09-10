// ============================================================================
// FRC Team 8753 - 2026 Swerve Drivetrain Constants
// ============================================================================
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// ============================================================================

#pragma once

/**
 * @brief Swerve drivetrain configuration constants
 *
 * Physical dimensions, gear ratios, PID tuning, and motion limits
 * for the 4-module swerve drive system.
 */
class DrivetrainConstants {
  public:
    // ========== Robot Physical Dimensions ==========
    /**
     * Robot frame dimensions (wheel center to wheel center)
     * These define the kinematic model for swerve calculations
     */
    static constexpr float L = 0.533;                       ///< Length front-to-back (meters)
    static constexpr float W = 0.533;                       ///< Width left-to-right (meters)

    // ========== Drive Motor Configuration ==========
    static constexpr double DriveMotorsHardLimit = 0.75;    ///< Max drive motor output (0.0-1.0)
    static constexpr double DriveGearRatio = 5.0;           ///< Drive gearbox reduction ratio (5:1)
    static constexpr double WheelCircumference = 0.102;     ///< Wheel circumference (meters, ~4" diameter)

    // ========== Speed Control ==========
    static constexpr double TriggerConstant = 30.0;         ///< Speed scaling factor for triggers
    static constexpr double DefaultDriveSpeed = 35.0;       ///< Base drive speed multiplier

    // ========== Joystick Deadzones ==========
    static constexpr double xdeadz = 0.1;
    static constexpr double ydeadz = 0.1;
    static constexpr double x2deadz = 0.1;

    // ========== Swerve Module PID Tuning ==========
    /**
     * Position PID for steering modules (Phoenix 6 slot 0)
     */
    static constexpr float ModuleP = 2.0;                   ///< Steering P-gain (position control)
    static constexpr float ModuleI = 0.0;                   ///< Steering I-gain (disabled)
    static constexpr float ModuleD = 0.0;                   ///< Steering D-gain (disabled)

    // ========== Straight Drive PID (Unused) ==========
    static constexpr float straightP = 0.0;                 ///< Legacy straight drive P-gain
    static constexpr float straightI = 0.0;                 ///< Legacy straight drive I-gain
    static constexpr float straightD = 0.0;                 ///< Legacy straight drive D-gain

    // ========== Path Following Configuration ==========
    static constexpr double lookaheadDistance = 0.3;        ///< Pure pursuit lookahead (meters)
    static constexpr double pathFollowP = 1.5;              ///< Path following P-gain
    static constexpr double pathFollowD = 0.1;              ///< Path following D-gain

    // ========== Motor Ramp Rates ==========
    static constexpr float DriveRampRateSeconds = 1.1;      ///< Drive motor acceleration limit (seconds)
    static constexpr float SteeringRampRateSeconds = 0.01;  ///< Steering acceleration limit (seconds)

    // ========== Encoder Fusion Configuration ==========
    /**
     * Ratios for fusing CANcoder with TalonFX internal encoder
     * Provides higher resolution and absolute position
     */
    static constexpr float sensorToMechanismRatio = 1.0;    ///< CANcoder to wheel ratio
    static constexpr float rotorToSensorRatio = 18.0;       ///< Motor rotations per CANcoder rotation (18:1)

    // ========== Output Clamping ==========
    static constexpr double outputClamp = 0.7;              ///< Max drive output (m/s or %)
    static constexpr double correctionClamp = 0.3;          ///< Max correction output
    static constexpr double rotationClamp = 0.3;            ///< Max rotation output
};
