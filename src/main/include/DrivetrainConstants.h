// ============================================================================
// FRC Team 8753 - 2026 Swerve Drivetrain Constants
// ============================================================================
// Complete ground-up configuration based on ChimiSwerve (Team 1684).
// Hardware: 4x MK3.5 Swerve Modules with Kraken X60 (TalonFX) & CTRE CANcoders.
// ============================================================================

#pragma once

class DrivetrainConstants {
  public:
    // ========== Robot Physical Dimensions ==========
    static constexpr float L = 0.533;                       ///< Length front-to-back (meters)
    static constexpr float W = 0.533;                       ///< Width left-to-right (meters)
    static constexpr double R = 0.7538;                     ///< Diagonal wheelbase radius sqrt(L^2 + W^2)

    // ========== Drive Motor Configuration ==========
    static constexpr double DriveMotorsHardLimit = 0.75;    ///< Max drive motor output (0.0-1.0)
    static constexpr double DriveGearRatio = 5.0;           ///< Drive gearbox reduction ratio (5:1)
    static constexpr double WheelDiameter = 0.102;          ///< Wheel diameter (meters, ~4")
    static constexpr double WheelCircumference = 0.102;     ///< Compatibility alias for WheelDiameter

    static constexpr double DriveOpenLoopRamp = 0.1;        ///< Open loop duty cycle ramp rate (seconds)
    static constexpr double kDriveCurrentLimit = 60.0;      ///< Drive motor stator current limit (Amps)

    // ========== Inversions & Magnet Offsets ==========
    static constexpr bool kFLDriveInverted = false;
    static constexpr bool kFRDriveInverted = true;
    static constexpr bool kBLDriveInverted = false;
    static constexpr bool kBRDriveInverted = true;

    static constexpr bool kSteerMotorInverted = false;
    static constexpr double kSteerPeakOutput = 1.0;

    static constexpr double kFLMagnetOffset = 0.0;
    static constexpr double kFRMagnetOffset = 0.0;
    static constexpr double kBLMagnetOffset = 0.0;
    static constexpr double kBRMagnetOffset = 0.0;

    // ========== Speed Control ==========
    static constexpr double TriggerConstant = 30.0;         ///< Speed scaling factor for triggers
    static constexpr double DefaultDriveSpeed = 35.0;       ///< Base drive speed multiplier

    // ========== Joystick Deadzones ==========
    static constexpr double xdeadz = 0.1;
    static constexpr double ydeadz = 0.1;
    static constexpr double x2deadz = 0.1;

    // ========== Swerve Module PID Tuning ==========
    static constexpr float ModuleP = 2.0;                   ///< Steering P-gain (position control)
    static constexpr float ModuleI = 0.0;                   ///< Steering I-gain (disabled)
    static constexpr float ModuleD = 0.0;                   ///< Steering D-gain (disabled)

    // ========== Straight Drive PID (Heading Lock) ==========
    static constexpr float straightP = 0.02;                ///< Straight drive heading lock P-gain
    static constexpr float straightI = 0.0;
    static constexpr float straightD = 0.0;

    // ========== Path Following Configuration ==========
    static constexpr double lookaheadDistance = 0.3;        ///< Pure pursuit lookahead (meters)
    static constexpr double pathFollowP = 1.5;              ///< Path following P-gain
    static constexpr double pathFollowD = 0.1;              ///< Path following D-gain

    // ========== Motor Ramp Rates ==========
    static constexpr float DriveRampRateSeconds = 1.1;      ///< Drive motor acceleration limit (seconds)
    static constexpr float SteeringRampRateSeconds = 0.01;  ///< Steering acceleration limit (seconds)

    // ========== Encoder Fusion Configuration ==========
    static constexpr float sensorToMechanismRatio = 1.0;    ///< CANcoder to wheel ratio
    static constexpr float rotorToSensorRatio = 18.0;       ///< Motor rotations per CANcoder rotation (18:1)

    // ========== Output Clamping ==========
    static constexpr double outputClamp = 0.7;              ///< Max drive output (m/s or %)
    static constexpr double correctionClamp = 0.3;          ///< Max correction output
    static constexpr double rotationClamp = 0.3;            ///< Max rotation output
};
