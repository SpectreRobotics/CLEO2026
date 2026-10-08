// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

class DrivetrainConstants {
  public:
    // Drivetrain.cpp
    // General
    // Constants for the robot dimensions and encoder configuration (meters, wheel center to wheel center)
    static constexpr float L = 0.533; // length
    static constexpr float W = 0.533; // width

    static constexpr double DriveMotorsHardLimit = 0.75;

    static constexpr double DriveGearRatio = 5.0;
    static constexpr double WheelCircumference = 0.102;

    static constexpr double TriggerConstant = 30.0;
    static constexpr double DefaultDriveSpeed = 35.0;

    // Joystick deadzones (used by Robot.cpp)
    static constexpr double xdeadz = 0.1;
    static constexpr double ydeadz = 0.1;
    static constexpr double x2deadz = 0.1;

    // PID
    static constexpr float ModuleP = 2.0;
    static constexpr float ModuleI = 0.0;
    static constexpr float ModuleD = 0.0;
    static constexpr float straightP = 0.0;
    static constexpr float straightI = 0.0;
    static constexpr float straightD = 0.0;

    // Path variables (in meters)
    static constexpr double lookaheadDistance = 0.3;
    static constexpr double pathFollowP = 1.5;
    static constexpr double pathFollowD = 0.1;

    // Setup
    static constexpr float DriveRampRateSeconds = 1.1;
    static constexpr float sensorToMechanismRatio = 1.0;
    static constexpr float rotorToSensorRatio = 18.0;
    static constexpr float SteeringRampRateSeconds = 0.01;

    // Calculations & Other
    static constexpr double outputClamp = 0.7;
    static constexpr double correctionClamp = 0.3;
    static constexpr double rotationClamp = 0.3;
};
