// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "LimelightHelpers.h"

#include <frc2/command/CommandScheduler.h>

using namespace units::literals;

Robot::Robot() {}

void Robot::RobotPeriodic() {
    m_timeAndJoystickReplay.Update();
    frc2::CommandScheduler::GetInstance().Run();

    /*
     * This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to target
     * and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible, though exact implementation
     * of how to use vision should be tuned per-robot and to the team's specification.
     */
    if (kUseLimelight) {
        auto const driveState = m_container.drivetrain.GetState();
        auto const heading = driveState.Pose.Rotation().Degrees();
        auto const omega = driveState.Speeds.omega;

        LimelightHelpers::SetRobotOrientation("limelight", heading.value(), 0, 0, 0, 0, 0);
        auto llMeasurement = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if (llMeasurement && llMeasurement->tagCount > 0 && units::math::abs(omega) < 2_tps) {
            m_container.drivetrain.AddVisionMeasurement(llMeasurement->pose, llMeasurement->timestampSeconds);
        }
    }
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
    m_autoTimer.Restart();
    m_autonomousCommand = m_container.GetAutonomousCommand();

    if (m_autonomousCommand) {
        frc2::CommandScheduler::GetInstance().Schedule(m_autonomousCommand);
    }
}

void Robot::AutonomousPeriodic() {
    // If no PathPlanner auto command is scheduled, you can run time-based autonomous logic here:
    if (!m_autonomousCommand) {
        // Example time-based autonomous:
        // if (m_autoTimer.Get() < 2_s) {
        //     // Drive forward at 1.0 m/s for 2 seconds
        //     m_container.drivetrain.SetControl(
        //         swerve::requests::RobotCentric{}.WithVelocityX(1.0_mps)
        //     );
        // } else {
        //     // Brake when time is up
        //     m_container.drivetrain.SetControl(swerve::requests::SwerveDriveBrake{});
        // }
    }
}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
    if (m_autonomousCommand) {
        frc2::CommandScheduler::GetInstance().Cancel(m_autonomousCommand);
    }
}

void Robot::TeleopPeriodic() {
    // Time-based teleop routines or custom periodic mechanism logic can be placed here.
}

void Robot::TeleopExit() {}

void Robot::TestInit() {
    frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
