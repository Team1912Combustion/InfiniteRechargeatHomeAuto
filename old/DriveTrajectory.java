// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class DriveTrajectory {

  public static Command driveTrajectory(DriveTrain drive, Trajectory trajectory) {

    /*
    RamseteController disabledRamsete = new RamseteController() {
      @Override
      public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters,
        double angularVelocityRefRadiansPerSecond) {
          return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
      }
    };
    */

    RamseteCommand command =
        new RamseteCommand(
        trajectory,
        drive::getPose,
        new RamseteController(AutoConstants.kRamseteB,AutoConstants.kRamseteZeta),
        // disabledRamsete,
        new SimpleMotorFeedforward(
            DriveConstants.kS,
            DriveConstants.kV,
            DriveConstants.kA),
        DriveConstants.kDriveKinematics,
        drive::getWheelSpeeds,
        new PIDController(DriveConstants.kP, 0, 0),
        new PIDController(DriveConstants.kP, 0, 0),
        drive::tankDriveVolts,
        drive);

    drive.resetOdometry(trajectory.getInitialPose());
    System.out.println("Ramsete set, odom reset, let's go");
    return command.andThen(() -> drive.tankDriveVolts(0, 0));

  }
}
