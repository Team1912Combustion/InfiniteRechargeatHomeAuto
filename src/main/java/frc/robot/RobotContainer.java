// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.List;

import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;

public class RobotContainer {
  private final DriveTrain drive = new DriveTrain();
 
  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {}

  public Command getAutonomousCommand() {

    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(DriveConstants.kS,
    DriveConstants.kV, DriveConstants.kA), DriveConstants.kDriveKinematics, 5);

    TrajectoryConfig config = new TrajectoryConfig(AutoConstants.MaxSpeedMetersPerSecond, AutoConstants.MaxAccelerationMetersPerSecondSquared)
    .setKinematics(DriveConstants.kDriveKinematics).addConstraint(autoVoltageConstraint)
    .addConstraint(new CentripetalAccelerationConstraint(DriveConstants.kmaxCentripetal));

    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(2, 0),
            new Translation2d(2, 2),
            new Translation2d(0, 2),
            new Translation2d(0, 0),
            new Translation2d(2, 0),
            new Translation2d(2, 2),
            new Translation2d(0, 2),
            new Translation2d(0, 0),
            new Translation2d(2, 0)),
            new Pose2d(2.0, 2.0, new Rotation2d(Units.degreesToRadians(135.))),
            config);

    Trajectory barrelPath =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(1., -2.2, new Rotation2d(0)),
            List.of(new Translation2d(3.2, -2.2),
            new Translation2d(3.8, -2.3),
            new Translation2d(4.3, -3.1),
            new Translation2d(3.8, -3.6),
            new Translation2d(3.4, -2.9),
            new Translation2d(4.3, -2.4),
            new Translation2d(6.0, -2.1),
            new Translation2d(6.8, -1.2),
            new Translation2d(6.0, -0.8),
            new Translation2d(5.6, -1.5),
            new Translation2d(6.2, -2.8),
            new Translation2d(7.4, -3.5),
            new Translation2d(8.2, -2.8),
            new Translation2d(6.4, -2.3),
            new Translation2d(4.8, -2.0),
            new Translation2d(2.0, -2.0)),
            new Pose2d(1.0, -1.9, new Rotation2d(Units.degreesToRadians(180))),
            config);

    try {
      FileWriter fileWriter = new FileWriter("/home/lvuser/barrelPathTrajectory.txt");
      PrintWriter printWriter = new PrintWriter(fileWriter);
      printWriter.print(barrelPath.toString());
      printWriter.close();
    } catch (IOException e) {
      e.printStackTrace();
    }

    RamseteController disabledRamsete = new RamseteController() {
      @Override
      public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters,
        double angularVelocityRefRadiansPerSecond) {
          return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
      }
    };

    RamseteCommand command =
        new RamseteCommand(
        barrelPath,
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

    drive.resetOdometry(barrelPath.getInitialPose());

    return command.andThen(() -> drive.tankDriveVolts(0, 0));

  }
}
