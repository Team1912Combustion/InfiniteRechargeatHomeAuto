// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class RunSlalomWP extends SequentialCommandGroup {

  Trajectory trajectory;

  // MKS units
  private static final double MaxSpeed = 3.;
	private static final double MaxAcceleration = 2.0;
	private static final double MaxCentripetal = 2.75;
  
  public RunSlalomWP(DriveTrain drivetrain) {

    String myPathName = "";
    String trajectoryfile = "";

    myPathName = "slalomWP";

    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(DriveConstants.kS,
	  DriveConstants.kV, DriveConstants.kA), DriveConstants.kDriveKinematics, 11);

    TrajectoryConfig config = new TrajectoryConfig(MaxSpeed, MaxAcceleration)
      .setKinematics(DriveConstants.kDriveKinematics).addConstraint(autoVoltageConstraint)
      .addConstraint(new CentripetalAccelerationConstraint(MaxCentripetal));

    trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.2, -3.9, new Rotation2d(0)),
      List.of( new Translation2d(2.1 , -3.7),
        new Translation2d(2.7 , -2.6),
        new Translation2d(3.6 , -1.9),
        new Translation2d(4.7 , -1.7),
        new Translation2d(5.6 , -1.8),
        new Translation2d(6.6 , -2.3),
        new Translation2d(7.3 , -3.7),
        new Translation2d(8.0 , -3.9),
        new Translation2d(8.6 , -3.4),
        new Translation2d(8.6 , -2.6),
        new Translation2d(8.2 , -2.1),
        new Translation2d(7.6 , -2.0),
        new Translation2d(7.2 , -2.6), 
        new Translation2d(6.5 , -3.7),
        new Translation2d(5.3 , -4.0),
        new Translation2d(4.4 , -4.0),
        new Translation2d(3.6 , -4.0),
        new Translation2d(3.0 , -3.7),
        new Translation2d(2.4 , -3.1),
        new Translation2d(2.0 , -2.6)),
      new Pose2d(0.8, -1.3, new Rotation2d(Units.degreesToRadians(150))),
      config);

    try {
      trajectoryfile = myPathName + ".txt";
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryfile);
      FileWriter fileWriter = new FileWriter(trajectoryPath.toString());
      PrintWriter printWriter = new PrintWriter(fileWriter);
      printWriter.print(trajectory.toString());
      printWriter.close();
    } catch (IOException ex) {
      DriverStation.reportError("Unable to write traj text: " + trajectoryfile, ex.getStackTrace());
    }

    drivetrain.resetOdometry(trajectory.getInitialPose());

    RamseteCommand m_ramsetecommand = new RamseteCommand (
      trajectory,
      drivetrain::getPose,
      new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
      new SimpleMotorFeedforward(
        DriveConstants.kS,
        DriveConstants.kV,
        DriveConstants.kA),
      DriveConstants.kDriveKinematics,
      drivetrain::getWheelSpeeds,
      new PIDController(DriveConstants.kP, 0, 0),
      new PIDController(DriveConstants.kP, 0, 0),
      drivetrain::tankDriveVolts,
      drivetrain
      );

    addCommands(
      m_ramsetecommand.andThen(() -> drivetrain.tankDriveVolts(0,0))
    );

  }
}

