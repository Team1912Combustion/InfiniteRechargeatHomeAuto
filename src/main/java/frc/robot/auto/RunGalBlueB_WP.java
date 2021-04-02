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
import frc.robot.subsystems.Intake;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class RunGalBlueB_WP extends SequentialCommandGroup {

  Trajectory trajectory;

  private static final double MaxSpeed = 2.0;
	private static final double MaxAcceleration = 1.0;
	private static final double MaxCentripetal = 1.0;

  public RunGalBlueB_WP(DriveTrain drivetrain, Intake intake) {

    String myPathName = "";
    String trajectoryfile = "";

    myPathName = "GalBlueB_WP";

    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(DriveConstants.kS,
	  DriveConstants.kV, DriveConstants.kA), DriveConstants.kDriveKinematics, 11);

    TrajectoryConfig config = new TrajectoryConfig(MaxSpeed, MaxAcceleration)
      .setKinematics(DriveConstants.kDriveKinematics).addConstraint(autoVoltageConstraint)
      .addConstraint(new CentripetalAccelerationConstraint(MaxCentripetal));

    trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0.3, -2.3, new Rotation2d(0)),
      List.of( new Translation2d(2.4 , -2.7),
        new Translation2d(4.4 , -3.1),
        new Translation2d(5.9 , -1.5),
        new Translation2d(7.5 , -3.1)),
      new Pose2d(8.7, -1.8, new Rotation2d(Units.degreesToRadians(45))),
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
      new IntakeOut(intake),
      new IntakeOn(intake),
      m_ramsetecommand.andThen(() -> drivetrain.tankDriveVolts(0,0)),
      new IntakeOff(intake),
      new IntakeIn(intake)
    );

  }
}

