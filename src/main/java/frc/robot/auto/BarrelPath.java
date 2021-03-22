// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.List;

import edu.wpi.first.wpilibj.util.Units;
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
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;

public class BarrelPath {

  public static Trajectory barrelPath;

  public static void init() {

	var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(DriveConstants.kS,
        DriveConstants.kV, DriveConstants.kA), DriveConstants.kDriveKinematics, 5);

    TrajectoryConfig config = new TrajectoryConfig(AutoConstants.MaxSpeedMetersPerSecond, AutoConstants.MaxAccelerationMetersPerSecondSquared)
    .setKinematics(DriveConstants.kDriveKinematics).addConstraint(autoVoltageConstraint)
    .addConstraint(new CentripetalAccelerationConstraint(DriveConstants.kmaxCentripetal));

	/*
	circlePath = TrajectoryGenerator.generateTrajectory(
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
		*/

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
  }
}
