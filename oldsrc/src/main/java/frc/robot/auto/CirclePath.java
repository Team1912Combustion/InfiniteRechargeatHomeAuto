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

public class CirclePath {

  static Trajectory circlePath;

  public static Trajectory getTrajectory() {
	return circlePath;
  }

  public static void init() {
	var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(DriveConstants.kS,
        DriveConstants.kV, DriveConstants.kA), DriveConstants.kDriveKinematics, 5);

    TrajectoryConfig config = new TrajectoryConfig(AutoConstants.MaxSpeedMetersPerSecond, AutoConstants.MaxAccelerationMetersPerSecondSquared)
    .setKinematics(DriveConstants.kDriveKinematics).addConstraint(autoVoltageConstraint)
    .addConstraint(new CentripetalAccelerationConstraint(DriveConstants.kmaxCentripetal));

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

    try {
      FileWriter fileWriter = new FileWriter("/home/lvuser/circlePathTrajectory.txt");
      //FileWriter fileWriter = new FileWriter("/tmp/circlePathTrajectory.txt");
      PrintWriter printWriter = new PrintWriter(fileWriter);
      printWriter.print(circlePath.toString());
      printWriter.close();
    } catch (IOException e) {
      e.printStackTrace();
	}
  }

}
