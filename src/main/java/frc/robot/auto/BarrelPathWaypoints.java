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

public class BarrelPathWaypoints {

  static Trajectory trajectory;

  public static Trajectory getTrajectory() {
	  return trajectory;
  }

  public static void init() {
		var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(DriveConstants.kS,
        DriveConstants.kV, DriveConstants.kA), DriveConstants.kDriveKinematics, 11);

    TrajectoryConfig config = new TrajectoryConfig(AutoConstants.MaxSpeedMetersPerSecond, AutoConstants.MaxAccelerationMetersPerSecondSquared)
    .setKinematics(DriveConstants.kDriveKinematics).addConstraint(autoVoltageConstraint)
    .addConstraint(new CentripetalAccelerationConstraint(DriveConstants.kmaxCentripetal));

		/* clean, 14s
    trajectory =
	TrajectoryGenerator.generateTrajectory(
		new Pose2d(1., -2.2, new Rotation2d(0)),
		List.of( new Translation2d(3.2 , -2.2),
				new Translation2d(3.8 , -2.3),
				new Translation2d(4.6 , -2.6),
				new Translation2d(4.8 , -3.1),
				new Translation2d(4.4 , -3.7),
				new Translation2d(3.7 , -3.9),
				new Translation2d(3.2 , -3.6),
				new Translation2d(3.1 , -2.9),
				new Translation2d(4.3 , -2.4),
				new Translation2d(6.1 , -2.3),
				new Translation2d(7.2 , -1.7),
				new Translation2d(6.9 , -1),
				new Translation2d(6   , -.8), 
				new Translation2d(5.3 , -1),
				new Translation2d(5.4 , -2),
				new Translation2d(6.1 , -3),
				new Translation2d(7.4 , -3.8),
				new Translation2d(8.3 , -3.6),
				new Translation2d(8.4 , -2.6),
				new Translation2d(7.5 , -2.1),
				new Translation2d(6.5 , -2.1),
				new Translation2d(4.8 , -2.1)),
	    new Pose2d(-1.0, -2.2, new Rotation2d(Units.degreesToRadians(180))),
		config);
		*/

    trajectory = TrajectoryGenerator.generateTrajectory(
			new Pose2d(1., -2.2, new Rotation2d(0)),
			List.of( new Translation2d(3.2 , -2.2),
				new Translation2d(3.8 , -2.3),
				new Translation2d(4.6 , -2.6),
				new Translation2d(4.7 , -3.1),
				new Translation2d(4.4 , -3.7),
				new Translation2d(3.7 , -3.9),
				new Translation2d(3.2 , -3.6),
				new Translation2d(3.1 , -2.9),
				new Translation2d(4.3 , -2.4),
				new Translation2d(6.1 , -2.3),
				new Translation2d(7.1 , -1.7),
				new Translation2d(6.9 , -1),
				new Translation2d(6   , -.8), 
				new Translation2d(5.3 , -1),
				new Translation2d(5.4 , -2),
				new Translation2d(6.1 , -3),
				new Translation2d(7.4 , -3.8),
				new Translation2d(8.3 , -3.6),
				new Translation2d(8.4 , -2.6),
				new Translation2d(7.5 , -2.1),
				new Translation2d(6.5 , -2.1),
				new Translation2d(4.8 , -2.1)),
	    new Pose2d(-1.0, -2.2, new Rotation2d(Units.degreesToRadians(180))),
	    config);

    try {
      FileWriter fileWriter = new FileWriter("/home/lvuser/barrelPathTrajectory.txt");
      //FileWriter fileWriter = new FileWriter("/tmp/barrelPathTrajectory.txt");
      PrintWriter printWriter = new PrintWriter(fileWriter);
      printWriter.print(trajectory.toString());
      printWriter.close();
    } catch (IOException e) {
      e.printStackTrace();
		}
  }

}
