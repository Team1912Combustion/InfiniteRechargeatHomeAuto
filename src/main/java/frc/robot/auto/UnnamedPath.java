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

public class UnnamedPath {

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

	trajectory = TrajectoryGenerator.generateTrajectory(
	    new Pose2d(1.1053814747951674,-3.9497864185529785, new Rotation2d(0)),
	    List.of(
        new Translation2d(2.5021874739619494,-2.9212292737119845),
        new Translation2d(3.010116928204416,-2.3244121649770864),
        new Translation2d(4.584698236356061,-1.7402932925982502),
        new Translation2d(6.222770726288015,-2.222826274128593),
        new Translation2d(7.759257325371476,-3.822804054992362),
        new Translation2d(8.419565615886683,-2.9847204554922926),
        new Translation2d(7.670369670879044,-2.057749201499792),
        new Translation2d(6.108486599083461,-3.708519927787807),
        new Translation2d(4.508508818219691,-4.051372309401471),
        new Translation2d(2.895832800999861,-3.670425218719622)),
	    new Pose2d(0.8514167476739342,-1.5117250381891407, new Rotation2d(Units.degreesToRadians(160.))),
		config);

    try {
      //FileWriter fileWriter = new FileWriter("/home/lvuser/circlePathTrajectory.txt");
      FileWriter fileWriter = new FileWriter("/tmp/unnamedPathTrajectory.txt");
      PrintWriter printWriter = new PrintWriter(fileWriter);
      printWriter.print(trajectory.toString());
      printWriter.close();
    } catch (IOException e) {
      e.printStackTrace();
	}
  }

}
