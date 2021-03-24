// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

public class BarrelPath {

  static Trajectory barrelPath;

  public static void init() {
		String myPathName = "";
		String trajectoryfile = "";

		myPathName = "barrel";

		trajectoryfile = myPathName + ".wpilib.json";
		Trajectory trajectory = new Trajectory();
		try {
			Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryfile);
			trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
		} catch (IOException ex) {
			DriverStation.reportError("Unable to open trajectory: " + trajectoryfile, ex.getStackTrace());
		}

		try {
			trajectoryfile = myPathName + ".txt";
			Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryfile);
			FileWriter fileWriter = new FileWriter(trajectoryPath.toString());
			PrintWriter printWriter = new PrintWriter(fileWriter);
			printWriter.print(trajectory.toString());
			printWriter.close();
		} catch (IOException e) {
			e.printStackTrace();
		}

  }

}