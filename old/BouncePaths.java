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

public class BouncePaths {

  static Trajectory bouncePath1;
  static Trajectory bouncePath2;
  static Trajectory bouncePath3;
  static Trajectory bouncePath4;

  public static Trajectory getTrajectory(int step) {
		Trajectory myPath = bouncePath1;
		switch (step) {
			case 1:
			  myPath = bouncePath1;
			case 2:
			  myPath = bouncePath2;
			case 3:
			  myPath = bouncePath3;
			case 4:
			  myPath = bouncePath4;
		}
		return myPath;
  }

  public static void init() {
		String myPathName = "";
		String trajectoryfile = "";

		for (int step = 1; step < 5; step++)
		{
			System.out.println("step:");
			System.out.println(step);

			switch (step) {
				case 1:
					myPathName = "bounce1"; break;
				case 2:
					myPathName = "bounce2"; break;
				case 3:
					myPathName = "bounce3"; break;
				case 4:
					myPathName = "bounce4"; break;
				default:
					myPathName = "Invalid myPathName in BouncePath"; break;
			}

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

			switch (step) {
				case 1:
					bouncePath1 = trajectory; break;
				case 2:
					bouncePath2 = trajectory; break;
				case 3:
					bouncePath3 = trajectory; break;
				case 4:
					bouncePath4 = trajectory; break;
				default:
					System.out.println("invalid option in BouncePaths");
			}
			System.out.println("have path for "+myPathName);

		}

  }

}