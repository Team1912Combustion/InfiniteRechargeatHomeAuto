// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.auto.*;
import frc.robot.commands.*;

import edu.wpi.first.wpilibj.trajectory.Trajectory;

public class RobotContainer {
  private DriveTrain drive = new DriveTrain();
  private Trajectory trajectory = new Trajectory();
 
  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {}

  public Command getAutonomousCommand() {
    trajectory = BarrelPath.getTrajectory();
    return DriveTrajectory.driveTrajectory(drive, trajectory);
  }

  public Command oldAutonomousCommand() {
    trajectory = CirclePath.getTrajectory();
    return DriveTrajectory.driveTrajectory(drive, trajectory);
  }

  public Command newergetAutonomousCommand() {
    Command circleBack = new CircleBack();
    return circleBack;
  }

}
