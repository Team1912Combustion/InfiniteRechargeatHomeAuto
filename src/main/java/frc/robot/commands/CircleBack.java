// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.subsystems.DriveTrain;
import frc.robot.auto.*;

public class CircleBack extends SequentialCommandGroup {

  public DriveTrain drive = new DriveTrain();
  public Trajectory trajectory = new Trajectory();

  public CircleBack() {
    addCommands(
      driveHalfCircle(),
      driveHalfCircleReverse()
      );
  }

  public Command driveHalfCircle() {
    trajectory = HalfCircle.getTrajectory();
    return DriveTrajectory.driveTrajectory(drive, trajectory);
  }

  public Command driveHalfCircleReverse() {
    trajectory = HalfCircleReverse.getTrajectory();
    return DriveTrajectory.driveTrajectory(drive, trajectory);
  }

}
