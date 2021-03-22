// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.BarrelPath;
import frc.robot.auto.DriveTrajectory;
import frc.robot.subsystems.DriveTrain;

public class RobotContainer {
  private final DriveTrain drive = new DriveTrain();
 
  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {}

  public Command getAutonomousCommand() {
    return DriveTrajectory.driveTrajectory(drive, BarrelPath.barrelPath);
  }
}
