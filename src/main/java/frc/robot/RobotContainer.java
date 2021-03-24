// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.auto.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

public class RobotContainer {
  private DriveTrain drive = new DriveTrain();
  private Trajectory trajectory = new Trajectory();

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  
  public RobotContainer() {
    configureButtonBindings();
    autoChooser.setDefaultOption("Barrel", RunBarrel());
    autoChooser.addOption("Barrel", RunBarrel());
    autoChooser.addOption("Slalom", RunSlalom());
    autoChooser.addOption("Bounce", new RunBounce());
    autoChooser.addOption("BarrelWP", RunBarrelWP());
    autoChooser.addOption("SlalomWP", RunSlalomWP());
    SmartDashboard.putData("Auto Chooser", autoChooser); 
  }

  private void configureButtonBindings() {}

  public Command RunBarrel() {
    trajectory = BarrelPath.getTrajectory();
    return DriveTrajectory.driveTrajectory(drive, trajectory);
  }

  public Command RunSlalom() {
    trajectory = SlalomPath.getTrajectory();
    return DriveTrajectory.driveTrajectory(drive, trajectory);
  }

  public Command RunBarrelWP() {
    trajectory = BarrelPathWaypoints.getTrajectory();
    return DriveTrajectory.driveTrajectory(drive, trajectory);
  }

  public Command RunSlalomWP() {
    trajectory = SlalomPathWaypoints.getTrajectory();
    return DriveTrajectory.driveTrajectory(drive, trajectory);
  }

  public class RunBounce extends SequentialCommandGroup {
    public RunBounce() {
      addCommands(
        driveBounce1(),
        driveBounce2(),
        driveBounce3(),
        driveBounce4()
        );
    }
    public Command driveBounce1() {
      trajectory = BouncePaths.getTrajectory(1);
      return DriveTrajectory.driveTrajectory(drive, trajectory);
    }
    public Command driveBounce2() {
      trajectory = BouncePaths.getTrajectory(2);
      return DriveTrajectory.driveTrajectory(drive, trajectory);
    }
    public Command driveBounce3() {
      trajectory = BouncePaths.getTrajectory(3);
      return DriveTrajectory.driveTrajectory(drive, trajectory);
    }
    public Command driveBounce4() {
      trajectory = BouncePaths.getTrajectory(4);
      return DriveTrajectory.driveTrajectory(drive, trajectory);
    }
  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }

}
