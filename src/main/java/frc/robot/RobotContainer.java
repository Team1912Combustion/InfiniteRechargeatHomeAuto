// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.auto.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
  private DriveTrain drive = new DriveTrain();
  private Intake intake = new Intake();

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  
  public RobotContainer() {
    configureButtonBindings();
    autoChooser.setDefaultOption("BarrelWP", new RunBarrelWP(drive));
    autoChooser.addOption("SlalomWP", new RunSlalomWP(drive));
    autoChooser.addOption("Barrel", new RunBarrel(drive));
    autoChooser.addOption("Slalom", new RunSlalom(drive));
    autoChooser.addOption("Bounce", new RunBounce(drive));
    autoChooser.addOption("GalRedA", new RunGalRedA(drive, intake));
    autoChooser.addOption("GalRedB", new RunGalRedB(drive, intake));
    autoChooser.addOption("GalBlueA", new RunGalBlueA(drive, intake));
    autoChooser.addOption("GalBlueB", new RunGalBlueB(drive, intake));
    autoChooser.addOption("GalRedA", new RunGalRedA_WP(drive, intake));
    autoChooser.addOption("GalRedB", new RunGalRedB_WP(drive, intake));
    autoChooser.addOption("GalBlueA", new RunGalBlueA_WP(drive, intake));
    autoChooser.addOption("GalBlueB", new RunGalBlueB_WP(drive, intake));
    SmartDashboard.putData("Auto Chooser", autoChooser); 
  }

  private void configureButtonBindings() {}

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return autoChooser.getSelected();
    //return new RunBarrel(drive);
    return new RunGalRedA(drive, intake);
    //return new RunSlalom(drive);
  }
}
