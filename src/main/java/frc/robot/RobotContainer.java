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

  public Command getAutonomousCommand(int iPathID) {

    System.out.println("In RobotContainer...");
    System.out.println("...choose iPathID = " + Integer.toString(iPathID) + "...");

    switch (iPathID) {
      case 0:
        System.out.println("... run RunGalBlueA");
        return new RunGalBlueA(drive, intake);
      case 1:
        System.out.println("... run RunGalRedA");
        return new RunGalRedA(drive, intake);
      case 2:
        System.out.println("... run RunGalBlueB");
        return new RunGalBlueB(drive, intake);
      case 3:
        System.out.println("... run RunGalRedB");
        return new RunGalRedB(drive, intake);
      default:
        System.out.println("... no default value - run IntakeOff - this should be boring");
        return new IntakeOff(intake);
    }

    //return autoChooser.getSelected();
    //return new RunBarrel(drive);
    //return new RunGalBlueB(drive, intake);
    //return new RunSlalom(drive);
  }
}
