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

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class RunGalRedA extends SequentialCommandGroup {

  Trajectory trajectory;

  public RunGalRedA(DriveTrain drivetrain, Intake intake) {
    String myPathName = "";
    String trajectoryfile = "";

    myPathName = "GalRedA";

    trajectoryfile = myPathName + ".wpilib.json";
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

    drivetrain.resetOdometry(trajectory.getInitialPose());

    RamseteCommand m_ramsetecommand = new RamseteCommand (
      trajectory,
      drivetrain::getPose,
      new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
      new SimpleMotorFeedforward(
        DriveConstants.kS,
        DriveConstants.kV,
        DriveConstants.kA),
      DriveConstants.kDriveKinematics,
      drivetrain::getWheelSpeeds,
      new PIDController(DriveConstants.kP, 0, 0),
      new PIDController(DriveConstants.kP, 0, 0),
      drivetrain::tankDriveVolts,
      drivetrain
      );

    addCommands(
      new PrintCommand("put Intake Out"),
      new IntakeOut(intake),
      new PrintCommand("turn Intake On"),
      new IntakeOn(intake),
      new PrintCommand("run path"),
      m_ramsetecommand.andThen(() -> drivetrain.tankDriveVolts(0,0)),
      new PrintCommand("turn Intake Off"),
      new IntakeOff(intake),
      new PrintCommand("bring Intake In"),
      new IntakeIn(intake)
      );

    /* !!!
    //alternative to test if IntakeOn (set the rooler speed to 0.8) needs to run in parallel with the drive
    addCommands(
      new IntakeOut(intake),
      parallel(new IntakeOn(intake), m_ramsetecommand.andThen(() -> drivetrain.tankDriveVolts(0,0))),
      new IntakeOff(intake),
      new IntakeIn(intake)
      );
    !!! */

  }
}
