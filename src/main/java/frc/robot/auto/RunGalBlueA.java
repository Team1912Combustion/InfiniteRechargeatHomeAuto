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
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class RunGalBlueA extends SequentialCommandGroup {

  Trajectory trajectory;

  public RunGalBlueA(DriveTrain drivetrain, Intake intake) {
    String myPathName = "";
    String trajectoryfile = "";

    myPathName = "GalBlueA";

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
      new IntakeOut(intake),
      new IntakeOn(intake),
      m_ramsetecommand.andThen(() -> drivetrain.tankDriveVolts(0,0)),
      new IntakeOff(intake),
      new IntakeIn(intake)
    );
  }
}
