// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.io.IOException;
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

public class RunIROS extends SequentialCommandGroup {

  Trajectory trajectory1;
  Trajectory trajectory2;

  public RunIROS(DriveTrain drivetrain, Intake intake) {
    String myPathName1 = "";
    String myPathName2 = "";
    String trajectoryfile = "";

    myPathName1 = "IROS1";
    myPathName2 = "IROS2";

    trajectoryfile = myPathName1 + ".wpilib.json";
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryfile);
      trajectory1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryfile, ex.getStackTrace());
    }

    trajectoryfile = myPathName2 + ".wpilib.json";
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryfile);
      trajectory2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryfile, ex.getStackTrace());
    }

    drivetrain.resetOdometry(trajectory1.getInitialPose());

    RamseteCommand m_ramsetecommand1 = new RamseteCommand (
      trajectory1,
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

    RamseteCommand m_ramsetecommand2 = new RamseteCommand (
      trajectory2,
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
      m_ramsetecommand1.andThen(() -> drivetrain.tankDriveVolts(0,0)),
      new IntakeOut(intake),
      new IntakeOn(intake),
      m_ramsetecommand2.andThen(() -> drivetrain.tankDriveVolts(0,0)),
      new IntakeOff(intake),
      new IntakeIn(intake)
    );
  }
}
