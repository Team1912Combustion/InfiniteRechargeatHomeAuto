// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
WPI_TalonFX leftFront = new WPI_TalonFX(DriveConstants.LeftFront);
WPI_TalonFX leftRear = new WPI_TalonFX(DriveConstants.LeftRear);
WPI_TalonFX rightFront = new WPI_TalonFX(DriveConstants.RightFront);
WPI_TalonFX rightRear = new WPI_TalonFX(DriveConstants.RightRear);
SpeedControllerGroup leftSide = new SpeedControllerGroup(leftFront, leftRear);
SpeedControllerGroup rightSide = new SpeedControllerGroup(rightFront, rightRear);
DifferentialDrive drive = new DifferentialDrive(leftSide, rightSide);
AHRS ahrs = new AHRS(SPI.Port.kMXP);
DifferentialDriveOdometry odometry;

 public DriveTrain() {
  leftFront.setInverted(false);
  leftRear.setInverted(false);
  rightFront.setInverted(false);
  rightRear.setInverted(false);    
  }

  @Override
  public void periodic() {
    odometry.update(myRotation(), leftFront.getSelectedSensorPosition()*DriveConstants.Conversion,
    rightFront.getSelectedSensorPosition()*DriveConstants.Conversion);
        // This method will be called once per scheduler run
  }
  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(leftFront.getSelectedSensorVelocity()*DriveConstants.Conversion,
    rightFront.getSelectedSensorVelocity()*DriveConstants.Conversion);
  }

  public void resetOdometry(Pose2d pose){
    resetEncoders();
    odometry.resetPosition(pose, ahrs.getRotation2d());
  }

  public void arcadeDrive (double fwd, double rot){
    drive.arcadeDrive(fwd, rot);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts){
    leftSide.setVoltage(leftVolts);
    rightSide.setVoltage(rightVolts);
    drive.feed();
  }

  public void resetEncoders(){
    leftFront.setSelectedSensorPosition(0);
    rightFront.setSelectedSensorPosition(0);
  }

  public double getAverageEncoderDistance(){
    return ((leftFront.getSelectedSensorPosition()+rightFront.getSelectedSensorPosition())/2.0)
    *DriveConstants.Conversion;
  }

  /**
   * Set the max output of the drive. 
   * @param setMaxOutput
   */
  public void setMaxOutput(double maxOutput){
    drive.setMaxOutput(maxOutput);
      }

  public void zeroHeading(){
    //ahrs.reset();
    ahrs.zeroYaw();
  }

  public double getHeading(){
    //return Math.IEEEremainder(ahrs.getAngle(), 350).getRotation2d().getDegrees();
    return Math.IEEEremainder(ahrs.getAngle(), 350);
  }

  public Rotation2d myRotation(){
    return Rotation2d.fromDegrees(ahrs.getYaw());
  }
  public double getTurnRate(){
    return ahrs.getRate();
  }
}
