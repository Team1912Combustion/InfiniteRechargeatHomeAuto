// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

public final class Constants {
public static final class DriveConstants {
    public static final int LeftFront = 3;
    public static final int LeftRear = 4;
    public static final int RightFront = 5;
    public static final int RightRear = 6;
    public static final double GearRatio = 10.71;
    
    public static final double TrackwidthMeters = 0.5589;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(TrackwidthMeters);

    // Uses the integrated Falcon 500 Encoders
    public static final int EncoderTPR = 2048;
    public static final double WheelDiameterMeters = Units.inchesToMeters(6.0);
    public static final double EncoderDistancePerPulse = (WheelDiameterMeters * Math.PI)/ GearRatio / (double) EncoderTPR;
    public static final double WheelCircumferenceMeters = WheelDiameterMeters*Math.PI;
    public static final double Conversion = WheelCircumferenceMeters/(GearRatio*EncoderTPR);

    public static final double kS = 0.658;
    public static final double kV = 2.48;
    public static final double kA = 0.256;
    public static final double kP = 0; //2.26;
  }

  public static final class OIConstants {
    public static final int DriverControllerPort = 0;
  }

  public static final class AutoConstants {
    // moved these to the individual waypoint paths for tuning
    //public static final double MaxCentripetal = 1.;
    //public static final double MaxSpeedMetersPerSecond = 1.0;
    //public static final double MaxAccelerationMetersPerSecondSquared = 2.0;

    public static final double kRamseteB = 2.0;
    public static final double kRamseteZeta = 0.7;
  }

  public static final class PneumaticConstants{
    public static final int IntakeSolenoidModule = 0;
    public static final int[] IntakeSolenoidPorts = new int[] { 0, 1 };
  }

  public final class MotorControllers{
    public static final int INTAKE = 4;
    public static final int ELEVATOR_1 = 9;
    public static final int ELEVATOR_2 = 10;
  }
  public final class MotorSpeeds{
    public static final double ELEVATOR_SPEED = .6;
  }

}
