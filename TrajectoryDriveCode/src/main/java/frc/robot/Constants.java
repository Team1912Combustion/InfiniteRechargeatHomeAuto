// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
public static final class DriveConstants {
    public static final int LeftFront = 3;
    public static final int LeftRear = 4;
    public static final int RightFront = 5;
    public static final int RightRear = 6;
    public static final double GearRatio = 10.75;

    public static final double TrackwidthMeters = Units.inchesToMeters(22.25);
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(TrackwidthMeters);

    public static final int EncoderTPR = 2048;
    public static final double WheelDiameterMeters = Units.inchesToMeters(6.0);
    public static final double EncoderDistancePerPulse =
        // Uses the integrated Falcon 500 Encoders
        (WheelDiameterMeters * Math.PI)/ GearRatio / (double) EncoderTPR;
    public static final double WheelCircumferenceMeters = WheelDiameterMeters*Math.PI;
    public static final double Conversion = WheelCircumferenceMeters/(GearRatio*EncoderTPR);
    

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double kS = 0.634;
    public static final double kV = 1.15;
    public static final double kA = 0.11;
    public static final double kP = 4;
    // Example value only - as above, this must be tuned for your drive!
    //public static final double PDriveVel = 8.5;
    public static final double PDriveVel = 8.5;
	
  }

  public static final class OIConstants {
    public static final int DriverControllerPort = 0;
  }

  public static final class AutoConstants {
    //public static final double MaxSpeedMetersPerSecond = 2;
    //public static final double MaxAccelerationMetersPerSecondSquared = 2;
    public static final double MaxSpeedMetersPerSecond = 1.0;
    public static final double MaxAccelerationMetersPerSecondSquared = .25;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }
}
