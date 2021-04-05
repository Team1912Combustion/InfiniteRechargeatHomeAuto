// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private NetworkTableEntry pathID;
  private NetworkTable vision;
  private NetworkTableInstance inst;
  private int iPathID = 0;

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    inst = NetworkTableInstance.getDefault();
    vision = inst.getTable("Vision");
    pathID = vision.getEntry("pathID");
    System.out.println(pathID.getName());
    System.out.println(pathID.getType().toString());
    System.out.println(pathID.getInfo().toString());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    double dPathID = 0;
    dPathID = pathID.getDouble(-1.);
    iPathID = (int) Math.round(dPathID);
    SmartDashboard.putNumber("iPathID", iPathID);
    SmartDashboard.putString("PathID name", pathID.getName());
    //System.out.println("Have iPathID = " + Integer.toString(iPathID) + "...");
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand(iPathID);

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}
}
