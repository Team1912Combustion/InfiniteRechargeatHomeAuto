// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorControllers;
import frc.robot.Constants.MotorSpeeds;

public class Elevator extends SubsystemBase {
  WPI_VictorSPX elevator1 = new WPI_VictorSPX(MotorControllers.ELEVATOR_1);
  WPI_VictorSPX elevator2 = new WPI_VictorSPX(MotorControllers.ELEVATOR_2);
  SpeedControllerGroup elevatorGroup = new SpeedControllerGroup(elevator1, elevator2);
  private DigitalInput sensor0 = new DigitalInput(0); 
  private DigitalInput sensor1 = new DigitalInput(1);

  /** Creates a new Elevator. */
  public Elevator() {
    elevator2.setInverted(true);
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
  public void elevateBall(double speed) {
    elevatorGroup.set(speed);
  }
  public void lowerBall(double speed) {
    elevatorGroup.set(speed);
  }
  public void moveElevator(double speed) {
    if (sensor1.get()) {
      if(! sensor0.get()) {
        elevator1.set(MotorSpeeds.ELEVATOR_SPEED);
        elevator2.set(MotorSpeeds.ELEVATOR_SPEED);
        // figure out which elevator is the back and add 25 percent to that elevator's speed called BACK_ELEVATOR_SPEED
        System.out.println("I'm working!!");
     }  else {
       elevatorGroup.set(0);
     }
    } else {
      elevatorGroup.set(0);
    }
  }
  public void stop() {
    elevatorGroup.set(0);
  }
}
