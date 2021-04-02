// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import frc.robot.Constants.MotorControllers;
import frc.robot.Constants.PneumaticConstants;

public class Intake extends SubsystemBase {
  CANSparkMax m_roller;
  DoubleSolenoid m_solenoid;

  /** Creates a new Intake. */
  public Intake() {
    m_roller = new CANSparkMax(MotorControllers.INTAKE, MotorType.kBrushless);
    m_solenoid = new DoubleSolenoid( PneumaticConstants.IntakeSolenoidModule,
      PneumaticConstants.IntakeSolenoidPorts[0],PneumaticConstants.IntakeSolenoidPorts[1]);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intakeRun(double speed) {
    m_roller.set(speed);
  }

  public void intakeOut() {
    m_solenoid.set(kForward);
  }
  
  public void intakeIn() {
    m_solenoid.set(kReverse);
  }

  public void stop() {
    m_roller.set(0);
  }
}
