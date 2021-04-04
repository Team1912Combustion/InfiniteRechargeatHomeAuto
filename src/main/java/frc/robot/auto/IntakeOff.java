// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeOff extends CommandBase {
  private Intake m_intake;

  public IntakeOff(Intake intake) {
    m_intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    m_intake.intakeRun(0);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_intake.intakeRun(0);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
