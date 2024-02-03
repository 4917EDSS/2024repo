// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSub;

/** An example command that uses an example subsystem. */
public class IntakeIntoLimitCmd extends Command {
  private IntakeSub m_intakeSub;

  /**
   * Creates a new ExampleCommand.
   *
   * @param The subsystem used by this command.
   */
  public IntakeIntoLimitCmd(IntakeSub intakeSub) {
    m_intakeSub = intakeSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSub.setIntakeMotors(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSub.setIntakeMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intakeSub.isNoteFullyIn();
  }
}

