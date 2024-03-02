// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSub;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSub;


public class IntakeReverseWhenNoteInCmd extends Command {
  private final ShooterSub m_shooterSub;
  private final IntakeSub m_intakeSub;

  /** Creates a new IntakeReverseWhenNoteInCmd. */
  public IntakeReverseWhenNoteInCmd(ShooterSub shooterSub, IntakeSub intakeSub) {
    m_shooterSub = shooterSub;
    m_intakeSub = intakeSub;

    addRequirements(shooterSub, intakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSub.setIntakeMotors(Constants.Intake.kNoteExpelPower);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_shooterSub.isNoteAtPosition(Constants.Shooter.kNoteSensorNearFlywheel);
  }
}
