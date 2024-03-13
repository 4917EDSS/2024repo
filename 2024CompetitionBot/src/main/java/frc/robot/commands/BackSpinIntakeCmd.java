// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.logging.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArduinoSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.LedSub;


public class BackSpinIntakeCmd extends Command {
  private static Logger m_logger = Logger.getLogger(IntakeUntilNoteInCmd.class.getName());

  private final IntakeSub m_intakeSub;

  /** Creates a new IntakeUntilNoteInCmd. */
  public BackSpinIntakeCmd(IntakeSub intakeSub) {
    m_intakeSub = intakeSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_logger.fine("IntakeUntilNoteInCmd - Init");

    m_intakeSub.setIntakeMotors(Constants.Intake.kNoteExpelPower);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Once the note has cleared the intake rollers, run those rollers in reverse to avoid controlling two notes
    // if(m_shooterSub.isNoteAtPosition(Constants.Shooter.kNoteSensorNearFlywheel)) {
    //   m_intakeSub.setIntakeMotors(Constants.Intake.kNoteExpelPower);
    // }
    // if(m_arduinoSub.isSensorTripped(Constants.Shooter.kNoteSensorFwFar)) {
    //   m_intakeSub.setIntakeMotors(Constants.Intake.kNoteExpelPower);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_logger.fine("IntakeUntilNoteInCmd - End" + (interrupted ? " (interrupted)" : ""));

    // Make sure we're running the intake rollers in reverse and the feed rollers are off
    m_intakeSub.setIntakeMotors(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Once we hit the last sensor, stop the feed rollers
    // if(m_arduinoSub.isSensorTripped(Constants.Shooter.kNoteSensorAtFlywheel)) {
    //   return true;
    // }
    return false;
  }
}