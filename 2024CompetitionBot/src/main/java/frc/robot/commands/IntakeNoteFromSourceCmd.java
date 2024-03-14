// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.logging.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArduinoSub;
import frc.robot.subsystems.FeederSub;
import frc.robot.subsystems.LedSub;
import frc.robot.subsystems.LedSub.LedColour;
import frc.robot.subsystems.LedSub.LedZones;


public class IntakeNoteFromSourceCmd extends Command {
  private static Logger m_logger = Logger.getLogger(IntakeNoteFromSourceCmd.class.getName());

  private final FeederSub m_feederSub;
  private final ArduinoSub m_arduinoSub;
  private final LedSub m_LedSub;

  /** Creates a new IntakeUntilNoteInCmd. */
  public IntakeNoteFromSourceCmd(FeederSub feederSub, ArduinoSub arduinoSub, LedSub ledSub) {
    m_feederSub = feederSub;
    m_arduinoSub = arduinoSub;
    m_LedSub = ledSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feederSub, ledSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_logger.fine("IntakeUntilNoteInCmd - Init");

    m_feederSub.spinBothFeeders(Constants.Shooter.kNoteLowerIntakePower, Constants.Shooter.kNoteUpperIntakePower);
    m_LedSub.setZoneColour(LedZones.ALL, LedColour.RED);
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
    m_feederSub.spinBothFeeders(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Once we hit the last sensor, stop the feed rollers
    // if(m_arduinoSub.isSensorTripped(Constants.Shooter.kNoteSensorAtFlywheel)) {
    //   return true;
    // }
    if(m_arduinoSub.isSensorTripped(Constants.Shooter.kNoteSensorFwNear)) {
      return true;
    }
    return false;
  }
}
