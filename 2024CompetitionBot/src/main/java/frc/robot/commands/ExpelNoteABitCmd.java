// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArduinoSub;
import frc.robot.subsystems.FeederSub;

public class ExpelNoteABitCmd extends Command {
  private final FeederSub m_feederSub;
  private final ArduinoSub m_arduinoSub;

  /** Creates a new IntakeUntilNoteInCmd. */
  public ExpelNoteABitCmd(FeederSub feederSub, ArduinoSub arduinoSub) {
    m_feederSub = feederSub;
    m_arduinoSub = arduinoSub;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feederSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_feederSub.spinBothFeeders(Constants.Shooter.kNoteLowerExpellPower, Constants.Shooter.kNoteUpperExpellPower);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
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
    if(m_arduinoSub.isSensorTripped(Constants.Shooter.kNoteSensorFwFar)) {
      return true;
    }
    return false;
  }
}
