// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArduinoSub;
import frc.robot.subsystems.FeederSub;

public class ReintakeAmpNoteCmd extends Command {
  /** Creates a new ReintakeAmpNoteCmd. */

  private final ArduinoSub m_arduinoSub;
  private final FeederSub m_feederSub;

  public ReintakeAmpNoteCmd(ArduinoSub arduinoSub, FeederSub feederSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arduinoSub, feederSub);

    m_arduinoSub = arduinoSub;
    m_feederSub = feederSub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_feederSub.spinBothFeeders(0.6, 0.6);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feederSub.spinBothFeeders(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_arduinoSub.isSensorTripped(Constants.Shooter.kNoteSensorFwFar)
        || m_arduinoSub.isSensorTripped(Constants.Shooter.kNoteSensorFwMid));
  }
}
