// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.FeederSub;
import java.time.Duration;
import java.time.Instant;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeNoteFeederCmd extends Command {


  private final FeederSub m_feederSub;
  private Instant start;

  /** Creates a new IntakeNoteFeederCmd. */
  public IntakeNoteFeederCmd(FeederSub feederSub) {
    m_feederSub = feederSub;
    start = Instant.now();
    addRequirements(feederSub); //here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_feederSub.spinBothFeeders(Constants.Shooter.kNoteLowerIntakePower, Constants.Shooter.kNoteUpperIntakePower);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Instant end = Instant.now();
    Duration timeElapsed = Duration.between(start, end);
    return timeElapsed.toMillis() > 250;
  }
}
