// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.time.Duration;
import java.time.Instant;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSub;

public class ExpelAmpNoteCmd extends Command {
  private final FeederSub m_feederSub;
  private Instant start;

  public ExpelAmpNoteCmd(FeederSub feederSub) {
    m_feederSub = feederSub;

    addRequirements(feederSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_feederSub.spinBothFeeders(-0.5, -0.5);
    start = Instant.now();
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
    return Duration.between(start, Instant.now()).toMillis() > 800;
  }
}
