// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.time.Duration;
import java.time.Instant;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FeederSub;
import frc.robot.subsystems.LedSub;
import frc.robot.subsystems.LedSub.LedColour;
import frc.robot.subsystems.LedSub.LedZones;

public class ExpelAmpNoteCmd extends Command {
  private final FeederSub m_feederSub;
  private final LedSub m_ledSub;

  private Instant start;

  public ExpelAmpNoteCmd(FeederSub feederSub, LedSub ledSub) {
    m_feederSub = feederSub;
    m_ledSub = ledSub;

    addRequirements(feederSub); // It's fine if two commands change LEDs
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_feederSub.finishedNoteIntake = false;
    m_feederSub.spinBothFeeders(Constants.Shooter.kLowerExpelPower, Constants.Shooter.kUpperExpelPower);
    start = Instant.now();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feederSub.spinBothFeeders(0, 0);
    m_ledSub.setZoneColour(LedZones.ALL, LedColour.GREEN);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Duration.between(start, Instant.now()).toMillis() > 500;
  }
}
