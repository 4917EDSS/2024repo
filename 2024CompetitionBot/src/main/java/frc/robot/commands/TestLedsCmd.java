// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LedSub;
import frc.robot.subsystems.LedSub.LedColour;
import frc.robot.subsystems.LedSub.LedZones;

/** An example command that uses an example subsystem. */
public class TestLedsCmd extends Command {
  private final LedColour m_ledColour;
  private final LedSub m_ledSub;

  public TestLedsCmd(LedColour colour, LedSub LedSub) {
    m_ledColour = colour;
    m_ledSub = LedSub;

    addRequirements(LedSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ledSub.setZoneColour(LedZones.ALL, m_ledColour);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

