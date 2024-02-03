// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.logging.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSub;

public class ShooterUpperFeederCmd extends Command {
  private static Logger m_logger = Logger.getLogger(ShooterUpperFeederCmd.class.getName());

  private final ShooterSub m_ShooterSub;
  private final boolean m_forward;

  /** Creates a new FeederRollersCmd. */
  public ShooterUpperFeederCmd(ShooterSub shooterSub, boolean forward) {
    m_forward = forward;
    // Use addRequirements() here to declare subsystem dependencies.
    m_ShooterSub = shooterSub;
    addRequirements(shooterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int direction = (m_forward == true) ? 1 : -1; //if moving forward keep going forward, else multiply direction to -1
    m_ShooterSub.spinUpperFeeder(0.10 * direction);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_ShooterSub.isNoteAtPosition();
  }
}
