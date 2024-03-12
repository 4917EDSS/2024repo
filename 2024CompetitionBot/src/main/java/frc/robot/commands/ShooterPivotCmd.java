// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.logging.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSub;

public class ShooterPivotCmd extends Command {
  private static Logger m_logger = Logger.getLogger(ShooterPivotCmd.class.getName());

  private final ShooterSub m_shooterSub;
  private final double m_targetPivotPosition;

  /** Creates a new PivotCmd. */
  public ShooterPivotCmd(double targetPivotPosition, ShooterSub shooterSub) {
    m_targetPivotPosition = targetPivotPosition;
    m_shooterSub = shooterSub;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_logger.fine("ShooterPivotCmd - Init");
    m_shooterSub.setTargetAngle(m_targetPivotPosition);
    m_shooterSub.runPivotControl(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_logger.fine("ShooterPivotCmd - End" + (interrupted ? " (interrupted)" : ""));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean hitLimit = (m_shooterSub.getPivotPower() < 0.0) ? m_shooterSub.isPivotAtReverseLimit()
        : m_shooterSub.isPivotAtForwardLimit(); // Emergency case to stop command

    return m_shooterSub.isAtPivotAngle() || hitLimit;
  }
}
