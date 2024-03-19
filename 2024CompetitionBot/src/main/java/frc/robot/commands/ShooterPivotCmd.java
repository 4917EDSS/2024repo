// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.logging.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSub;

public class ShooterPivotCmd extends Command {
  private static Logger m_logger = Logger.getLogger(ShooterPivotCmd.class.getName());

  private final ShooterSub m_shooterSub;
  private final double m_targetPivotPosition;
  private final double m_maxSpeed;

  public ShooterPivotCmd(double targetPivotPosition, double maxSpeed, ShooterSub shooterSub) {
    m_targetPivotPosition = targetPivotPosition;
    m_maxSpeed = maxSpeed;
    m_shooterSub = shooterSub;

    addRequirements(shooterSub);
  }

  public ShooterPivotCmd(double targetPivotPosition, ShooterSub shooterSub) {
    this(targetPivotPosition, 1.0, shooterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_logger.fine("ShooterPivotCmd - Init");
    if(m_targetPivotPosition != 0) {
      m_shooterSub.setTargetAngle(m_targetPivotPosition);
      m_shooterSub.runPivotControl(true);
    } else {
      m_shooterSub.disableTargetAngle();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_targetPivotPosition == 0) {
      m_shooterSub.movePivot(-m_maxSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_logger.fine("ShooterPivotCmd - End" + (interrupted ? " (interrupted)" : ""));
    m_shooterSub.movePivot(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean hitLimit = (m_shooterSub.getPivotPower() < 0.0) ? m_shooterSub.isPivotAtReverseLimit()
        : m_shooterSub.isPivotAtForwardLimit(); // Emergency case to stop command
    if(m_targetPivotPosition != 0) {
      return m_shooterSub.isAtPivotAngle() || hitLimit;
    } else {
      return hitLimit;
    }
  }
}
