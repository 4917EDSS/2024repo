// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSub;

public class ShooterPivotCmd extends Command {

  // PID Controllers

  private final PIDController m_pivotForwardPid = new PIDController(0.04, 0, 0); // TODO: Tune the Pivot PID

  private final ShooterSub m_ShooterSub;
  private final double m_targetPivotPosition;
  private boolean atSetpoint = false;

  /** Creates a new PivotCmd. */
  public ShooterPivotCmd(double targetPivotPosition, ShooterSub shooterSub) {
    m_targetPivotPosition = targetPivotPosition;
    m_ShooterSub = shooterSub;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    atSetpoint = m_ShooterSub.setPivotAngle(m_targetPivotPosition);
    // double driveOutput = m_pivotForwardPid.calculate(m_ShooterSub.getPivotAngle(), m_targetPivotPosition);
    // driveOutput = MathUtil.clamp(driveOutput, -1.0, 1.0);
    // m_ShooterSub.movePivot(driveOutput);
    // System.out.println(driveOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterSub.movePivot(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //double tolerance = 1;
    boolean hitLimit = (m_ShooterSub.getPivotPower() < 0.0) ? m_ShooterSub.isPivotAtReverseLimit()
        : m_ShooterSub.isPivotAtForwardLimit(); // Emergency case to stop command
    return atSetpoint || hitLimit;
  }
}
