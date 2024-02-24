// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSub;
import frc.robot.subsystems.ShooterSub;

public class MovingAngleCmd extends Command {
  private final PIDController m_pivotForwardPid = new PIDController(0.04, 0, 0);
  private final VisionSub m_visionSub;
  private final ShooterSub m_shooterSub;

  /** Creates a new MovingAngle. */
  public MovingAngleCmd(VisionSub visionSub, ShooterSub shooterSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_visionSub = visionSub;
    m_shooterSub = shooterSub;
    addRequirements(visionSub, shooterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = m_visionSub.getHorizontalAngle();
    double driveOutput = m_pivotForwardPid.calculate(m_shooterSub.getPivotAngle(), angle);
    driveOutput = MathUtil.clamp(driveOutput, -1.0, 1.0);
    m_shooterSub.movePivot(driveOutput);
    System.out.println(driveOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
