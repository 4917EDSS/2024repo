// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.logging.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSub;
import frc.robot.subsystems.ShooterSub;

public class PivotToAprilTagCmd extends Command {
  private static Logger m_logger = Logger.getLogger(PivotToAprilTagCmd.class.getName());

  private final PIDController m_pivotForwardPid = new PIDController(0.04, 0, 0);
  private final VisionSub m_visionSub;
  private final ShooterSub m_shooterSub;

  /** Creates a new PivotToAprilTagCmd. */
  public PivotToAprilTagCmd(VisionSub visionSub, ShooterSub shooterSub) {
    m_visionSub = visionSub;
    m_shooterSub = shooterSub;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(visionSub, shooterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_logger.fine("PivotToAprilTagCmd - Init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = m_visionSub.getHorizontalAngle();
    double driveOutput = m_pivotForwardPid.calculate(m_shooterSub.getPivotAngle(), angle);
    driveOutput = MathUtil.clamp(driveOutput, -1.0, 1.0);
    m_shooterSub.movePivot(driveOutput);
    m_logger.info("Pivot drive power = " + driveOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_logger.fine("PivotToAprilTagCmd - End" + (interrupted ? " (interrupted)" : ""));
    m_shooterSub.movePivot(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
