// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.logging.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSub;

public class ShooterFlywheelCmd extends Command {
  private static Logger m_logger = Logger.getLogger(ShooterFlywheelCmd.class.getName());

  private final FlywheelSub m_flywheelSub;

  /** Creates a new FlywheelCmd. */
  public ShooterFlywheelCmd(FlywheelSub flywheelSub) {
    m_flywheelSub = flywheelSub;

    addRequirements(flywheelSub);
  }

  // Called when the command is initially scheduled. 
  @Override
  public void initialize() {
    m_logger.fine("ShooterFlywheelCmd - Init");
    m_flywheelSub.enableFlywheel();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_logger.fine("ShooterFlywheelCmd - End" + (interrupted ? " (interrupted)" : ""));
    // TODO: Set shoot status LEDs to ready
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_flywheelSub.isAtTargetVelocity();
  }
}
