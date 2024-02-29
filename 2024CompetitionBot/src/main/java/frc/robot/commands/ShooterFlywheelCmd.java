// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.time.Duration;
import java.time.Instant;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSub;

public class ShooterFlywheelCmd extends Command {

  private final FlywheelSub m_flywheelSub;

  /** Creates a new FlywheelCmd. */
  public ShooterFlywheelCmd(FlywheelSub flywheelSub) {
    m_flywheelSub = flywheelSub;

    addRequirements(flywheelSub);
  }

  // Called when the command is initially scheduled. 
  @Override
  public void initialize() {
    m_flywheelSub.enableFlywheel();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_flywheelSub.isAtTargetVelocity();
  }
}
