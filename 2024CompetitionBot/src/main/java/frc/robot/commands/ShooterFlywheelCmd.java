// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSub;

public class ShooterFlywheelCmd extends Command {
  private final ShooterSub m_ShooterSub;

  /** Creates a new FlywheelCmd. */
  public ShooterFlywheelCmd(ShooterSub shooterSub) {

    // Use addRequirements() here to declare subsystem dependencies.
    m_ShooterSub = shooterSub;
    addRequirements(shooterSub);
  }


  // Called when the command is initially scheduled. 
  @Override
  public void initialize() {
    m_ShooterSub.resetFlywheel();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ShooterSub.spinFlywheel(10);
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
