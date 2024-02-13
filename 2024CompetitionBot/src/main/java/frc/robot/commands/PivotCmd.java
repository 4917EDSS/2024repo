// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.ShooterSub;

public class PivotCmd extends Command {

  private final CommandPS4Controller m_controller;
  private final ShooterSub m_shooterSub;
  private final double kDeadband = 0.03;

  /** Creates a new PivotCmd. */
  public PivotCmd(CommandPS4Controller controller, ShooterSub shooterSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooterSub = shooterSub;
    m_controller = controller;

    addRequirements(shooterSub);
  }

  private double applyDeadband(double power) {
    if(Math.abs(power) <= kDeadband) {
      power = 0.0;
    }
    return power;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = m_controller.getLeftY();
    applyDeadband(power);
    m_shooterSub.movePivot(power);

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
