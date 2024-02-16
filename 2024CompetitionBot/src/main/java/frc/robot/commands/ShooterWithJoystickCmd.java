// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.ShooterSub;

public class ShooterWithJoystickCmd extends Command {
  private CommandPS4Controller m_controller;
  private ShooterSub m_shooterSub;

  /** Creates a new ShooterWithJoystickCmd. */
  public ShooterWithJoystickCmd(CommandPS4Controller controller, ShooterSub shooterSub) {
    m_shooterSub = shooterSub;
    m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterSub.spinFlywheel(m_controller.getRightY());
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
