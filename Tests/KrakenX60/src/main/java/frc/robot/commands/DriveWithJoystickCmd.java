// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.KrakenSub;

public class DriveWithJoystickCmd extends Command {
  private final CommandPS4Controller m_controller;
  private final KrakenSub m_krakenSub;

  /** Creates a new DriveWithJoystickCmd. */
  public DriveWithJoystickCmd(CommandPS4Controller controller, KrakenSub krakenSub) {
    m_controller = controller;
    m_krakenSub = krakenSub;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(krakenSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = -m_controller.getLeftY();
    int sign = 1;

    if(Math.abs(power) < 0.05) {
      power = 0;
    }

    if(power < 0.0) {
      sign = -1;
      power *= -1.0;
    }
    power = Math.pow(power, 2) * sign;

    m_krakenSub.runMotor(power);
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
