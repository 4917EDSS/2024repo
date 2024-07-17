// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.MotorControlSub;

public class JoystickCmd extends Command {
  /** Creates a new JoystickCmd. */
  private final CommandPS4Controller m_controller;
  private final MotorControlSub m_motorControlSub;

  public JoystickCmd(CommandPS4Controller controller, MotorControlSub motorControlSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_controller = controller;
    m_motorControlSub = motorControlSub;

    addRequirements(motorControlSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double motor1JoystickPower = m_controller.getLeftY();
    double motor2JoystickPower = m_controller.getRightY();

    if(Math.abs(motor1JoystickPower) < 0.05) {
      motor1JoystickPower = 0;
    } else {
      m_motorControlSub.setPowerMotor1(motor1JoystickPower);
    }

    if(Math.abs(motor2JoystickPower) < 0.05) {
      motor2JoystickPower = 0;
    } else {
      m_motorControlSub.setPowerMotor2(motor2JoystickPower);
    }
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
