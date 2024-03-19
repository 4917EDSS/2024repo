// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.logging.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.ShooterSub;

public class ShooterWithJoystickCmd extends Command {
  private static Logger m_logger = Logger.getLogger(ShooterShootCmd.class.getName());

  private final CommandPS4Controller m_controller;
  private final ShooterSub m_shooterSub;

  private boolean m_wasInDeadZone;

  public ShooterWithJoystickCmd(CommandPS4Controller controller, ShooterSub shooterSub) {
    m_controller = controller;
    m_shooterSub = shooterSub;

    addRequirements(shooterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_logger.fine("ShooterWithJoystick - Init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get controller joystick value
    double pivotPower = -m_controller.getLeftY();

    // create deadband if power is less than 5%
    if(Math.abs(pivotPower) < 0.05) {
      if(!m_wasInDeadZone) {
        m_shooterSub.setTargetAngle((m_shooterSub.getPivotAngle()));
      }
      m_wasInDeadZone = true;
    } else {
      m_wasInDeadZone = false;
      m_shooterSub.disableTargetAngle();
      m_shooterSub.movePivot(pivotPower / 2);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
