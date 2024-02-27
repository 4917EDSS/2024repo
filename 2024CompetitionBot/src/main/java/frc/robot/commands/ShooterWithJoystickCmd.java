// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.IntakeSub;

public class ShooterWithJoystickCmd extends Command {
  private CommandPS4Controller m_controller;
  private ShooterSub m_shooterSub;
  private IntakeSub m_intakeSub;

  /** Creates a new ShooterWithJoystickCmd. */
  public ShooterWithJoystickCmd(CommandPS4Controller controller, ShooterSub shooterSub, IntakeSub intakeSub) {
    m_shooterSub = shooterSub;
    m_controller = controller;
    m_intakeSub = intakeSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSub, intakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get controller joystick value
    double pivotPower = -m_controller.getLeftY();
    double intakePower = -m_controller.getRightY();

    // create deadband if power is less than 5%
    if(Math.abs(pivotPower) < 0.05) {
      pivotPower = 0;
    }
    if(Math.abs(intakePower) < 0.05) {
      intakePower = 0;
    }
    // square power value to give more control when moving slower
    pivotPower *= pivotPower;
    intakePower *= intakePower;

    // set movePivot with the new power
    m_shooterSub.movePivot(pivotPower);
    m_shooterSub.spinBothFeeders(intakePower, intakePower / 2);
    m_intakeSub.setIntakeMotors(intakePower);
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
