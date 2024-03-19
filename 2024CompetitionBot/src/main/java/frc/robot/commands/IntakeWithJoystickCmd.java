// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.FeederSub;

public class IntakeWithJoystickCmd extends Command {
  private final CommandPS4Controller m_controller;
  private final FeederSub m_feederSub;

  public IntakeWithJoystickCmd(CommandPS4Controller controller, FeederSub feederSub) {
    m_controller = controller;
    m_feederSub = feederSub;

    addRequirements(feederSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double intakePower = -m_controller.getRightY();

    if(Math.abs(intakePower) < 0.05) {
      intakePower = 0;
    }
    // square power value to give more control when moving slower
    intakePower *= Math.abs(intakePower);

    // set movePivot with the new power
    m_feederSub.spinBothFeeders(intakePower, intakePower);
    m_feederSub.setIntakeMotors(intakePower);
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
