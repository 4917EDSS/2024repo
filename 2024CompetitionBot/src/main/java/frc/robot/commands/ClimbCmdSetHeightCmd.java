// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSub;
import frc.robot.subsystems.DrivetrainSub;


public class ClimbCmdSetHeightCmd extends Command {
  private final ClimbSub m_ClimbSub;
  double m_targetHeight;


  /** Creates a new Climb. */
  public ClimbCmdSetHeightCmd(ClimbSub climbSub, double position) {
    m_ClimbSub = climbSub;

    m_targetHeight = position;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //we need to change the hight in metters to how much it gose up in metters per full rotation

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //set power
    double LeftEncoderValue = ClimbSub.getLeftHeight();
    if(isLeftAtTargetHeight() == false)
      m_ClimbSub.setClimbPowerLeft(5);

    double RightEncoderValue = ClimbSub.getRightHeight();
    if(isRightAtTargetHeight() == false)
      m_ClimbSub.setClimbPowerRight(5);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (isLeftAtTargetHeight() && isRightAtTargetHeight());
  }

  private boolean isLeftAtTargetHeight() {
    double LeftEncoderValue = ClimbSub.getLeftHeight();
    return (LeftEncoderValue >= m_targetHeight);
  }

  private boolean isRightAtTargetHeight() {
    double RightEncoderValue = ClimbSub.getRightHeight();
    return (RightEncoderValue >= m_targetHeight);
  }
}

