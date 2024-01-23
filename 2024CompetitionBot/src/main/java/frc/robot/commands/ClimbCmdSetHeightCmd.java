// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSub;
import frc.robot.subsystems.DrivetrainSub;


public class ClimbCmdSetHeightCmd extends Command {
  private final ClimbSub m_climbSub;
  private final DrivetrainSub m_drivetrainSub;
  private double m_targetHeight;

  private final PIDController m_pivotForwardPid = new PIDController(1.0, 0, 0); // TODO: Tune the Driving PID

  /** Creates a new Climb. */
  public ClimbCmdSetHeightCmd(ClimbSub climbSub, double height, DrivetrainSub drivetrainSub) {
    m_climbSub = climbSub;
    m_drivetrainSub = drivetrainSub;
    m_targetHeight = height;

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
    double direction;
    double currentLeftHeight = m_climbSub.getLeftHeight();
    double currentRightHeight = m_climbSub.getRightHeight();
    double tolerance = 5;
    int leftDirection = (m_targetHeight > currentLeftHeight) ? 1 : -1;
    int rightDirection = (m_targetHeight > currentRightHeight) ? 1 : -1;

    // TODO Need to figure out PID controller for climbing 
    //TODO final double driveOutput = m_pivotForwardPid.calculate(m_ShooterSub.getPivotVelocity(), 0.10 * direction); //10 is a target velocity we don't know what it is
    //TODO m_ShooterSub.movePivot(driveOutput);

    if(isLeftAtTargetHeight() == false)
      m_climbSub.setClimbPowerLeft(0.1 * leftDirection);

    if(isRightAtTargetHeight() == false)
      m_climbSub.setClimbPowerRight(0.1 * rightDirection);
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

