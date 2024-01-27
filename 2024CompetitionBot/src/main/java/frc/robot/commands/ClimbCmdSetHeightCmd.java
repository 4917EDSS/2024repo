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
  private final double kHeightTolerence = 0.01;

  private final ClimbSub m_climbSub;
  private final DrivetrainSub m_drivetrainSub;
  private double m_targetHeight;

  private final PIDController m_pivotForwardPid = new PIDController(1.0, 0, 0); // TODO: Tune the Driving PID

  /** Creates a new Climb. */
  public ClimbCmdSetHeightCmd(double heightM, DrivetrainSub drivetrainSub, ClimbSub climbSub) {
    m_climbSub = climbSub;
    m_drivetrainSub = drivetrainSub;
    m_targetHeight = heightM;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climbSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //set power
    double currentLeftHeight = m_climbSub.getLeftHeight();
    double currentRightHeight = m_climbSub.getRightHeight();
    int leftDirection = (m_targetHeight > currentLeftHeight) ? 1 : -1;
    int rightDirection = (m_targetHeight > currentRightHeight) ? 1 : -1;

    // TODO Need to figure out PID controller for climbing
    //TODO final double driveOutput = m_pivotForwardPid.calculate(m_ShooterSub.getPivotVelocity(), 0.10 * direction);
    //10 is a target velocity we don't know what it is
    //TODO m_ShooterSub.movePivot(driveOutput);

    //if(!isLeftAtTargetHeight() && (m_drivetrainSub.getRoll() <= 0.0)) {
    if(!isLeftAtTargetHeight()) {
      m_climbSub.setClimbPowerLeft(0.2 * leftDirection);
    } else {
      m_climbSub.setClimbPowerLeft(0.0);
    }
    //if(!isRightAtTargetHeight() && (m_drivetrainSub.getRoll() >= 0.0)) {
    if(!isRightAtTargetHeight()) {
      m_climbSub.setClimbPowerRight(0.2 * rightDirection);
    } else {
      m_climbSub.setClimbPowerRight(0.0);
    }

    // m_climbSub.setClimbPowerRight(0.2);
    // m_climbSub.setClimbPowerLeft(0.2);
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
    return (Math.abs(m_climbSub.getLeftHeight() - m_targetHeight) < kHeightTolerence);
  }

  private boolean isRightAtTargetHeight() {
    return (Math.abs(m_climbSub.getRightHeight() - m_targetHeight) < kHeightTolerence);
  }
}

