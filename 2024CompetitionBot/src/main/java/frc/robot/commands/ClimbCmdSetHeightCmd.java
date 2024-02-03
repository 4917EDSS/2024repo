// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.logging.Logger;
import javax.swing.plaf.TreeUI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSub;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.VisionSub;


public class ClimbCmdSetHeightCmd extends Command {
  private static Logger m_logger = Logger.getLogger(ClimbCmdSetHeightCmd.class.getName());
  private final double kHeightTolerence = 0.01;
  private final double kRollZero = -4.3;
  private final double kRollTolerence = 10;
  private final double kMinRollAngle = kRollZero - kRollTolerence;
  private final double kMaxRollAngle = kRollZero + kRollTolerence;

  private final ClimbSub m_climbSub;
  private final DrivetrainSub m_drivetrainSub;
  private final double m_targetHeight;
  private final double m_power;

  private boolean m_leftMotorDone = false;
  private boolean m_rightMotorDone = false;

  //private final PIDController m_pivotForwardPid = new PIDController(1.0, 0, 0); // TODO: Tune the Driving PID

  /** Creates a new Climb. */
  public ClimbCmdSetHeightCmd(double heightM, double power, DrivetrainSub drivetrainSub, ClimbSub climbSub) {
    m_climbSub = climbSub;
    m_drivetrainSub = drivetrainSub;
    m_targetHeight = heightM;
    m_power = power;


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climbSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_leftMotorDone = false;
    m_rightMotorDone = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //set power
    double currentLeftHeight = m_climbSub.getLeftHeight();
    double currentRightHeight = m_climbSub.getRightHeight();
    int leftDirection = (m_targetHeight > currentLeftHeight) ? 1 : -1;
    int rightDirection = (m_targetHeight > currentRightHeight) ? 1 : -1;


    boolean moveLeft = true;
    boolean moveRight = true;

    boolean isLeftAtTargetHeight = isLeftAtTargetHeight();
    boolean isRightAtTargetHeight = isRightAtTargetHeight();
    double roll_angle = m_drivetrainSub.getRoll();

    if(isLeftAtTargetHeight) {
      m_leftMotorDone = true;
    }

    if(isRightAtTargetHeight) {
      m_rightMotorDone = true;
    }

    // Is left at height or right at height
    if(isLeftAtTargetHeight || isRightAtTargetHeight) {
      // If left at height
      if(isLeftAtTargetHeight) {
        // Stop left
        moveLeft = false;
      }
      // If right at height
      if(isRightAtTargetHeight) {
        // Stop right
        moveRight = false;
      }
    } else if(roll_angle < kMinRollAngle) {
      // else if roll angle < minRoll  (i.e. tilted to the right because right tilt is negative)
      // if direction is positive
      if(leftDirection > 0) {
        // Stop right motor
        moveRight = false;
      } else {
        // else (direction is negative)
        // Stop the left motor
        moveLeft = false;
      }
      // else if roll angle > maxRoll
    } else if(roll_angle > kMaxRollAngle) {
      // if direction is positive
      if(rightDirection > 0) {
        // Stop left motor
        moveLeft = false;
      } else {
        // else
        // Stop right motor
        moveRight = false;
      }
    }

    if(moveLeft && !m_leftMotorDone) {
      m_climbSub.setClimbPowerLeft(m_power * leftDirection);
    } else {
      m_climbSub.setClimbPowerLeft(0.0);
    }
    if(moveRight && !m_rightMotorDone) {
      m_climbSub.setClimbPowerRight(m_power * rightDirection);
    } else {
      m_climbSub.setClimbPowerRight(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climbSub.setClimbPowerLeft(0.0);
    m_climbSub.setClimbPowerRight(0.0);
  }

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

