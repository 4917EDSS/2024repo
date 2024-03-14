// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.logging.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSub;
import frc.robot.subsystems.DrivetrainSub;


public class ClimbSetHeightCmd extends Command {
  private static Logger m_logger = Logger.getLogger(ClimbSetHeightCmd.class.getName());

  private final double m_targetHeight;
  private double m_power;
  private final ClimbSub m_climbSub;
  private final DrivetrainSub m_drivetrainSub;

  private boolean m_leftMotorDone = false;
  private boolean m_rightMotorDone = false;
  private boolean m_goingUp = false;


  /** Creates a new Climb. */
  public ClimbSetHeightCmd(double heightM, double power, DrivetrainSub drivetrainSub, ClimbSub climbSub) {
    m_climbSub = climbSub;
    m_drivetrainSub = drivetrainSub;
    m_targetHeight = heightM;
    m_power = power;

    // Use addRequirements() here to declare subsystem dependencies.
    // Don't require drivetrainSub since we only use it to read the 'roll' angle
    addRequirements(climbSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_logger.fine("ClimbSetHeightCmd - Init");
    m_leftMotorDone = false;
    m_rightMotorDone = false;
    m_goingUp = m_climbSub.getLeftHeight() < m_targetHeight;
    if(!m_goingUp) {
      m_power = -m_power;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double currentLeftHeight = m_climbSub.getLeftHeight();
    // double currentRightHeight = m_climbSub.getRightHeight();
    // int leftDirection = (m_targetHeight > currentLeftHeight) ? 1 : -1;
    // int rightDirection = (m_targetHeight > currentRightHeight) ? 1 : -1;

    // boolean moveLeft = true;
    // boolean moveRight = true;
    // boolean isLeftAtTargetHeight = isLeftAtTargetHeight();
    // boolean isRightAtTargetHeight = isRightAtTargetHeight();
    // double roll_angle = m_drivetrainSub.getRollRotationDegrees();

    // if(isLeftAtTargetHeight) {
    //   m_leftMotorDone = true;
    // }

    // if(isRightAtTargetHeight) {
    //   m_rightMotorDone = true;
    // }

    // // Is left at height or right at height
    // if(isLeftAtTargetHeight || isRightAtTargetHeight) {
    //   // If left at height
    //   if(isLeftAtTargetHeight) {
    //     // Stop left
    //     moveLeft = false;
    //   }
    //   // If right at height
    //   if(isRightAtTargetHeight) {
    //     // Stop right
    //     moveRight = false;
    //   }
    // } else if(roll_angle < Constants.Climb.kMinRollAngle) {
    //   // else if roll angle < minRoll  (i.e. tilted to the right because right tilt is negative)
    //   // if direction is positive
    //   if(leftDirection > 0) {
    //     // Stop right motor
    //     moveRight = false;
    //   } else {
    //     // else (direction is negative)
    //     // Stop the left motor
    //     moveLeft = false;
    //   }
    //   // else if roll angle > maxRoll
    // } else if(roll_angle > Constants.Climb.kMaxRollAngle) {
    //   // if direction is positive
    //   if(rightDirection > 0) {
    //     // Stop left motor
    //     moveLeft = false;
    //   } else {
    //     // else
    //     // Stop right motor
    //     moveRight = false;
    //   }
    // }

    // if(moveLeft && !m_leftMotorDone) {
    //   m_climbSub.setClimbPowerLeft(m_power * leftDirection);

    //   //m_logger.fine(" move left " + moveLeft);
    //   //m_logger.fine("left motor done " + m_leftMotorDone);

    // } else {
    //   m_climbSub.setClimbPowerLeft(0.0);


    // }
    // if(moveRight && !m_rightMotorDone) {
    //   m_climbSub.setClimbPowerRight(m_power * rightDirection);
    //   //m_logger.fine(" move right " + moveRight);
    //   // m_logger.fine("right motor done " + m_rightMotorDone);

    // } else {
    //   m_climbSub.setClimbPowerRight(0.0);
    // }

    double angleAdjustment = 0;


    m_climbSub.setClimbPowerRight(m_power + angleAdjustment);
    m_climbSub.setClimbPowerLeft(m_power - angleAdjustment);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_logger.fine("ClimbSetHeightCmd - End" + (interrupted ? " (interrupted)" : ""));
    m_climbSub.setClimbPowerLeft(0.0);
    m_climbSub.setClimbPowerRight(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (isLeftAtTargetHeight() || isRightAtTargetHeight());
  }

  private boolean isLeftAtTargetHeight() {
    if(m_goingUp) {
      return m_climbSub.getLeftHeight() >= m_targetHeight;
    } else {
      return m_climbSub.getLeftHeight() <= m_targetHeight;
    }
  }

  private boolean isRightAtTargetHeight() {
    if(m_goingUp) {
      return m_climbSub.getRightHeight() >= m_targetHeight;
    } else {
      return m_climbSub.getRightHeight() <= m_targetHeight;
    }
  }
}

