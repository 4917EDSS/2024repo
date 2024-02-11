// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ClimbSub extends SubsystemBase {
  private static Logger m_logger = Logger.getLogger(ClimbSub.class.getName());

  private final static CANSparkMax m_climbMotorLeft =
      new CANSparkMax(Constants.CanIds.kClimbMotorL, CANSparkLowLevel.MotorType.kBrushless);
  private final static CANSparkMax m_climbMotorRight =
      new CANSparkMax(Constants.CanIds.kClimbMotorR, CANSparkLowLevel.MotorType.kBrushless);

  private final ShuffleboardTab m_shuffleboardTab = Shuffleboard.getTab("Climb");
  private final GenericEntry m_sbClimbLeftpower, m_sbClimbRightpower, m_sbClimbLeftheight, m_sbClimbRightheight,
      m_sbClimbLeftLimit, m_sbClimbRightLimit;


  /** Creates a new ClimbSub. */
  public ClimbSub() {
    m_climbMotorLeft.setInverted(true);
    m_climbMotorRight.setInverted(false);
    m_climbMotorLeft.setIdleMode(IdleMode.kBrake);
    m_climbMotorRight.setIdleMode(IdleMode.kBrake);
    m_climbMotorLeft.getEncoder().setPositionConversionFactor(Constants.Climb.kTickCofficient);
    m_climbMotorRight.getEncoder().setPositionConversionFactor(Constants.Climb.kTickCofficient);

    SmartDashboard.putData("ClimbReset", new InstantCommand(() -> resetEncoders()));
    m_sbClimbLeftpower = m_shuffleboardTab.add("Climb Left Power", 0).getEntry();
    m_sbClimbRightpower = m_shuffleboardTab.add("Climb Right Power", 0).getEntry();
    m_sbClimbLeftheight = m_shuffleboardTab.add("Climb Left Height", 0).getEntry();
    m_sbClimbRightheight = m_shuffleboardTab.add("Climb Right Height", 0).getEntry();
    m_sbClimbLeftLimit = m_shuffleboardTab.add("Climb Left Limit", 0).getEntry();
    m_sbClimbRightLimit = m_shuffleboardTab.add("Climb Right Limit", 0).getEntry();

    init();
  }

  public void init() {
    m_logger.info("Initializing ClimbSub");
    setClimbPowerLeft(0.0);
    setClimbPowerRight(0.0);
    resetEncoders();
  }

  @Override
  public void periodic() {
    updateShuffleBoard();
  }

  private void updateShuffleBoard() {
    m_sbClimbLeftpower.setDouble(m_climbMotorLeft.get());
    m_sbClimbRightpower.setDouble(m_climbMotorRight.get());
    m_sbClimbLeftheight.setDouble(getLeftHeight());
    m_sbClimbRightheight.setDouble(getRightHeight());
    m_sbClimbLeftLimit.setBoolean(isLeftAtLimit());
    m_sbClimbRightLimit.setBoolean(isRightAtLimit());
  }

  public void setClimbPowerLeft(double leftPower) {
    m_climbMotorLeft.set(leftPower);
  }

  public void setClimbPowerRight(double rightPower) {
    m_climbMotorRight.set(rightPower);
  }

  public double getLeftHeight() {
    return m_climbMotorLeft.getEncoder().getPosition();
  }

  public double getRightHeight() {
    return m_climbMotorRight.getEncoder().getPosition();
  }

  public double getLeftVelocity() {
    return m_climbMotorLeft.getEncoder().getVelocity();
  }

  public double getRightVelocity() {
    return m_climbMotorRight.getEncoder().getVelocity();
  }

  public boolean isLeftAtLimit() {
    return m_climbMotorLeft.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed();
  }

  public boolean isRightAtLimit() {
    return m_climbMotorRight.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed();
  }

  public void resetEncoders() {
    m_logger.warning("Reseting Encoders");
    m_climbMotorLeft.getEncoder().setPosition(0.0);
    m_climbMotorRight.getEncoder().setPosition(0.0);
  }
}
