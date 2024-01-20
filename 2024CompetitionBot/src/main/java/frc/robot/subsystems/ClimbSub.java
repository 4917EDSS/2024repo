// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;


public class ClimbSub extends SubsystemBase {
  private final static CANSparkMax m_climbMotorRight =
      new CANSparkMax(Constants.CanIds.kClimbMotorRight, CANSparkLowLevel.MotorType.kBrushless);
  private final static CANSparkMax m_climbMotorLeft =
      new CANSparkMax(Constants.CanIds.kClimbMotorLeft, CANSparkLowLevel.MotorType.kBrushless);

  /** Creates a new ClimbSub. */
  public ClimbSub() {
    m_climbMotorLeft.setIdleMode(IdleMode.kBrake);
    m_climbMotorRight.setIdleMode(IdleMode.kBrake);
    m_climbMotorLeft.getEncoder().setPositionConversionFactor(Constants.ClimbConstants.kTickCofficient);
    m_climbMotorRight.getEncoder().setPositionConversionFactor(Constants.ClimbConstants.kTickCofficient);

  }

  @Override
  public void periodic() {

  }

  public void setClimbPowerLeft(double leftPower) {
    m_climbMotorLeft.set(leftPower);

  }

  public void setClimbPowerRight(double rightPower) {
    m_climbMotorRight.set(rightPower);
  }

  public static double getLeftHeight() {
    return m_climbMotorLeft.getEncoder().getPosition();
  }

  public static double getRightHeight() {
    return m_climbMotorRight.getEncoder().getPosition();
  }
}
