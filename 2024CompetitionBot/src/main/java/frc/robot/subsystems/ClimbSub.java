// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;


public class ClimbSub extends SubsystemBase {
  private final static CANSparkMax m_climbMotorLeft =
      new CANSparkMax(Constants.CanIds.kClimbMotorL, CANSparkLowLevel.MotorType.kBrushless);
  private final static CANSparkMax m_climbMotorRight =
      new CANSparkMax(Constants.CanIds.kClimbMotorR, CANSparkLowLevel.MotorType.kBrushless);

  /** Creates a new ClimbSub. */
  public ClimbSub() {
    m_climbMotorLeft.setIdleMode(IdleMode.kBrake);
    m_climbMotorRight.setIdleMode(IdleMode.kBrake);
    m_climbMotorLeft.getEncoder().setPositionConversionFactor(Constants.ClimbConstants.kTickCofficient);
    m_climbMotorRight.getEncoder().setPositionConversionFactor(Constants.ClimbConstants.kTickCofficient);
    setClimbPowerLeft(0.0);
    setClimbPowerRight(0.0);
    SmartDashboard.putData("ClimbReset", new InstantCommand(() -> resetEncoders()));

  }

  @Override
  public void periodic() {
    updateSmartDashboard();
  }

  private void updateSmartDashboard() {
    SmartDashboard.putNumber("Climb Left Power", m_climbMotorLeft.get());
    SmartDashboard.putNumber("Climb Right Power", m_climbMotorRight.get());
    SmartDashboard.putNumber("Climb Left Height", getLeftHeight());
    SmartDashboard.putNumber("Climb Right Height", getRightHeight());
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

  public void resetEncoders() {
    System.out.println("Reseting Encoders");
    m_climbMotorLeft.getEncoder().setPosition(0.0);
    m_climbMotorRight.getEncoder().setPosition(0.0);
  }
}
