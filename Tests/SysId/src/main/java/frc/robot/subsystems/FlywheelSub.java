// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FlywheelSub extends SubsystemBase {
  private final CANSparkMax m_flywheelL =
      new CANSparkMax(Constants.CanIds.kFlywheelL, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_flywheelR =
      new CANSparkMax(Constants.CanIds.kFlywheelR, CANSparkLowLevel.MotorType.kBrushless);


  public FlywheelSub() {
    init();
  }

  public void init() {
    m_flywheelR.setInverted(false);
    m_flywheelL.setInverted(true);
    m_flywheelR.setIdleMode(IdleMode.kCoast);
    m_flywheelL.setIdleMode(IdleMode.kCoast);
    m_flywheelR.setSmartCurrentLimit(40);
    m_flywheelL.setSmartCurrentLimit(40);
    m_flywheelL.getEncoder().setVelocityConversionFactor(1.0);
    m_flywheelR.getEncoder().setVelocityConversionFactor(1.0);
    m_flywheelR.set(0);
    m_flywheelL.set(0);

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("L Pos", getPositionL());
    SmartDashboard.putNumber("R Pos", getPositionR());
    SmartDashboard.putNumber("L Vel", getVelocityL());
    SmartDashboard.putNumber("R Vel", getVelocityR());
  }

  public double getPositionL() {
    return m_flywheelL.getEncoder().getPosition();
  }

  public double getPositionR() {
    return m_flywheelL.getEncoder().getPosition();
  }

  public double getVelocityL() {
    return m_flywheelL.getEncoder().getVelocity();
  }

  public double getVelocityR() {
    return m_flywheelR.getEncoder().getVelocity();
  }
}

