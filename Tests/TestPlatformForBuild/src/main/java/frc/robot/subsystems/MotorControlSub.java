// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;


public class MotorControlSub extends SubsystemBase {
  public Boolean m_couple = false;
  public double m_motor1Power;
  public double m_motor2Power;
  PowerDistribution powerDistributionModule = new PowerDistribution(1, ModuleType.kRev);
  private final CANSparkMax m_motor1;
  private final CANSparkMax m_motor2;
  private RelativeEncoder m_motor1Encoder;
  private RelativeEncoder m_motor2Encoder;
  private double m_motor1Current;
  private double m_motor2Current;


  /** Creates a new MotorControl. */
  public MotorControlSub() {
    m_motor1 = new CANSparkMax(Constants.CanIds.kMotor1, MotorType.kBrushless);
    m_motor2 = new CANSparkMax(Constants.CanIds.kMotor2, MotorType.kBrushless);
    m_motor1.setInverted(false);
    m_motor2.setInverted(false);
    m_motor1Encoder = m_motor1.getEncoder();
    m_motor2Encoder = m_motor2.getEncoder();
    m_motor1Current = powerDistributionModule.getCurrent(Constants.Breakers.kMotor1);
    m_motor2Current = powerDistributionModule.getCurrent(Constants.Breakers.kMotor2);
    SmartDashboard.putNumber("Motor 1 RPM", m_motor1Encoder.getVelocity());
    SmartDashboard.putNumber("Motor 2 RPM", m_motor2Encoder.getVelocity());
    SmartDashboard.putNumber("Motor 1 Current", m_motor1Current);
    SmartDashboard.putNumber("Motor 2 Current", m_motor2Current);
    SmartDashboard.putBoolean("Coupled", m_couple);
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run

  }

  public void setPowerMotor1(double motor1Power) {
    if(m_couple) {
      double temporaryMotor2Power = m_motor1Power;
      m_motor2.set(temporaryMotor2Power);
      SmartDashboard.putNumber("Motor 2 Power", temporaryMotor2Power);
      m_motor2Power = temporaryMotor2Power;
    }
    m_motor1.set(motor1Power);
    SmartDashboard.putNumber("Motor 1 Power", motor1Power);
    m_motor1Power = motor1Power;
  }

  public void setPowerMotor2(double motor2Power) {
    if(m_couple) {
      double temporaryMotor1Power = m_motor2Power;
      m_motor1.set(temporaryMotor1Power);
      SmartDashboard.putNumber("Motor 1 Power", temporaryMotor1Power);
      m_motor1Power = temporaryMotor1Power;
    }
    m_motor1.set(motor2Power);
    SmartDashboard.putNumber("Motor 2 Power", motor2Power);
    m_motor2Power = motor2Power;
  }


}
