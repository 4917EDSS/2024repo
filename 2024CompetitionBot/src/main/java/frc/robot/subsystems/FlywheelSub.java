// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FlywheelSub extends SubsystemBase {
  /** Creates a new FlywheelSub. */

  private final CANSparkMax m_flywheelL =
      new CANSparkMax(Constants.CanIds.kFlywheelL, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_flywheelR =
      new CANSparkMax(Constants.CanIds.kFlywheelR, CANSparkLowLevel.MotorType.kBrushless);
  // PID Controllers
  private final PIDController m_FlyWheelPID = new PIDController(1.0, 0, 0); // TODO: Tune the Driving PID

  private final int m_FlywheelTolerance = 5;


  public FlywheelSub() {
    m_flywheelR.setInverted(false);
    m_flywheelL.setInverted(true);
    m_flywheelR.setIdleMode(IdleMode.kCoast);
    m_flywheelL.setIdleMode(IdleMode.kCoast);
    m_flywheelR.setSmartCurrentLimit(40);
    m_flywheelL.setSmartCurrentLimit(40);
    m_flywheelR.getEncoder().setVelocityConversionFactor(1.0);
    m_flywheelL.getEncoder().setVelocityConversionFactor(1.0);
    m_flywheelL.follow(m_flywheelR);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Flywheel needs to spin at full power prior to m_shooterSub.spinBothFeeders being executed. 
    // only using left Flywheel velocity. Todo add a right velocity
    double driveOutput = m_FlyWheelPID.calculate(getFlywheelVelocityL(), 4200); //10 is a target velocity we don't know what it is 

    EnableFlywheel(driveOutput);
  }

  public void EnableFlywheel(double power) {
    m_flywheelR.set(power);
    m_flywheelL.set(power);

  }

  public void DisableFlywheel() {
    m_flywheelR.set(0);
    m_flywheelL.set(0);

  }


  public boolean isAtTargetVelocity(double Velocity) {

    if(Math.abs(getFlywheelVelocityL() - Velocity) < m_FlywheelTolerance) {
      return true;
    }
  }

  public double getFlywheelVelocityR() {
    return m_flywheelR.getEncoder().getVelocity();
  }

  public double getFlywheelVelocityL() {
    return m_flywheelL.getEncoder().getVelocity();
  }
}
