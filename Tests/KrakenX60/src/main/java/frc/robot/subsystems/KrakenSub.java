// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
// import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class KrakenSub extends SubsystemBase {
  private final TalonFX m_testMotor = new TalonFX(Constants.CanIds.kKrakenMotor);
  //private final TalonFX m_testMotor2 = new TalonFX(Constants.CanIds.kKrakenMotor2);
  private final DutyCycleOut m_testMotorDutyCycle = new DutyCycleOut(0.0); // Create a permanent duty cycle object to improve performance

  /** Creates a new KrakenSub. */
  public KrakenSub() {
    init();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void init() {
    TalonFXConfigurator talonFxConfiguarator = m_testMotor.getConfigurator();

    CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = 3;
    limitConfigs.StatorCurrentLimitEnable = true;
    talonFxConfiguarator.apply(limitConfigs);

    MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
    outputConfigs.DutyCycleNeutralDeadband = 0.02; // Ignore values below 2%
    outputConfigs.Inverted = InvertedValue.CounterClockwise_Positive; // Invert = Clockwise
    outputConfigs.NeutralMode = NeutralModeValue.Brake;
    talonFxConfiguarator.apply(outputConfigs);

    // To configure a second motor to follow the first motor
    //boolean turnOppositeDirectionFromMaster = true; // False if both motors turn in same direction, true to make them turn in opposite directions
    //m_testMotor2.setControl(new Follower(m_testMotor.getDeviceID(), turnOppositeDirectionFromMaster));
  }

  public void runMotor(double power) {
    m_testMotor.setControl(m_testMotorDutyCycle.withOutput(power));
  }
}
