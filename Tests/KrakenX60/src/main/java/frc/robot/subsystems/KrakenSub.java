// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
// import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class KrakenSub extends SubsystemBase {
  private final TalonFX m_testMotor = new TalonFX(Constants.CanIds.kKrakenMotor);
  private final StatusSignal<Double> m_testMotorPosition = m_testMotor.getPosition();
  private final StatusSignal<Double> m_testMotorVelocity = m_testMotor.getVelocity();
  private final StatusSignal<Double> m_testMotorAcceleration = m_testMotor.getAcceleration();
  private final StatusSignal<Double> m_testMotorAmps = m_testMotor.getSupplyCurrent();

  //private final TalonFX m_testMotor2 = new TalonFX(Constants.CanIds.kKrakenMotor2);
  private final DutyCycleOut m_testMotorDutyCycle = new DutyCycleOut(0.0); // Create a permanent duty cycle object to improve performance

  /** Creates a new KrakenSub. */
  public KrakenSub() {
    init();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Kraken 1 Pos", getPosition());
    SmartDashboard.putNumber("Kraken 1 Vel", getVelocity());
    SmartDashboard.putNumber("Kraken 1 Acc", getAcceleration());
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

    FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
    feedbackConfigs.SensorToMechanismRatio = 0.5;
    talonFxConfiguarator.apply(feedbackConfigs);

    // To configure a second motor to follow the first motor
    //boolean turnOppositeDirectionFromMaster = true; // False if both motors turn in same direction, true to make them turn in opposite directions
    //m_testMotor2.setControl(new Follower(m_testMotor.getDeviceID(), turnOppositeDirectionFromMaster));

    resetPosition();
  }

  public void runMotor(double power) {
    m_testMotor.setControl(m_testMotorDutyCycle.withOutput(power));
  }

  public double getPosition() {
    m_testMotorPosition.refresh();
    return m_testMotorPosition.getValueAsDouble();
  }

  public double getVelocity() {
    m_testMotorVelocity.refresh();
    return m_testMotorVelocity.getValueAsDouble();
  }

  public double getAcceleration() {
    m_testMotorAcceleration.refresh();
    return m_testMotorAcceleration.getValueAsDouble();
  }

  public double getAmps() {
    m_testMotorAmps.refresh();
    return m_testMotorAmps.getValueAsDouble();
  }

  public void resetPosition() {
    m_testMotor.setPosition(0, 0.2); // Not sure if the timeout is necessary or beneficial
  }
}
