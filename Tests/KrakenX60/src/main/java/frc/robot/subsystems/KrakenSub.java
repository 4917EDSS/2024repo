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
  private final TalonFX m_kDriveMotorFL = new TalonFX(Constants.CanIds.kDriveMotorFL);
  private final StatusSignal<Double> m_kDriveMotorFLPosition = m_kDriveMotorFL.getPosition();
  private final StatusSignal<Double> m_kDriveMotorFLVelocity = m_kDriveMotorFL.getVelocity();
  private final StatusSignal<Double> m_kDriveMotorFLAcceleration = m_kDriveMotorFL.getAcceleration();

  private final TalonFX m_kDriveMotorFR = new TalonFX(Constants.CanIds.kDriveMotorFR);
  private final StatusSignal<Double> m_kDriveMotorFRPosition = m_kDriveMotorFR.getPosition();
  private final StatusSignal<Double> m_kDriveMotorFRVelocity = m_kDriveMotorFR.getVelocity();
  private final StatusSignal<Double> m_kDriveMotorFRAcceleration = m_kDriveMotorFR.getAcceleration();

  private final TalonFX m_kDriveMotorBL = new TalonFX(Constants.CanIds.kDriveMotorBL);
  private final StatusSignal<Double> m_kDriveMotorBLPosition = m_kDriveMotorBL.getPosition();
  private final StatusSignal<Double> m_kDriveMotorBLVelocity = m_kDriveMotorBL.getVelocity();
  private final StatusSignal<Double> m_kDriveMotorBLAcceleration = m_kDriveMotorBL.getAcceleration();

  private final TalonFX m_kDriveMotorBR = new TalonFX(Constants.CanIds.kDriveMotorBR);
  private final StatusSignal<Double> m_kDriveMotorBRPosition = m_kDriveMotorBR.getPosition();
  private final StatusSignal<Double> m_kDriveMotorBRVelocity = m_kDriveMotorBR.getVelocity();
  private final StatusSignal<Double> m_kDriveMotorBRAcceleration = m_kDriveMotorBR.getAcceleration();

  //private final TalonFX m_testMotor2 = new TalonFX(Constants.CanIds.kKrakenMotor2);
  private final DutyCycleOut m_testMotorDutyCycle = new DutyCycleOut(0.0); // Create a permanent duty cycle object to improve performance

  /** Creates a new KrakenSub. */
  public KrakenSub() {
    init();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Kraken 1 Pos", getPosition(m_kDriveMotorFLPosition));
    SmartDashboard.putNumber("Kraken 2 Pos", getPosition(m_kDriveMotorFRPosition));
    SmartDashboard.putNumber("Kraken 3 Pos", getPosition(m_kDriveMotorBLPosition));
    SmartDashboard.putNumber("Kraken 4 Pos", getPosition(m_kDriveMotorBRPosition));

    SmartDashboard.putNumber("Kraken 1 Vel", getVelocity(m_kDriveMotorFLVelocity));
    SmartDashboard.putNumber("Kraken 2 Vel", getVelocity(m_kDriveMotorFRVelocity));
    SmartDashboard.putNumber("Kraken 3 Vel", getVelocity(m_kDriveMotorBLVelocity));
    SmartDashboard.putNumber("Kraken 4 Vel", getVelocity(m_kDriveMotorBRVelocity));

    SmartDashboard.putNumber("Kraken 1 Acc", getAcceleration(m_kDriveMotorFLAcceleration));
    SmartDashboard.putNumber("Kraken 2 Acc", getAcceleration(m_kDriveMotorFRAcceleration));
    SmartDashboard.putNumber("Kraken 3 Acc", getAcceleration(m_kDriveMotorBLAcceleration));
    SmartDashboard.putNumber("Kraken 4 Acc", getAcceleration(m_kDriveMotorBRAcceleration));

  }

  public void init() {
    TalonFXConfigurator talonFxConfiguaratorFL = m_kDriveMotorFL.getConfigurator();
    TalonFXConfigurator talonFxConfiguaratorFR = m_kDriveMotorFR.getConfigurator();
    TalonFXConfigurator talonFxConfiguaratorBL = m_kDriveMotorBL.getConfigurator();
    TalonFXConfigurator talonFxConfiguaratorBR = m_kDriveMotorBR.getConfigurator();

    CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = 3;
    limitConfigs.StatorCurrentLimitEnable = true;
    talonFxConfiguaratorFL.apply(limitConfigs);
    talonFxConfiguaratorFR.apply(limitConfigs);
    talonFxConfiguaratorBL.apply(limitConfigs);
    talonFxConfiguaratorBR.apply(limitConfigs);

    MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
    outputConfigs.DutyCycleNeutralDeadband = 0.02; // Ignore values below 2%
    outputConfigs.Inverted = InvertedValue.CounterClockwise_Positive; // Invert = Clockwise
    outputConfigs.NeutralMode = NeutralModeValue.Brake;
    talonFxConfiguaratorFL.apply(outputConfigs);
    talonFxConfiguaratorFR.apply(outputConfigs);
    talonFxConfiguaratorBL.apply(outputConfigs);
    talonFxConfiguaratorBR.apply(outputConfigs);

    FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
    feedbackConfigs.SensorToMechanismRatio = 0.5;
    talonFxConfiguaratorFL.apply(feedbackConfigs);
    talonFxConfiguaratorFR.apply(feedbackConfigs);
    talonFxConfiguaratorBL.apply(feedbackConfigs);
    talonFxConfiguaratorBR.apply(feedbackConfigs);

    // To configure a second motor to follow the first motor
    //boolean turnOppositeDirectionFromMaster = true; // False if both motors turn in same direction, true to make them turn in opposite directions
    //m_testMotor2.setControl(new Follower(m_testMotor.getDeviceID(), turnOppositeDirectionFromMaster));

    resetPosition();
  }

  public void runMotor(double power) {
    m_kDriveMotorFL.setControl(m_testMotorDutyCycle.withOutput(power));
    m_kDriveMotorFR.setControl(m_testMotorDutyCycle.withOutput(power));
    m_kDriveMotorBL.setControl(m_testMotorDutyCycle.withOutput(power));
    m_kDriveMotorBR.setControl(m_testMotorDutyCycle.withOutput(power));
  }

  public double getPosition(StatusSignal<Double> motor) {
    motor.refresh();
    return motor.getValueAsDouble();
  }

  public double getVelocity(StatusSignal<Double> motor) {
    motor.refresh();
    return motor.getValueAsDouble();
  }

  public double getAcceleration(StatusSignal<Double> motor) {
    motor.refresh();
    return motor.getValueAsDouble();
  }

  public void resetPosition() {
    m_kDriveMotorFL.setPosition(0, 0.2); // Not sure if the timeout is necessary or beneficial
    m_kDriveMotorBL.setPosition(0, 0.2);
    m_kDriveMotorFR.setPosition(0, 0.2);
    m_kDriveMotorBR.setPosition(0, 0.2);
  }
}
