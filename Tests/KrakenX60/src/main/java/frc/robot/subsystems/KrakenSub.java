// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class KrakenSub extends SubsystemBase {
  private final TalonFX m_testMotor = new TalonFX(Constants.CanIds.kKrakenMotor);
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
    var talonFxConfiguarator = m_testMotor.getConfigurator();
    var limitConfigs = new CurrentLimitsConfigs();

    limitConfigs.StatorCurrentLimit = 3;
    limitConfigs.StatorCurrentLimitEnable = true;
    talonFxConfiguarator.apply(limitConfigs);
  }

  public void runMotor(double power) {
    m_testMotor.setControl(m_testMotorDutyCycle.withOutput(power));
  }
}
