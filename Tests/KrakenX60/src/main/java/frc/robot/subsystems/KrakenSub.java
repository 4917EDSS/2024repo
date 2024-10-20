// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class KrakenSub extends SubsystemBase {
  private final TalonFX m_testMotor = new TalonFX(Constants.CanIds.kKrakenMotor);

  /** Creates a new KrakenSub. */
  public KrakenSub() {
    //m_testMotor
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runMotor(double power) {
    //m_testMotor.setControl
    m_testMotor.set(power);
  }
}
