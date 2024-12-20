// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSub extends SubsystemBase {
  private final Spark m_leftMotor = new Spark(1);
  private final Spark m_rightMotor = new Spark(0);


  /** Creates a new DrivetrainSub. */
  public DrivetrainSub() {

  }


  public void drive(double leftPower, double rightPower) {
    m_leftMotor.set(leftPower);
    m_rightMotor.set(rightPower);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}
