// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;


public class ClimbSub extends SubsystemBase {
  private final CANSparkMax m_climbMotorRight =
      new CanSparkMax(Constants.CanIds.kClimbMotorRight, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_climbMotorLeft =
      new CanSparkMax(Constants.CanIds.kClimbMotorLeft, CANSparkLowLevel.MotorType.kBrushless);

  /** Creates a new ClimbSub. */
  public ClimbSub() {}

  @Override
  public void periodic() {

  }

  public void ClimbMotorRight(double power) {
    m_climbMotorLeft.set(power);
  }

  public void ClimbMotorLeft(double power) {
    m_climbMotorLeft.set(power);
    // This method will be called once per scheduler run
  }
}
