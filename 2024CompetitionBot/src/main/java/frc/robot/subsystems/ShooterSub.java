// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import frc.robot.Constants;

public class ShooterSub extends SubsystemBase {
  /** Creates a new Shooter. */

  private final CANSparkMax m_CanSparkMax =
      new CANSparkMax(Constants.CanIds.kFlywheelMotor, CANSparkLowLevel.MotorType.kBrushless);

  public ShooterSub() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
