// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkLowLevel;


public class IntakeSub extends SubsystemBase {
  private final CANSparkMax m_intakeRollers =
      new CANSparkMax(Constants.CanIds.kIntakeRollers, CANSparkLowLevel.MotorType.kBrushless);
  // Line below most likely will not be used, but can be used as a working absolute encoder if necessary
  private final SparkAbsoluteEncoder m_intakeEncoder = m_intakeRollers.getAbsoluteEncoder (Type.kDutyCycle);
  private final DigitalInput m_intakeLimitSwitch = new DigitalInput(Constants.DioIds.kIntakeLimitPort);
  /** Creates a new Intake. */
  public IntakeSub() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateSmartDashboard();
  }
  public boolean getNoteFullyIn() {
    return !m_intakeLimitSwitch.get();
  }

  public void updateSmartDashboard() {
    SmartDashboard.putBoolean("Note In", getNoteFullyIn());
    SmartDashboard.putNumber("Rotation Finished",getEncoderRotations());
  }

  public void setIntakeMotors(double power) {
    m_intakeRollers.set(power);
  }

  public double getEncoderRotations() {
    return m_intakeEncoder.getPosition();
  }
  
}

