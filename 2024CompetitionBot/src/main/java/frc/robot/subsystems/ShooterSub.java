// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import frc.robot.Constants;


public class ShooterSub extends SubsystemBase {


  /** Creates a new Shooter. */

  private final CANSparkMax m_Flywheel =
      new CANSparkMax(Constants.CanIds.kFlywheel, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_UpperFeeder =
      new CANSparkMax(Constants.CanIds.kUpperFeeder, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_LowerFeeder =
      new CANSparkMax(Constants.CanIds.kLowerFeeder, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_Pivot =
      new CANSparkMax(Constants.CanIds.kPivot, CANSparkLowLevel.MotorType.kBrushless);
   private final CANSparkMax m_Transfer =
      new CANSparkMax(Constants.CanIds.kTransfer, CANSparkLowLevel.MotorType.kBrushless);    

  private final DigitalInput m_NotePosition = new DigitalInput(Constants.DioIds.kShooterNoteLimit);

  public ShooterSub() {
    //When true, positive power will turn motor backwards, negitive forwards.
    m_Flywheel.setInverted(false);
    m_UpperFeeder.setInverted(false);
    m_LowerFeeder.setInverted(false);
    m_Pivot.setInverted(false);
    m_Transfer.setInverted(false);
  }

  public void init() {
    setCurrentLimit();
    setBrake(IdleMode.kBrake);
    resetFlywheel();
    resetPivot();
    m_Flywheel.getEncoder().setVelocityConversionFactor(0.0259);
    m_Pivot.getEncoder().setVelocityConversionFactor(0.0259);
    m_Pivot.getEncoder().setPositionConversionFactor(0.68);
  }

  public void resetFlywheel() {
    m_Flywheel.getEncoder().setPosition(0);
  }

  public void resetPivot() {
    m_Pivot.getEncoder().setPosition(0);
  }

  private void setBrake(IdleMode mode) {
    m_Flywheel.setIdleMode(mode);
    m_UpperFeeder.setIdleMode(mode);
    m_LowerFeeder.setIdleMode(mode);
    m_Pivot.setIdleMode(mode);
    m_Transfer.setIdleMode(mode);

  }

  private void setCurrentLimit() {
    m_Flywheel.setSmartCurrentLimit(40);
    m_UpperFeeder.setSmartCurrentLimit(40);
    m_LowerFeeder.setSmartCurrentLimit(40);
    m_Pivot.setSmartCurrentLimit(40);
    m_Transfer.setSmartCurrentLimit(40);
  }

  public void spinFlywheel(double power) {
    m_Flywheel.set(power);
  }

  public void spinUpperFeeder(double power) {
    m_UpperFeeder.set(power);
  }

  public void spinLowerFeeder(double power) {
    m_LowerFeeder.set(power);
  }

  public void movePivot(double power) {
    m_Pivot.set(power);
  }

  public void spinTransfer(double power) {
    m_Transfer.set(power);
  }

  public double getFlywheelVelocity() {
    return m_Flywheel.getEncoder().getVelocity();
  }

  public double getPivotPosition() {
    return m_Pivot.getEncoder().getPosition();
  }

  public double getPivotVelocity() {
    return m_Pivot.getEncoder().getVelocity();
  }

  public boolean isNoteAtPosition() {
    return m_NotePosition.get();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updatesmartdashboard();
  }

  private void updatesmartdashboard() {
    SmartDashboard.putNumber("Shooter Flywheel velicity", getFlywheelVelocity());
    SmartDashboard.putNumber("Shooter Pivot Position", getPivotPosition());
    SmartDashboard.putNumber("Shooter Pivot Velocity", getPivotVelocity());
    SmartDashboard.putNumber("Shooter Flywheel Power", m_Flywheel.get());
    SmartDashboard.putNumber("Shooter Pivot Power", m_Pivot.get());
    SmartDashboard.putBoolean("Shooter Note In Position", isNoteAtPosition());
  }
}
