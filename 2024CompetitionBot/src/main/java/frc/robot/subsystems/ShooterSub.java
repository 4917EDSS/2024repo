// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import java.util.logging.Logger;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import frc.robot.Constants;
import frc.robot.subsystems.LedSub.LedColour;
import frc.robot.subsystems.LedSub.LedZones;


public class ShooterSub extends SubsystemBase {
  private static Logger m_logger = Logger.getLogger(SubsystemBase.class.getName());


  /** Creates a new Shooter. */
  private final LedSub m_LedSub = new LedSub();
  private final CANSparkMax m_flywheel =
      new CANSparkMax(Constants.CanIds.kFlywheelL, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_upperFeeder =
      new CANSparkMax(Constants.CanIds.kUpperFeeder, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_lowerFeeder =
      new CANSparkMax(Constants.CanIds.kLowerFeeder, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_pivot =
      new CANSparkMax(Constants.CanIds.kPivot, CANSparkLowLevel.MotorType.kBrushless);
  // private final CANSparkMax m_transfer =
  //     new CANSparkMax(Constants.CanIds.kTransfer, CANSparkLowLevel.MotorType.kBrushless);

  private final DigitalInput m_NotePosition = new DigitalInput(Constants.DioIds.kShooterNoteLimit);


  PIDController m_shooterPivotPID = new PIDController(0.01, 0.0, 0.0);

  ArmFeedforward m_armFeedforward = new ArmFeedforward(0, 0, 0);

  public ShooterSub() {
    //When true, positive power will turn motor backwards, negitive forwards.
    m_flywheel.setInverted(false);
    m_upperFeeder.setInverted(false);
    m_lowerFeeder.setInverted(false);
    m_pivot.setInverted(false);
    //m_transfer.setInverted(false);
  }

  public void init() {
    setCurrentLimit();
    setBrake(IdleMode.kBrake);
    resetFlywheel();
    resetPivot();
    m_flywheel.getEncoder().setVelocityConversionFactor(0.0259);
    m_pivot.getEncoder().setVelocityConversionFactor(0.0259);
    m_pivot.getEncoder().setPositionConversionFactor(0.68);
  }

  public void resetFlywheel() {
    m_flywheel.getEncoder().setPosition(0);
  }

  public void resetPivot() {
    m_pivot.getEncoder().setPosition(0);
  }

  private void setBrake(IdleMode mode) {
    m_flywheel.setIdleMode(mode);
    m_upperFeeder.setIdleMode(mode);
    m_lowerFeeder.setIdleMode(mode);
    m_pivot.setIdleMode(mode);
    //m_transfer.setIdleMode(mode);

  }

  private void setCurrentLimit() {
    m_flywheel.setSmartCurrentLimit(40);
    m_upperFeeder.setSmartCurrentLimit(40);
    m_lowerFeeder.setSmartCurrentLimit(40);
    m_pivot.setSmartCurrentLimit(40);
    //m_transfer.setSmartCurrentLimit(40);
  }

  public void spinFlywheel(double power) {
    m_flywheel.set(power);
  }

  public void spinUpperFeeder(double power) {
    m_upperFeeder.set(power);
  }

  public void spinLowerFeeder(double power) {
    m_lowerFeeder.set(power);
  }

  public void movePivot(double power) {
    m_pivot.set(power);
  }

  public void setAngle(double angle) {
    double pivotPower = m_shooterPivotPID.calculate(0.0, 0.0);
    double pivotFeedforward = m_armFeedforward.calculate(0.0, 0.0, 0.0);
    //TODO: Replace these placeholder values
  }

  //public void spinTransfer(double power) {
  //m_transfer.set(power);
  //}

  public double getFlywheelVelocity() {
    return m_flywheel.getEncoder().getVelocity();
  }

  public double getPivotPosition() {
    return m_pivot.getEncoder().getPosition();
  }

  public double getPivotVelocity() {
    return m_pivot.getEncoder().getVelocity();
  }

  public boolean isNoteAtPosition() {
    return m_NotePosition.get();
  }

  public boolean isPivotAtReverseLimit() {
    return m_pivot.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed();
  }

  public boolean isPivotAtForwardLimit() {
    return m_pivot.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //updatesmartdashboard();
    if(getPivotPosition() == Constants.ShooterPivotPositionConstants.kAmpPosition) {
      m_LedSub.setZoneColour(LedZones.DIAG_SHOOTER_POSITION, LedColour.PURPLE);
    }

    if(getPivotPosition() == Constants.ShooterPivotPositionConstants.kSpeakerPosition) {
      m_LedSub.setZoneColour(LedZones.DIAG_SHOOTER_POSITION, LedColour.WHITE);
    }


  }

  private void updatesmartdashboard() {
    SmartDashboard.putNumber("Shooter Flywheel velicity", getFlywheelVelocity());
    SmartDashboard.putNumber("Shooter Pivot Position", getPivotPosition());
    SmartDashboard.putNumber("Shooter Pivot Velocity", getPivotVelocity());
    SmartDashboard.putNumber("Shooter Flywheel Power", m_flywheel.get());
    SmartDashboard.putNumber("Shooter Pivot Power", m_pivot.get());
    SmartDashboard.putBoolean("Shooter Note In Position", isNoteAtPosition());
  }
}
