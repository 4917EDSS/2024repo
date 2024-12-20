// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class FeederSub extends SubsystemBase {
  /** Creates a new FeederSub. */
  public boolean finishedNoteIntake = false;

  private final CANSparkMax m_intakeRollers =
      new CANSparkMax(Constants.CanIds.kIntakeRollers, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_upperFeeder =
      new CANSparkMax(Constants.CanIds.kUpperFeeder, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_lowerFeeder =
      new CANSparkMax(Constants.CanIds.kLowerFeeder, CANSparkLowLevel.MotorType.kBrushless);

  public FeederSub() {

    init();
  }

  public void init() {
    m_intakeRollers.setSmartCurrentLimit(40);

    m_upperFeeder.setInverted(true);
    m_lowerFeeder.setInverted(false);
    /*
     * if(Constants.Drivetrain.serialNumber.equals(Constants.RobotSpecific.PracticeSerialNumber)) {
     * m_lowerFeeder.setInverted(Constants.RobotSpecific.Practice.kInvertLowerFeeder);
     * } else if(Constants.Drivetrain.serialNumber.equals(Constants.RobotSpecific.CompetitionSerialNumber)) {
     * m_lowerFeeder.setInverted(Constants.RobotSpecific.Competition.kInvertLowerFeeder);
     * } else {
     * m_lowerFeeder.setInverted(Constants.RobotSpecific.Unknown.kInvertLowerFeeder);
     * }
     */

    m_upperFeeder.setIdleMode(IdleMode.kBrake);
    m_lowerFeeder.setIdleMode(IdleMode.kBrake);

    m_upperFeeder.setSmartCurrentLimit(40);
    m_lowerFeeder.setSmartCurrentLimit(40);

    setIntakeMotors(0.0);
    spinBothFeeders(0, 0);
  }

  // positive power intakes 
  public void setIntakeMotors(double power) {
    m_intakeRollers.set(power);
  }

  public void spinUpperFeeder(double power) {
    m_upperFeeder.set(power);
  }

  public void spinLowerFeeder(double power) {
    m_lowerFeeder.set(power);
  }

  public void spinBothFeeders(double lowerPower, double upperPower) {
    spinLowerFeeder(lowerPower);
    spinUpperFeeder(upperPower);
  }

  public void spinAllFeedersAndIntake(double lowerPower, double upperPower, double power) {
    spinLowerFeeder(lowerPower);
    spinUpperFeeder(upperPower);
    m_intakeRollers.set(power);
  }

  @Override
  public void periodic() {
    RobotContainer.noteInFeeder = finishedNoteIntake;
    // This method will be called once per scheduler run
  }
}
