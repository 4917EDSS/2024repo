// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class FlywheelSub extends SubsystemBase {
  private final CANSparkMax m_flywheelL =
      new CANSparkMax(Constants.CanIds.kFlywheelL, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_flywheelR =
      new CANSparkMax(Constants.CanIds.kFlywheelR, CANSparkLowLevel.MotorType.kBrushless);

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));

  // Create a new SysId routine for characterizing the shooter.
  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motor(s).
              (Measure<Voltage> volts) -> {
                m_flywheelL.setVoltage(volts.in(Volts));
                m_flywheelR.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the shooter motor.
                log.motor("flywheel")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_flywheelL.getBusVoltage(), Volts))
                    .angularPosition(m_angle.mut_replace((getPositionL()), Rotations))
                    .angularVelocity(m_velocity.mut_replace(getVelocityL(), RotationsPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("shooter")
              this));
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////

  public FlywheelSub() {
    init();
  }

  public void init() {
    m_flywheelL.setInverted(true);
    m_flywheelR.setInverted(false);
    m_flywheelL.setIdleMode(IdleMode.kCoast);
    m_flywheelR.setIdleMode(IdleMode.kCoast);
    m_flywheelL.setSmartCurrentLimit(Constants.Flywheel.kCurrentLimit);
    m_flywheelR.setSmartCurrentLimit(Constants.Flywheel.kCurrentLimit);
    m_flywheelL.getEncoder().setVelocityConversionFactor(Constants.Flywheel.kEncoderConversionFactor);
    m_flywheelR.getEncoder().setVelocityConversionFactor(Constants.Flywheel.kEncoderConversionFactor);
    m_flywheelL.set(0.0);
    m_flywheelR.set(0.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("L Pos", getPositionL());
    SmartDashboard.putNumber("R Pos", getPositionR());
    SmartDashboard.putNumber("L Vel", getVelocityL());
    SmartDashboard.putNumber("R Vel", getVelocityR());
  }

  public double getPositionL() {
    return m_flywheelL.getEncoder().getPosition();
  }

  public double getPositionR() {
    return m_flywheelR.getEncoder().getPosition();
  }

  public double getVelocityL() {
    return m_flywheelL.getEncoder().getVelocity();
  }

  public double getVelocityR() {
    return m_flywheelR.getEncoder().getVelocity();
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}

