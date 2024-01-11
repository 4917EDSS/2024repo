// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {
  TalonFX m_driveMotorFL;
  TalonFX m_driveMotorFR;
  TalonFX m_driveMotorBL;
  TalonFX m_driveMotorBR;
  TalonFX m_driveTurnMotorFL;
  TalonFX m_driveTurnMotorFR;
  TalonFX m_driveTurnMotorRL;
  TalonFX m_driveTurnMotorRR;

  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    /*
    public final static int kDriveMotorFL = 1; // Front left motor
    public final static int kTurningMotorFL = 2;
    public final static int kDriveMotorFR = 3; // Front right motor
    public final static int kTurningMotorFR = 4;
    public final static int kDriveMotorBL = 5; // Back left motor
    public final static int kTurningMotorBL = 6;
    public final static int kDriveMotorBR = 7; // Back right motor
    public final static int kTurningMotorBR = 8;
     */
    m_driveMotorFL = new TalonFX(1);
    m_driveMotorFR = new TalonFX(3);
    m_driveMotorBL = new TalonFX(5);
    m_driveMotorBR = new TalonFX(7);
    m_driveTurnMotorFL = new TalonFX(2);
    m_driveTurnMotorFR = new TalonFX(4);
    m_driveTurnMotorRL = new TalonFX(6);
    m_driveTurnMotorRR = new TalonFX(8);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void drive(double power) {
    m_driveMotorFL.set(power);
    m_driveMotorFR.set(power);
    m_driveMotorBL.set(power);
    m_driveMotorBR.set(power);
  }
}
