// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  public static final class ModuleConstants {
    public static final double kMaxDriveMPerS = 1.0; // TODO: To be measured
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 10 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 10 * Math.PI;

    public static final double kDriveDistanceFactor = (1000.0 / 51782); // Millimeters per tick
    /**
     * mm/100ms to m/s
     */
    public static final double kDriveVelocityFactor = (10.0 / 1000.0); // mm/100ms to m/s 

    public static final int kTurningEncoderCPR = 4096; // For CTRE CANCoder magnetic encoder
    // Assumes the encoders are on a 1:1 reduction with the module shaft.
    public static final double kTurningEncoderDistancePerPulse = (2 * Math.PI) / (double) kTurningEncoderCPR; // Radians per pulse

    public static final double kPModuleTurningController = 0.5;
    public static final double kPModuleDriveController = 1;
  }

  private final CANSparkMax m_driveMotor;
  private final TalonFX m_steeringMotor;
  private final RelativeEncoder m_driveEncoder;

  private final PIDController m_drivePID = new PIDController(0.1, 0, 0);
  private final ProfiledPIDController m_steeringPID =
      new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(
          ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
          ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));


  // private final Spark

  public SwerveModule(int driveMotorID, int steeringMotorID) {
    // Initialize motors and sensors
    m_driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    m_steeringMotor = new TalonFX(steeringMotorID);
    m_driveEncoder = m_driveMotor.getEncoder();


    // Set conversion factors
    m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveVelocityFactor);
    m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveDistanceFactor);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getVelocity(),
        new Rotation2d(m_steeringMotor.getPosition().getValueAsDouble()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_driveEncoder.getPosition(),
        new Rotation2d(m_steeringMotor.getPosition().getValueAsDouble()));
  }

  public void setState(SwerveModuleState state) {
    //var encoderRotation = m_steeringMotor.getDis
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
