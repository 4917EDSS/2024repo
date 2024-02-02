// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Majority of code is referenced from
// https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervebot/

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  public static final class ModuleConstants {

    // Maxes
    public static final double kMaxModuleAngularSpeed = 8.0 * Math.PI; // In Radians Per Second
    public static final double kMaxModuleAngularAcceleration = 100.0 * Math.PI; // In Radians Per Second Squared

    // Conversion factors
    public static final double kDriveDistanceFactor = (Math.PI * 0.1016) / (6.0 * 1.2);//Circumference(m) * gear ratio

    public static final double kDriveVelocityFactor = kDriveDistanceFactor / 60.0; // RPM to m/s

  }

  // Motors and Encoders

  private final CANSparkMax m_driveMotor;
  private final TalonFX m_steeringMotor;
  private final RelativeEncoder m_driveEncoder;
  private final CANcoder m_steeringEncoder;
  private final double m_turningEncoderOffset;

  // PID Controllers

  private final PIDController m_drivePID = new PIDController(0.0, 0, 0.0); // TODO: Tune the Driving PID

  private final ProfiledPIDController m_steeringPID =
      new ProfiledPIDController(0.5, 0, 0, new TrapezoidProfile.Constraints(
          ModuleConstants.kMaxModuleAngularSpeed,
          ModuleConstants.kMaxModuleAngularAcceleration)); // TODO: Also tune the Steering PID

  // These predict PID values which makes it work in real time
  // TODO: Feed forward will probably need tuning as well
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.1, 0.01); // Distance, Velocity
  private final SimpleMotorFeedforward m_steeringFeedforward = new SimpleMotorFeedforward(0, 0);


  public SwerveModule(int driveMotorID, int steeringMotorID, int steeringEncoderID, double absoluteEncoderOffsetRad) { // Drive motor ID, Steering motor ID

    // Initialize motors and sensors
    m_driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    m_steeringMotor = new TalonFX(steeringMotorID);
    m_driveEncoder = m_driveMotor.getEncoder();
    m_steeringEncoder = new CANcoder(steeringEncoderID);

    // Make it loop from -PI to PI
    m_driveMotor.setIdleMode(IdleMode.kBrake);
    m_steeringMotor.setNeutralMode(NeutralModeValue.Brake);
    m_steeringPID.enableContinuousInput(-Math.PI, Math.PI);


    // Set conversion factors
    m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveVelocityFactor);
    m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveDistanceFactor);

    m_turningEncoderOffset = absoluteEncoderOffsetRad;

  }

  public double getTurningRotation() { // In radians
    double position = (m_steeringEncoder.getAbsolutePosition().getValueAsDouble());// -0.5 to 0.5

    position *= Math.PI * 2.0; // Converts from -0.5 to 0.5 to -PI to PI
    position -= m_turningEncoderOffset;

    // Calculate offsets
    if(position < -Math.PI) {
      double a = -position - Math.PI;
      position = Math.PI - a;
    } else if(position > Math.PI) {
      double a = position - Math.PI;
      position = -Math.PI + a;
    }
    // compensate for the offset's effect on the absolute encoder roll-over
    // want values from -pi to +pi in rad
    // if(position - m_turningEncoderOffset < -Math.PI) {
    //   position += ((2.0 * Math.PI) - m_turningEncoderOffset);
    // } else if(position - m_turningEncoderOffset > Math.PI) {
    //   position -= ((2.0 * Math.PI) - m_turningEncoderOffset);
    // }


    return position;// - m_turningEncoderOffset;
  }

  public SwerveModuleState getState() { // Swerve states are what Kinematics uses for calculations
    return new SwerveModuleState(m_driveEncoder.getVelocity(),
        new Rotation2d(getTurningRotation()));
  }

  public SwerveModulePosition getPosition() { // Get current SwerveModule position
    return new SwerveModulePosition(m_driveEncoder.getPosition(),
        new Rotation2d(getTurningRotation()));
  }

  public void setState(SwerveModuleState state) { // Set the proper motor speeds and directions for the given state
    var steeringRotation = new Rotation2d(getTurningRotation());

    if(Math.abs(state.speedMetersPerSecond) < 0.1) { // deadband
      stop();
      return;
    }

    // Optimize the state so it doesn't rotate 270 degrees for a 90 degree turn
    SwerveModuleState betterState = SwerveModuleState.optimize(state, steeringRotation);

    // Apparently this makes driving smoother by using the "cosine of angle error"
    betterState.speedMetersPerSecond *= betterState.angle.minus(steeringRotation).getCos();

    // Calculate power using velocity and PID
    final double driveOutput = m_drivePID.calculate(m_driveEncoder.getVelocity(), betterState.speedMetersPerSecond);
    final double driveFeedforward = m_driveFeedforward.calculate(betterState.speedMetersPerSecond);
    // Calculate steering power using difference in angle
    final double steeringOutput =
        m_steeringPID.calculate(getTurningRotation(), betterState.angle.getRadians());
    final double steeringFeedforward = m_steeringFeedforward.calculate(m_steeringPID.getSetpoint().velocity);

    // Clamp these as needed
    double drivePower = driveOutput + driveFeedforward;
    double steeringPower = steeringOutput + steeringFeedforward;
    m_driveMotor.set(drivePower); // Safety first
    m_steeringMotor.set(Math.min(Math.max(steeringPower, -0.4), 0.4));
  }

  public void stop() {
    m_driveMotor.set(0);
    m_steeringMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
