// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Majority of code is referenced from
// https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervebot/

package frc.robot.subsystems;

import java.util.logging.Logger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
  private static Logger m_logger = Logger.getLogger(SwerveModule.class.getName());

  // Motors and Encoders
  private final TalonFX m_driveMotor;
  public final TalonFX m_steeringMotor;
  private final CANcoder m_steeringEncoder;
  private final double m_turningEncoderOffset;

  private final StatusSignal<Double> m_driveMotorPosition;
  private final StatusSignal<Double> m_driveMotorVelocity;
  private final StatusSignal<Double> m_driveMotorAcceleration;
  private final StatusSignal<Double> m_driveMotorAmps;

  public final double kDriveDistanceFactor;// 6.52 Circumference(m) * gear ratio 
  public final double kDriveVelocityFactor; // RPM to m/s

  // PID Controllers
  private final PIDController m_drivePID = new PIDController(0.5, 0, 0.0);

  //Profiled PID controller combines a trapezoidal motion profile and PID
  //A trapezoidal motion profile splits the motion into three segments, acceleration, constant velocity, and deceleration
  //It takes maximum velocity and acceleration as parameters
  private final ProfiledPIDController m_steeringPID =
      new ProfiledPIDController(0.5, 0, 0, new TrapezoidProfile.Constraints(
          Constants.ModuleConstants.kMaxModuleAngularSpeed,
          Constants.ModuleConstants.kMaxModuleAngularAcceleration));

  // These predict PID values which makes it work in real time
  //It has three parameters, ks, kv, ka
  //ks is the voltage required to overcome the static friction of the motor. Some constant portion of the motor's power will always be dedicated to
  //overcoming this friction
  //kv is the voltage needed to hold the motors at a constant velocity, the relationship between speed and voltage is almost completely linear
  //ka is the voltage needed to induce a given acceleration in the motor shaft, like kv the relationship is almost linear
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.02, 0.2);
  private final SimpleMotorFeedforward m_steeringFeedforward = new SimpleMotorFeedforward(0.0, 0);

  public SwerveModule(int driveMotorID, int steeringMotorID, int steeringEncoderID, double absoluteEncoderOffsetRad) { // Drive motor ID, Steering motor ID
    // Initialize motors and sensors

    //Creates drive and steering motors/encoders
    m_driveMotor = new TalonFX(driveMotorID);
    m_steeringMotor = new TalonFX(steeringMotorID);
    m_steeringEncoder = new CANcoder(steeringEncoderID);

    m_driveMotorPosition = m_driveMotor.getPosition();
    m_driveMotorVelocity = m_driveMotor.getVelocity();
    m_driveMotorAcceleration = m_driveMotor.getAcceleration();
    m_driveMotorAmps = m_driveMotor.getSupplyCurrent();

    TalonFXConfigurator driveConfigurator = m_driveMotor.getConfigurator();
    TalonFXConfigurator steeringConfigurator = m_steeringMotor.getConfigurator();

    CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = 10;
    limitConfigs.StatorCurrentLimitEnable = true;
    driveConfigurator.apply(limitConfigs);

    limitConfigs.StatorCurrentLimit = 10;
    limitConfigs.StatorCurrentLimitEnable = true;
    steeringConfigurator.apply(limitConfigs);

    MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
    outputConfigs.DutyCycleNeutralDeadband = 0.02; // Ignore values below 2%
    outputConfigs.Inverted = InvertedValue.CounterClockwise_Positive; // Invert = Clockwise
    outputConfigs.NeutralMode = NeutralModeValue.Brake;
    driveConfigurator.apply(outputConfigs);

    outputConfigs.DutyCycleNeutralDeadband = 0.02; // Ignore values below 2%
    outputConfigs.Inverted = InvertedValue.CounterClockwise_Positive; // Invert = Clockwise
    outputConfigs.NeutralMode = NeutralModeValue.Brake;
    steeringConfigurator.apply(outputConfigs);

    //Changes gear ratio based on which roborio it detects
    double GearRatio;
    if(Constants.Drivetrain.serialNumber.equals(Constants.RobotSpecific.PracticeSerialNumber)) {
      GearRatio = Constants.RobotSpecific.Practice.kGearRatio;
    } else if(Constants.Drivetrain.serialNumber.equals(Constants.RobotSpecific.CompetitionSerialNumber)) {
      GearRatio = Constants.RobotSpecific.Competition.kGearRatio;
    } else {
      GearRatio = Constants.RobotSpecific.Unknown.kGearRatio;
    }
    //Calculates the circumfrence of the wheel, and divides it by the gear ratio
    kDriveDistanceFactor = (GearRatio) / (Math.PI * Constants.ModuleConstants.kWheelDiameter);
    kDriveVelocityFactor = kDriveDistanceFactor / 60.0;

    FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
    feedbackConfigs.SensorToMechanismRatio = kDriveDistanceFactor;
    driveConfigurator.apply(feedbackConfigs);

    // feedbackConfigs.SensorToMechanismRatio = 1.0;
    // steeringConfigurator.apply(feedbackConfigs);

    // Make it loop from -PI to PI
    m_steeringPID.enableContinuousInput(-Math.PI, Math.PI);

    // Set conversion factors

    m_turningEncoderOffset = absoluteEncoderOffsetRad;

    m_driveMotor.setPosition(0.0);

    init();
  }

  public void init() {
    //Prints "Initializing a SwerveModule" to the logs
    m_logger.info("Initializing a SwerveModule");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getTurningRotation() { // In radians
    //Gets the steering encoder value as a double
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

  //Sets the p value of the PID
  public void setP(double val) {
    m_drivePID.setP(val);
  }

  //Gets the p value of the PID
  public double getP() {
    return m_drivePID.getP();
  }

  public double getTurningEncoder() { // Without any offset
    double position = (m_steeringEncoder.getAbsolutePosition().getValueAsDouble());// -0.5 to 0.5

    position *= Math.PI * 2.0; // Converts from -0.5 to 0.5 to -PI to PI

    // Calculate offsets
    if(position < -Math.PI) {
      double a = -position - Math.PI;
      position = Math.PI - a;
    } else if(position > Math.PI) {
      double a = position - Math.PI;
      position = -Math.PI + a;
    }

    return position;
  }

  //Gets the swerve module state (velocity and rotation)

  public SwerveModuleState getState() { // Swerve states are what Kinematics uses for calculations
    return new SwerveModuleState(getDriveVelocity(),
        new Rotation2d(getTurningRotation()));
  }

  //Gets the swerve module position (position and rotation)
  public SwerveModulePosition getPosition() { // Get current SwerveModule position
    return new SwerveModulePosition(getDrivePosition(),
        new Rotation2d(getTurningRotation()));
  }

  public double getDrivePosition() {
    m_driveMotorPosition.refresh();
    return m_driveMotorPosition.getValueAsDouble();
  }

  public double getDriveVelocity() {
    m_driveMotorVelocity.refresh();
    return m_driveMotorVelocity.getValueAsDouble();
  }

  public double getDriveAcceleration() {
    m_driveMotorAcceleration.refresh();
    return m_driveMotorAcceleration.getValueAsDouble();
  }

  public double getDriveAmps() {
    m_driveMotorAmps.refresh();
    return m_driveMotorAmps.getValueAsDouble();
  }

  //Gets the drive motor power
  public double testGetPower() {
    return m_driveMotor.get();
  }

  public void testSetPower(double power) {
    m_steeringMotor.set(power);
  }

  public void setState(SwerveModuleState state) { // Set the proper motor speeds and directions for the given state
    var steeringRotation = new Rotation2d(getTurningRotation());

    // TODO ONCMP
    // This deadband is actually surprisingly high. This is likely what caused us so much trouble
    // with aligning to vision, as we can't move slowly. We should signifcantly lower this (maybe like 0.01).
    // We already have deadbands in the actual joystick code.
    if(Math.abs(state.speedMetersPerSecond) < 0.02) { // deadband
      stop();
      return;
    }

    // Optimize the state so it doesn't rotate 270 degrees for a 90 degree turn
    SwerveModuleState betterState = SwerveModuleState.optimize(state, steeringRotation);

    // Apparently this makes driving smoother by using the "cosine of angle error"
    betterState.speedMetersPerSecond *= betterState.angle.minus(steeringRotation).getCos();

    // Calculate power using velocity and PID
    double driveOutput = m_drivePID.calculate(getDriveVelocity(), betterState.speedMetersPerSecond);
    double driveFeedforward = m_driveFeedforward.calculate(betterState.speedMetersPerSecond);
    // Calculate steering power using difference in angle
    double steeringOutput =
        m_steeringPID.calculate(getTurningRotation(), betterState.angle.getRadians());
    double steeringFeedforward = m_steeringFeedforward.calculate(m_steeringPID.getSetpoint().velocity);

    //Gets drive and steering power by combining output with feedforward
    double drivePower = driveOutput + driveFeedforward;
    double steeringPower = steeringOutput + steeringFeedforward;
    m_driveMotor.set(MathUtil.clamp(drivePower, -1.0, 1.0)); // Safety first

    m_steeringMotor.set(MathUtil.clamp(steeringPower, -1.0, 1.0)); // This clamp doesn't actually affect the PID calculation so it can just be a -1 to 1 clamp
  }

  //Method to stop drive and steering motors
  public void stop() {
    m_driveMotor.set(0);
    m_steeringMotor.set(0);
  }


}
