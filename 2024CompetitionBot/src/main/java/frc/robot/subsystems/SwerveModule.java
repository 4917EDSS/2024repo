// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.sensors.AbsoluteSensorRange;
// import com.ctre.phoenix6.hardware.CANCoder;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.FeedbackConfigs;
// import com.ctre.phoenix.sensors.SensorTimeBase;
// import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

// https://v6.docs.ctr-electronics.com/en/stable/docs/migration/migration-guide/index.html -Pheonix 5 to Pheonix 6
// migration

public class SwerveModule {
  /**
   * Constants for this module
   */
  public static final class ModuleConstants {
    public static final double kMaxDriveMPerS = 1.0; // TODO: To be measured
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 10 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 10 * Math.PI;

    //public static final int kDriveEncoderCPR = 2048; // For CTRE TalonFX built-in encoder
    //public static final double kWheelDiameterMeters = 0.1016; // 4"

    public static final double kDriveEncoderDistancePerPulseMmPerTick = (1000.0 / 51782); // Millimeters per tick
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

  // Member variables
  private final TalonFX m_driveMotor;
  private final CANSparkMax m_turningMotor;
  private final CANcoder m_turningEncoder;

  private final PIDController m_drivePIDController =
      new PIDController(1.0, 0, 0);
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          ModuleConstants.kPModuleTurningController,
          0,
          0,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));
  private double m_turningEncoderOffset;

  /**
   * Construct the SwerveModule
   * 
   * @param driveMotorId CAN ID for the drive motor
   * @param turningMotorId CAN ID for the turning motor
   * @param turningEncoderId CAN ID for the turning encoder
   * @param reverseTurningEncoderDirection Set to true to reverse the default sensor direction
   * @param turningEncoderOffsetRad Value encoder shows when wheel is facing forward
   */
  // public SwerveModule(int driveMotorId, int turningMotorId, int turningEncoderId,
  //     boolean reverseTurningEncoderDirection, double turningEncoderOffsetRad) {

  public SwerveModule(int driveMotorId, int turningMotorId, int turningEncoderId,
      SensorDirectionValue kReverseTurningEncoderDirection, double turningEncoderOffsetRad) {

    // Create all the hardware objects
    m_driveMotor = new TalonFX(driveMotorId);
    m_turningMotor = new CANSparkMax(turningMotorId, CANSparkLowLevel.MotorType.kBrushless);
    m_turningEncoder = new CANcoder(turningEncoderId);

    m_driveMotor.setInverted(true);

    {
      //section for setting the feedback configs. 
      // venkat had this in a section block as there are multiple lines
      // to achieve this vs the phoenix5 approach which was just a single line.

      //  m_driveMotor.configSelectedFeedbackCoefficient(ModuleConstants.kDriveEncoderDistancePerPulseMmPerTick);
      FeedbackConfigs fbconfigs = new FeedbackConfigs();
      fbconfigs.withRotorToSensorRatio(ModuleConstants.kDriveEncoderDistancePerPulseMmPerTick);
      fbconfigs.withSensorToMechanismRatio(1);
      m_driveMotor.getConfigurator().apply(fbconfigs);
    }


    // Configure all the hardware objects as needed (including drive motor encoder)
    {
      // m_turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinus180);
      // m_turningEncoder.configSensorDirection(reverseTurningEncoderDirection); // False means positive rotation occurs when magnet is spun counter-clockwise when observer is facing the LED side of CANCoder.


      //We dont know how to fix this, based on the below conversations we think setting
      // the sensor to mechanism ratio will solve this out. !!!
      // replaced by code: fbconfigs.withSensorToMechanismRatio(1);
      // https://chiefdelphi.com/t/ctre-2024-phoenix-software-now-available/449023/11

      // m_turningEncoder.configFeedbackCoefficient(ModuleConstants.kTurningEncoderDistancePerPulse, "radians",
      //     SensorTimeBase.PerSecond);


      CANcoderConfiguration configs = new CANcoderConfiguration();
      configs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
      configs.MagnetSensor.SensorDirection = kReverseTurningEncoderDirection;
      // configs.MagnetSensor.MagnetOffset

      //write these configs to the cancoder
      m_turningEncoder.getConfigurator().apply(configs);
    }


    // Configure the PID controllers
    // Limit the PID Controller's input range between -pi and pi and set the input to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    m_turningEncoderOffset = turningEncoderOffsetRad;

    resetEncoders();
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocityMPerS(), new Rotation2d(getTurningPositionAbsoluteRad()));
  }

  public SwerveModulePosition getPosition() {
    // return new SwerveModulePosition(m_driveMotor.getSelectedSensorPosition() / 1000.0,
    // new Rotation2d(getTurningPositionAbsoluteRad()));
    return new SwerveModulePosition(m_driveMotor.getPosition().getValueAsDouble() / 1000.0,
        new Rotation2d(getTurningPositionAbsoluteRad()));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    if(Math.abs(desiredState.speedMetersPerSecond) < 0.1) {
      stop();
      return;
    }

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(getTurningPositionAbsoluteRad()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        //m_drivePIDController.calculate(getDriveVelocityMPerS(), state.speedMetersPerSecond);
        state.speedMetersPerSecond / ModuleConstants.kMaxDriveMPerS;

    // Calculate the turning motor output from the turning PID controller.
    double tpar = getTurningPositionAbsoluteRad();
    double sar = state.angle.getRadians();
    final double turnOutput =
        m_turningPIDController.calculate(tpar, sar);
    if(m_turningEncoderOffset == Constants.SwerveModuleConstants.kAbsoluteEncoderOffsetBL) {
      SmartDashboard.putNumber("TurnOutput", turnOutput);
      SmartDashboard.putNumber("CurTPos", tpar);
      SmartDashboard.putNumber("TargetTPos", sar);
      SmartDashboard.putNumber("Delta", sar - tpar);

    }

    // Calculate the turning motor output from the turning PID controller.
    // m_driveMotor.set(ControlMode.PercentOutput, driveOutput);
    m_driveMotor.setControl(new DutyCycleOut(driveOutput));
    m_turningMotor.set(turnOutput);
  }

  public void stop() {
    // m_driveMotor.set(ControlMode.PercentOutput, 0);
    m_driveMotor.setControl(new DutyCycleOut(0));
    m_turningMotor.set(0);
  }


  public void setPID(boolean isDrive, double p, double i, double d) {
    if(isDrive) {
      if(m_drivePIDController.getP() != p) {
        m_drivePIDController.setP(p);
      }
      if(m_drivePIDController.getI() != i) {
        m_drivePIDController.setI(i);
      }
      if(m_drivePIDController.getD() != d) {
        m_drivePIDController.setD(d);
      }
    } else {
      if(m_turningPIDController.getP() != p) {
        m_turningPIDController.setP(p);
      }
      if(m_turningPIDController.getI() != i) {
        m_turningPIDController.setI(i);
      }
      if(m_turningPIDController.getD() != d) {
        m_turningPIDController.setD(d);
      }
    }
  }

  public double getDrivePositionM() {
    // return m_driveMotor.getSelectedSensorPosition();
    return m_driveMotor.getPosition().getValueAsDouble();
  }

  public double getTurningPosition() {
    return m_turningMotor.getEncoder().getPosition();
  }

  public double getDriveVelocityMPerS() {
    // return m_driveMotor.getSelectedSensorVelocity() * ModuleConstants.kDriveVelocityFactor;
    return m_driveMotor.getVelocity().getValueAsDouble() * ModuleConstants.kDriveVelocityFactor;
  }

  public double getTurningVelocity() {
    return m_turningMotor.getEncoder().getVelocity();
  }

  public double getTurningPositionAbsoluteRad() {
    // double position = m_turningEncoder.getAbsolutePosition() - m_turningEncoderOffset;

    double position = m_turningEncoder.getPosition().getValueAsDouble() - m_turningEncoderOffset;


    // Compensate for the offset's effect on the absolute encorder roll-over
    // Want values from -PI to +PI (radians)
    if(position < -Math.PI) {
      position += (2 * Math.PI);
    } else if(position > Math.PI) {
      position -= (2 * Math.PI);
    }

    return position;
  }

  public void resetEncoders() {
    // m_driveMotor.setSelectedSensorPosition(0.0);
    m_driveMotor.setPosition(0.0);
    // m_turningMotor.getEncoder().setPosition(0.0); // Can't reset this absolute encoder
  }
}
