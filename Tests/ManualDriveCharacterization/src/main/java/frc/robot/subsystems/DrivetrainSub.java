// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSub extends SubsystemBase {
  /** Creates a new DrivetrainSub. */
  private final CANSparkMax m_driveMotorFL;
  private final CANSparkMax m_driveMotorFR;
  private final CANSparkMax m_driveMotorBL;
  private final CANSparkMax m_driveMotorBR;

  public final TalonFX m_steeringMotorFL;
  public final TalonFX m_steeringMotorFR;
  public final TalonFX m_steeringMotorBL;
  public final TalonFX m_steeringMotorBR;

  private final RelativeEncoder m_driveEncoderFL;
  private final RelativeEncoder m_driveEncoderFR;
  private final RelativeEncoder m_driveEncoderBL;
  private final RelativeEncoder m_driveEncoderBR;

  private final CANcoder m_steeringAbsoluteEncoderFL;
  private final CANcoder m_steeringAbsoluteEncoderFR;
  private final CANcoder m_steeringAbsoluteEncoderBL;
  private final CANcoder m_steeringAbsoluteEncoderBR;

  private final AHRS m_gyro;

  private boolean m_isRotateMode;
  private boolean m_autoRotate;

  public DrivetrainSub() {
    // Create hardware objects
    m_driveMotorFL = new CANSparkMax(Constants.CanIds.kDriveMotorFL, MotorType.kBrushless);
    m_driveMotorFR = new CANSparkMax(Constants.CanIds.kDriveMotorFR, MotorType.kBrushless);
    m_driveMotorBL = new CANSparkMax(Constants.CanIds.kDriveMotorBL, MotorType.kBrushless);
    m_driveMotorBR = new CANSparkMax(Constants.CanIds.kDriveMotorBR, MotorType.kBrushless);

    m_steeringMotorFL = new TalonFX(Constants.CanIds.kSteeringMotorFL);
    m_steeringMotorFR = new TalonFX(Constants.CanIds.kSteeringMotorFR);
    m_steeringMotorBL = new TalonFX(Constants.CanIds.kSteeringMotorBL);
    m_steeringMotorBR = new TalonFX(Constants.CanIds.kSteeringMotorBR);

    m_driveEncoderFL = m_driveMotorFL.getEncoder();
    m_driveEncoderFR = m_driveMotorFR.getEncoder();
    m_driveEncoderBL = m_driveMotorBL.getEncoder();
    m_driveEncoderBR = m_driveMotorBR.getEncoder();

    m_steeringAbsoluteEncoderFL = new CANcoder(Constants.CanIds.kEncoderFL);
    m_steeringAbsoluteEncoderFR = new CANcoder(Constants.CanIds.kEncoderFR);
    m_steeringAbsoluteEncoderBL = new CANcoder(Constants.CanIds.kEncoderBL);
    m_steeringAbsoluteEncoderBR = new CANcoder(Constants.CanIds.kEncoderBR);

    m_gyro = new AHRS(SPI.Port.kMXP);

    // Configure motors and encoders
    m_driveMotorFL.setIdleMode(IdleMode.kBrake);
    m_driveMotorFR.setIdleMode(IdleMode.kBrake);
    m_driveMotorBL.setIdleMode(IdleMode.kBrake);
    m_driveMotorBR.setIdleMode(IdleMode.kBrake);

    m_steeringMotorFL.setNeutralMode(NeutralModeValue.Brake);
    m_steeringMotorFR.setNeutralMode(NeutralModeValue.Brake);
    m_steeringMotorBL.setNeutralMode(NeutralModeValue.Brake);
    m_steeringMotorBR.setNeutralMode(NeutralModeValue.Brake);

    m_driveMotorFL.setSmartCurrentLimit(Constants.Drivetrain.driveCurrentLimit);
    m_driveMotorFR.setSmartCurrentLimit(Constants.Drivetrain.driveCurrentLimit);
    m_driveMotorBL.setSmartCurrentLimit(Constants.Drivetrain.driveCurrentLimit);
    m_driveMotorBR.setSmartCurrentLimit(Constants.Drivetrain.driveCurrentLimit);

    m_driveEncoderFL.setPositionConversionFactor(Constants.Drivetrain.kDriveDistanceFactor);
    m_driveEncoderFR.setPositionConversionFactor(Constants.Drivetrain.kDriveDistanceFactor);
    m_driveEncoderBL.setPositionConversionFactor(Constants.Drivetrain.kDriveDistanceFactor);
    m_driveEncoderBR.setPositionConversionFactor(Constants.Drivetrain.kDriveDistanceFactor);

    m_driveEncoderFL.setVelocityConversionFactor(Constants.Drivetrain.kDriveVelocityFactor);
    m_driveEncoderFR.setVelocityConversionFactor(Constants.Drivetrain.kDriveVelocityFactor);
    m_driveEncoderBL.setVelocityConversionFactor(Constants.Drivetrain.kDriveVelocityFactor);
    m_driveEncoderBR.setVelocityConversionFactor(Constants.Drivetrain.kDriveVelocityFactor);

    // Note:  Resetting the gyro here doesn't work as the NavX is not ready for it
    init();
  }

  public void init() {
    m_isRotateMode = false;
    m_autoRotate = true;

    m_driveMotorFL.set(0.0);
    m_driveMotorFR.set(0.0);
    m_driveMotorBL.set(0.0);
    m_driveMotorBR.set(0.0);

    m_driveEncoderFL.setPosition(0.0);
    m_driveEncoderFR.setPosition(0.0);
    m_driveEncoderBL.setPosition(0.0);
    m_driveEncoderBR.setPosition(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    final double kP = 0.4;
    final double kMaxPower = 0.25;
    final double kTolerance = 0.01;

    double currentAngleRad = 0;
    double power = 0;
    double targetAngle = 0.0;

    // Set the angle of the wheels
    // FL
    // Encoder returns angle in turns (-0.5 to 0.5) so convert to radians and then subtract offset
    currentAngleRad = (m_steeringAbsoluteEncoderFL.getAbsolutePosition().getValueAsDouble() * (Math.PI * 2))
        - Constants.Drivetrain.kAbsoluteEncoderOffsetFL;
    // Compensate for offset changing rollover point
    if(currentAngleRad < -Math.PI) {
      currentAngleRad = Math.PI - (-currentAngleRad - Math.PI);
    } else if(currentAngleRad > Math.PI) {
      currentAngleRad = -Math.PI + (currentAngleRad - Math.PI);
    }
    targetAngle = m_isRotateMode ? -Math.PI / 4 : 0.0;
    SmartDashboard.putNumber("FL T Ang", targetAngle);
    SmartDashboard.putNumber("FL C Ang", currentAngleRad);
    SmartDashboard.putNumber("FL Raw Ang",
        m_steeringAbsoluteEncoderFL.getAbsolutePosition().getValueAsDouble() * (Math.PI * 2));
    if(m_autoRotate) {
      if(Math.abs(targetAngle - currentAngleRad) < kTolerance) {
        power = 0.0;
      } else {
        power = MathUtil.clamp((targetAngle - currentAngleRad) * kP, -kMaxPower, kMaxPower);
      }
      m_steeringMotorFL.set(power);
      SmartDashboard.putNumber("FL Pwr", power);
    }

    // FR
    currentAngleRad = (m_steeringAbsoluteEncoderFR.getAbsolutePosition().getValueAsDouble() * (Math.PI * 2))
        - Constants.Drivetrain.kAbsoluteEncoderOffsetFR;
    // Compensate for offset changing rollover point
    if(currentAngleRad < -Math.PI) {
      currentAngleRad = Math.PI - (-currentAngleRad - Math.PI);
    } else if(currentAngleRad > Math.PI) {
      currentAngleRad = -Math.PI + (currentAngleRad - Math.PI);
    }
    targetAngle = m_isRotateMode ? -3 * Math.PI / 4 : 0.0;
    SmartDashboard.putNumber("FR T Ang", targetAngle);
    SmartDashboard.putNumber("FR C Ang", currentAngleRad);
    SmartDashboard.putNumber("FR Raw Ang",
        m_steeringAbsoluteEncoderFR.getAbsolutePosition().getValueAsDouble() * (Math.PI * 2));
    if(m_autoRotate) {
      if(Math.abs(targetAngle - currentAngleRad) < kTolerance) {
        power = 0.0;
      } else {
        power = MathUtil.clamp((targetAngle - currentAngleRad) * kP, -kMaxPower, kMaxPower);
      }
      m_steeringMotorFR.set(power);
      SmartDashboard.putNumber("FR Pwr", power);
    }

    // BL
    currentAngleRad = (m_steeringAbsoluteEncoderBL.getAbsolutePosition().getValueAsDouble() * (Math.PI * 2))
        - Constants.Drivetrain.kAbsoluteEncoderOffsetBL;
    // Compensate for offset changing rollover point
    if(currentAngleRad < -Math.PI) {
      currentAngleRad = Math.PI - (-currentAngleRad - Math.PI);
    } else if(currentAngleRad > Math.PI) {
      currentAngleRad = -Math.PI + (currentAngleRad - Math.PI);
    }
    targetAngle = m_isRotateMode ? Math.PI / 4 : 0.0;
    SmartDashboard.putNumber("BL T Ang", targetAngle);
    SmartDashboard.putNumber("BL C Ang", currentAngleRad);
    SmartDashboard.putNumber("BL Raw Ang",
        m_steeringAbsoluteEncoderBL.getAbsolutePosition().getValueAsDouble() * (Math.PI * 2));
    if(m_autoRotate) {
      if(Math.abs(targetAngle - currentAngleRad) < kTolerance) {
        power = 0.0;
      } else {
        power = MathUtil.clamp((targetAngle - currentAngleRad) * kP, -kMaxPower, kMaxPower);
      }
      m_steeringMotorBL.set(power);
      SmartDashboard.putNumber("BL Pwr", power);
    }

    // BR
    currentAngleRad = (m_steeringAbsoluteEncoderBR.getAbsolutePosition().getValueAsDouble() * (Math.PI * 2))
        - Constants.Drivetrain.kAbsoluteEncoderOffsetBR;
    // Compensate for offset changing rollover point
    if(currentAngleRad < -Math.PI) {
      currentAngleRad = Math.PI - (-currentAngleRad - Math.PI);
    } else if(currentAngleRad > Math.PI) {
      currentAngleRad = -Math.PI + (currentAngleRad - Math.PI);
    }
    targetAngle = m_isRotateMode ? 3 * Math.PI / 4 : 0.0;
    SmartDashboard.putNumber("BR T Ang", targetAngle);
    SmartDashboard.putNumber("BR C Ang", currentAngleRad);
    SmartDashboard.putNumber("BR Raw Ang",
        m_steeringAbsoluteEncoderBR.getAbsolutePosition().getValueAsDouble() * (Math.PI * 2));
    if(m_autoRotate) {
      if(Math.abs(targetAngle - currentAngleRad) < kTolerance) {
        power = 0.0;
      } else {
        power = MathUtil.clamp((targetAngle - currentAngleRad) * kP, -kMaxPower, kMaxPower);
      }
      m_steeringMotorBR.set(power);
      SmartDashboard.putNumber("BR Pwr", power);
    }

    // Display other values
    SmartDashboard.putNumber("FL Dist", m_driveEncoderFL.getPosition());
    SmartDashboard.putNumber("FR Dist", m_driveEncoderFR.getPosition());
    SmartDashboard.putNumber("BL Dist", m_driveEncoderBL.getPosition());
    SmartDashboard.putNumber("BR Dist", m_driveEncoderBR.getPosition());
    SmartDashboard.putNumber("FL Vel", m_driveEncoderFL.getVelocity());
    SmartDashboard.putNumber("FR Vel", m_driveEncoderFR.getVelocity());
    SmartDashboard.putNumber("BL Vel", m_driveEncoderBL.getVelocity());
    SmartDashboard.putNumber("BR Vel", m_driveEncoderBR.getVelocity());
    SmartDashboard.putNumber("Gyro Ang", m_gyro.getAngle());
    SmartDashboard.putNumber("Gyro Vel", m_gyro.getRate());
  }

  /**
   * Set the power of the drive motors,
   * Also sets the power to 0 if the input power is less than the deadband.
   * 
   * @param power 0.0 to 1.0 for fowards, 0.0 to -1.0 for backwards
   * @param deadband size of band were a small power will translate to zero power
   */
  public void setDrivePower(double power, double deadband) {
    if(Math.abs(power) < deadband) {
      power = 0.0;
    }

    m_driveMotorFL.set(power);
    m_driveMotorFR.set(power);
    m_driveMotorBL.set(power);
    m_driveMotorBR.set(power);
  }

  /**
   * Sets the swerve wheel angle to straight or angles for 360deg rotation
   * 
   * @param rotateMode true for wheels angled at 45ded in a circle, false for wheels straight forwards/backwards
   */
  public void setRotateMode(boolean rotateMode) {
    m_isRotateMode = rotateMode;
  }

  public void setAutoRotate(boolean autoRotate) {
    m_autoRotate = autoRotate;
    if(!autoRotate) {
      setRotatePower(0.0);
    }
    SmartDashboard.putBoolean("Auto-Rotate", autoRotate);
  }

  public void setRotatePower(double power) {
    m_steeringMotorFL.set(power);
    m_steeringMotorFR.set(power);
    m_steeringMotorBL.set(power);
    m_steeringMotorBR.set(power);

    SmartDashboard.putNumber("FL Pwr", power);
    SmartDashboard.putNumber("FR Pwr", power);
    SmartDashboard.putNumber("BL Pwr", power);
    SmartDashboard.putNumber("BR Pwr", power);
  }

  public void resetGyro() {
    m_gyro.reset();
  }
}
