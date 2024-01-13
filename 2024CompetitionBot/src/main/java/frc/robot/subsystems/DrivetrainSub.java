// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSub extends SubsystemBase {
  private static final boolean kGyroReversed = false;
  private static final double kMaxSpeedMetersPerSecond = 3; // TO-do: Tune if needed
  private static final double kTrackWidth = 0.678; // meters
  // Distance between centers of right and left wheels on robot
  private static final double kWheelBase = 0.633; // meters
  // Distance between front and back wheels on robot
  private static final SwerveDriveKinematics kDriveKinematics =
      new SwerveDriveKinematics(
          new Translation2d(kWheelBase / 2, kTrackWidth / 2),
          new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
          new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
          new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

  // Create the 4 swerve modules
  private final SwerveModule m_frontLeft = new SwerveModule(
      Constants.CanIds.kDriveMotorFL,
      Constants.CanIds.kTurningMotorFL,
      Constants.CanIds.kEncoderFL,
      false,
      Constants.SwerveModuleConstants.kAbsoluteEncoderOffsetFL);
  private final SwerveModule m_frontRight = new SwerveModule(
      Constants.CanIds.kDriveMotorFR,
      Constants.CanIds.kTurningMotorFR,
      Constants.CanIds.kEncoderFR,
      false,
      Constants.SwerveModuleConstants.kAbsoluteEncoderOffsetFR);
  private final SwerveModule m_backLeft = new SwerveModule(
      Constants.CanIds.kDriveMotorBL,
      Constants.CanIds.kTurningMotorBL,
      Constants.CanIds.kEncoderBL,
      false,
      Constants.SwerveModuleConstants.kAbsoluteEncoderOffsetBL);
  private final SwerveModule m_backRight = new SwerveModule(
      Constants.CanIds.kDriveMotorBR,
      Constants.CanIds.kTurningMotorBR,
      Constants.CanIds.kEncoderBR,
      false,
      Constants.SwerveModuleConstants.kAbsoluteEncoderOffsetBR);

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          kDriveKinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
              m_frontLeft.getPosition(),
              m_frontRight.getPosition(),
              m_backLeft.getPosition(),
              m_backRight.getPosition()
          });


  /** Creates a new DrivetrainSub. */
  public DrivetrainSub() {
    // SmartDashboard.putNumber("dP", 0.1);
    // SmartDashboard.putNumber("dI", 0);
    // SmartDashboard.putNumber("dD", 0);
    SmartDashboard.putNumber("tP", 0.5);
    SmartDashboard.putNumber("tI", 0);
    SmartDashboard.putNumber("tD", 0);
  }

  private int delayCount = 10;

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        });

    // This method will be called once per scheduler run
    // SwerveModulePosition smp = m_frontLeft.getPosition();
    // SwerveModuleState sms = m_frontLeft.getState();

    // SmartDashboard.putNumber("FL Dist m", m_frontLeft.getDrivePositionM());
    // SmartDashboard.putNumber("FL Vel mps", m_frontLeft.getDriveVelocityMPerS());
    // SmartDashboard.putNumber("FL Turn pos", m_frontLeft.getTurningPosition());
    // SmartDashboard.putNumber("FL Turn vel", m_frontLeft.getTurningVelocity());
    SmartDashboard.putNumber("Heading deg", getHeading());

    SmartDashboard.putNumber("FL Abs", m_frontLeft.getTurningPositionAbsoluteRad());
    SmartDashboard.putNumber("FR Abs", m_frontRight.getTurningPositionAbsoluteRad());
    SmartDashboard.putNumber("BL Abs", m_backLeft.getTurningPositionAbsoluteRad());
    SmartDashboard.putNumber("BR Abs", m_backRight.getTurningPositionAbsoluteRad());

    if(--delayCount == 0) {
      delayCount = 10;
      // double dp = SmartDashboard.getNumber("dP", 0.1);
      // double di = SmartDashboard.getNumber("dI", 0);
      // double dd = SmartDashboard.getNumber("dD", 0);
      double tp = SmartDashboard.getNumber("tP", 0.5);
      double ti = SmartDashboard.getNumber("tI", 0);
      double td = SmartDashboard.getNumber("tD", 0);

      // m_frontLeft.setPID(true, dp, di, dd);
      m_frontLeft.setPID(false, tp, ti, td);
      // m_frontRight.setPID(true, dp, di, dd);
      m_frontRight.setPID(false, tp, ti, td);
      // m_backLeft.setPID(true, dp, di, dd);
      m_backLeft.setPID(false, tp, ti, td);
      // m_backRight.setPID(true, dp, di, dd);
      m_backRight.setPID(false, tp, ti, td);
    }
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeedMetersPerSecond); // Used to be called normalizeWheelSpeeds
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_backLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_backRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getRotation2d().getDegrees(), 360);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (kGyroReversed ? -1.0 : 1.0);
  }

  public void stopModules() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }


}
