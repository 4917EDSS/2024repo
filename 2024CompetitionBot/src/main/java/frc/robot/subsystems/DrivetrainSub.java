// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.Constants;

public class DrivetrainSub extends SubsystemBase {
  public static final double kMaxSpeed = 3.0; // 3 meters per second

  private final Translation2d m_frontLeftLocation = new Translation2d(-0.381, -0.318); // I have no idea why these are 0.381
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.318);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.318);
  private final Translation2d m_backRightLocation = new Translation2d(0.381, 0.318);

  private final SwerveModule m_frontLeft =
      new SwerveModule(Constants.CanIds.kDriveMotorFL, Constants.CanIds.kSteeringMotorFL, Constants.CanIds.kEncoderFL);
  private final SwerveModule m_frontRight =
      new SwerveModule(Constants.CanIds.kDriveMotorFL, Constants.CanIds.kSteeringMotorFL, Constants.CanIds.kEncoderFL);
  private final SwerveModule m_backLeft =
      new SwerveModule(Constants.CanIds.kDriveMotorFL, Constants.CanIds.kSteeringMotorFL, Constants.CanIds.kEncoderFL);
  private final SwerveModule m_backRight =
      new SwerveModule(Constants.CanIds.kDriveMotorFL, Constants.CanIds.kSteeringMotorFL, Constants.CanIds.kEncoderFL);

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d(), new SwerveModulePosition[] {
          m_frontLeft.getPosition(), m_frontRight.getPosition(), m_backLeft.getPosition(), m_backRight.getPosition()});

  /** Creates a new DrivetrainSub. */
  public DrivetrainSub() {
    m_gyro.reset();
  }

  public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldOriented, double periodSeconds) {

    var swerveStates = m_kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(
        fieldOriented ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rotation),
        periodSeconds)); // Get swerve states

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveStates, kMaxSpeed); // Keep motors below max speed (Might not need to be used)
    m_frontLeft.setState(swerveStates[0]);
    m_frontRight.setState(swerveStates[1]);
    m_backLeft.setState(swerveStates[2]);
    m_backRight.setState(swerveStates[3]);
  }

  public void updateOdemetry() {
    m_odometry.update(m_gyro.getRotation2d(), new SwerveModulePosition[] {
        m_frontLeft.getPosition(), m_frontRight.getPosition(), m_backLeft.getPosition(), m_backRight.getPosition()});
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdemetry();
    double xPos = m_odometry.getPoseMeters().getX();
    double yPos = m_odometry.getPoseMeters().getY();

    SmartDashboard.putNumber("XPOS", xPos);
    SmartDashboard.putNumber("YPOS", yPos);
    SmartDashboard.putNumber("GYRO", m_gyro.getAngle());
  }


}
