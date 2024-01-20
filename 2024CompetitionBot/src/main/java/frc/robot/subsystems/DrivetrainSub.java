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
  public static final double kMaxSpeed = 10.0; // 3 meters per second

  // Locations of Swerve Modules relative to the center of the robot
  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.318); // I have no idea why these are 0.381
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.318);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.318);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.318);

  // Swerve Modules that control the motors
  private final SwerveModule m_frontLeft =
      new SwerveModule(Constants.CanIds.kDriveMotorFL, Constants.CanIds.kSteeringMotorFL, Constants.CanIds.kEncoderFL,
          Constants.DriveConstants.kAbsoluteEncoderOffsetFL);
  private final SwerveModule m_frontRight =
      new SwerveModule(Constants.CanIds.kDriveMotorFR, Constants.CanIds.kSteeringMotorFR, Constants.CanIds.kEncoderFR,
          Constants.DriveConstants.kAbsoluteEncoderOffsetFR);
  private final SwerveModule m_backLeft =
      new SwerveModule(Constants.CanIds.kDriveMotorBL, Constants.CanIds.kSteeringMotorBL, Constants.CanIds.kEncoderBL,
          Constants.DriveConstants.kAbsoluteEncoderOffsetBL);
  private final SwerveModule m_backRight =
      new SwerveModule(Constants.CanIds.kDriveMotorBR, Constants.CanIds.kSteeringMotorBR, Constants.CanIds.kEncoderBR,
          Constants.DriveConstants.kAbsoluteEncoderOffsetBR);

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // Kinematics controls movement, Odemetry tracks position
  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d(), new SwerveModulePosition[] {
          m_frontLeft.getPosition(), m_frontRight.getPosition(), m_backLeft.getPosition(), m_backRight.getPosition()});


  /** Creates a new DrivetrainSub. */
  public DrivetrainSub() {
    m_gyro.reset();
  }

  public void resetGyro() {
    m_gyro.reset();
  }


  public void drive(double xSpeed, double ySpeed, double rotationSpeed, double periodSeconds) { // Period should be time period between whenever this is called
    if(Math.abs(xSpeed) < 0.1)
      xSpeed = 0.0;
    if(Math.abs(ySpeed) < 0.1)
      ySpeed = 0.0;
    if(Math.abs(rotationSpeed) < 0.1)
      rotationSpeed = 0.0;
    xSpeed *= 10.0;
    ySpeed *= 10.0;
    rotationSpeed *= 7.0;
    ChassisSpeeds speedS =
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, m_gyro.getRotation2d());

    //var swerveStates = m_kinematics.toSwerveModuleStates(speedS); // Get swerve states

    var swerveStates = m_kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, m_gyro.getRotation2d()), periodSeconds)); // Get swerve states

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveStates, kMaxSpeed); // Keep motors below max speed (Might not need to be used)

    // Drive motors
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
    updateOdemetry(); // TODO: Move this to an autonomous periodic so it isn't running during teleop
    double xPos = m_odometry.getPoseMeters().getX();
    double yPos = m_odometry.getPoseMeters().getY();
    SmartDashboard.putNumber("XPOS", xPos);
    SmartDashboard.putNumber("YPOS", yPos);
    SmartDashboard.putNumber("GYRO", m_gyro.getAngle() % 360);

    SmartDashboard.putNumber("FL encoder", m_frontLeft.getTurningRotation());
    SmartDashboard.putNumber("FR encoder", m_frontRight.getTurningRotation());
    SmartDashboard.putNumber("BL encoder", m_backLeft.getTurningRotation());
    SmartDashboard.putNumber("BR encoder", m_backRight.getTurningRotation());
  }


}
