// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
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
  // Speed multipliers
  public static final double kDriveSpeed = 100.0;
  public static final double kTurnSpeed = 50.0;

  public static final double kMaxSpeed = 10000.0;// meters per second

  // Locations of Swerve Modules relative to the center of the robot
  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.318); // I have no idea why these are 0.381
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.318);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.318);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.318);

  private Translation2d relativePos = new Translation2d(0.0, 0.0);
  private Translation2d odometryPos = new Translation2d(0.0, 0.0);
  private PIDController m_odometryPIDx = new PIDController(0.2, 0.0, 0.0); // X and Y PIDs
  private PIDController m_odometryPIDy = new PIDController(0.2, 0.0, 0.0);
  private PIDController m_odometryPIDr = new PIDController(0.5, 0.0, 0.0); // Rotational PID

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
    m_odometryPIDx.setTolerance(0.1); // In meters
    m_odometryPIDy.setTolerance(0.1); // In meters
  }

  public void resetGyro() {
    m_gyro.reset();
  }

  public float getRoll() {
    return m_gyro.getRoll();
  }

  public void drive(double xSpeed, double ySpeed, double rotationSpeed, double periodSeconds) { // Period should be time period between whenever this is called
    xSpeed *= kDriveSpeed;
    ySpeed *= kDriveSpeed;
    rotationSpeed *= kTurnSpeed;
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

  public void translateOdometry(Translation2d pos) { // Set target position TODO: Figure out why this doesn't set the position
    odometryPos = new Translation2d(pos.getX(), pos.getY());
  }

  public boolean updateOdometryTransform() { // Returns true when at position
    double xPower = m_odometryPIDx.calculate(getPos().getX(), odometryPos.getX());//MathUtil.clamp(m_odometryPIDx.calculate(getPos().getX(), odometryPos.getX()), -100.5, 100.5);
    double yPower = m_odometryPIDy.calculate(getPos().getY(), odometryPos.getY());//MathUtil.clamp(m_odometryPIDy.calculate(getPos().getY(), odometryPos.getY()), -100.5, 100.5);
    if(!m_odometryPIDx.atSetpoint() && !m_odometryPIDy.atSetpoint()) {
      drive(xPower, yPower, 0.0, 0.02);
      return false;
    }
    return true;
  }

  public void updateOdometry() {
    m_odometry.update(m_gyro.getRotation2d(), new SwerveModulePosition[] {
        m_frontLeft.getPosition(), m_frontRight.getPosition(), m_backLeft.getPosition(), m_backRight.getPosition()});
  }

  public void resetOdometry() {
    Pose2d zero = new Pose2d(new Translation2d(0.0, 0.0), m_gyro.getRotation2d());
    m_odometry.resetPosition(m_gyro.getRotation2d(), new SwerveModulePosition[] {
        m_frontLeft.getPosition(), m_frontRight.getPosition(), m_backLeft.getPosition(), m_backRight.getPosition()},
        zero);
  }

  public void resetRelativePos() {
    relativePos = getPos();
  }

  public double getDistanceRelativeOrigin() { // Distance from a relative point set by the reset
    return getPos().getDistance(odometryPos);
  }

  public Translation2d getPos() { // Get position from odometry
    return m_odometry.getPoseMeters().getTranslation();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry(); // TODO: Move this to an autonomous periodic so it isn't running during teleop
    double xPos = m_odometry.getPoseMeters().getX();
    double yPos = m_odometry.getPoseMeters().getY();
    SmartDashboard.putNumber("XPOS", xPos);
    SmartDashboard.putNumber("YPOS", yPos);
    SmartDashboard.putNumber("GYRO", m_gyro.getAngle() % 360);

    SmartDashboard.putNumber("FL encoder", m_frontLeft.getTurningRotation());
    SmartDashboard.putNumber("FR encoder", m_frontRight.getTurningRotation());
    SmartDashboard.putNumber("BL encoder", m_backLeft.getTurningRotation());
    SmartDashboard.putNumber("BR encoder", m_backRight.getTurningRotation());

    SmartDashboard.putNumber("Distance from point(m)", getDistanceRelativeOrigin());
  }
}
