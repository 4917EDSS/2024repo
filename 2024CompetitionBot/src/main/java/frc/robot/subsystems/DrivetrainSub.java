// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
  public static final double kMaxDriveSpeed = 1000.0; // In m/s
  public static final double kMaxTurnSpeed = 30.0; // was 50

  //public static final double kMaxSpeed = 10000.0;// meters per second

  // Locations of Swerve Modules relative to the center of the robot
  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.318); // I have no idea why these are 0.381
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.318);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.318);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.318);

  // PID value setting
  private double kPIDp = 0.4;
  private double kPIDd = 0.0;
  private double kThreshold = 0.05;

  private double kRotPIDp = 0.1;
  private double kRotPIDd = 0.0;
  private double kTurnThreshold = 1.0;

  private Pose2d targetPos = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
  private PIDController m_odometryPIDx = new PIDController(kPIDp, 0.0, kPIDd); // X and Y PIDs
  private PIDController m_odometryPIDy = new PIDController(kPIDp, 0.0, kPIDd);
  private PIDController m_odometryPIDr = new PIDController(kRotPIDp, 0.0, kRotPIDd); // Rotational PID

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
    m_gyro.setAngleAdjustment(90);
    m_odometryPIDx.setTolerance(kThreshold); // In meters
    m_odometryPIDy.setTolerance(kThreshold); // In meters

    m_odometryPIDy.setTolerance(1.0, 3.0); // In degrees

    m_odometryPIDr.enableContinuousInput(-180.0, 180.0);
  }

  public Rotation2d getRotation() {
    return m_gyro.getRotation2d();
  }

  public double getRotationDegrees() {
    return MathUtil.inputModulus(getRotation().getDegrees(), -180.0, 180.0);
  }

  public void resetGyro() {
    m_gyro.reset();
  }

  public float getRoll() {
    // proto type bot roll is navx pitch
    return m_gyro.getPitch();
  }

  public void drive(double xSpeed, double ySpeed, double rotationSpeed, double periodSeconds) { // Period should be time period between whenever this is called
    xSpeed *= kMaxDriveSpeed;
    ySpeed *= kMaxDriveSpeed;
    rotationSpeed *= -kMaxTurnSpeed; // This is negative so it's CCW Positive
    //var swerveStates = m_kinematics.toSwerveModuleStates(speedS); // Get swerve states
    // X and Y are swapped because it drives sideways for some reason
    var swerveStates = m_kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, m_gyro.getRotation2d()), periodSeconds)); // Get swerve states

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveStates, kMaxDriveSpeed); // Keep motors below max speed (Might not need to be used)

    // Drive motors
    m_frontLeft.setState(swerveStates[0]);
    m_frontRight.setState(swerveStates[1]);
    m_backLeft.setState(swerveStates[2]);
    m_backRight.setState(swerveStates[3]);
  }

  public void translateOdometry(Translation2d pos) { // Set target position 
    targetPos = new Pose2d(pos.getX(), pos.getY(), m_gyro.getRotation2d());
  }

  public void translateOdometry(Pose2d pos) { // Set target position and rotation (degrees)
    targetPos = new Pose2d(pos.getTranslation(), pos.getRotation());
  }

  public boolean updateOdometryTransform() { // Returns true when at position
    //double rotationDifference = m_odometryPIDr.getPositionError(); // In degrees
    double rotationClamp = 1.0;
    double xPower = MathUtil.clamp(m_odometryPIDx.calculate(getPos().getX(), targetPos.getX()), -0.5, 0.5);
    double yPower = MathUtil.clamp(m_odometryPIDy.calculate(getPos().getY(), targetPos.getY()), -0.5, 0.5);
    double rotationPower =
        MathUtil.clamp(
            m_odometryPIDr.calculate(getRotationDegrees(),
                MathUtil.inputModulus(targetPos.getRotation().getDegrees(), -180.0, 180.0)),
            -rotationClamp, rotationClamp);
    drive(xPower, yPower, rotationPower, 0.02);
    return ((m_odometryPIDx.atSetpoint() && m_odometryPIDy.atSetpoint()) && m_odometryPIDr.atSetpoint());
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
    SmartDashboard.putNumber("TARGET XPOS", targetPos.getX());
    SmartDashboard.putNumber("TARGET YPOS", targetPos.getY());
    SmartDashboard.putNumber("TARGET ROT",
        MathUtil.inputModulus(targetPos.getRotation().getDegrees(), -180.0, 180.0));

    SmartDashboard.putNumber("GYRO", getRotationDegrees());
    SmartDashboard.putNumber("Yaw", m_gyro.getAngle() % 360);
    SmartDashboard.putNumber("Roll", m_gyro.getRoll());
    SmartDashboard.putNumber("Pitch", m_gyro.getPitch());

    SmartDashboard.putNumber("FL encoder", m_frontLeft.getTurningRotation());
    SmartDashboard.putNumber("FR encoder", m_frontRight.getTurningRotation());
    SmartDashboard.putNumber("BL encoder", m_backLeft.getTurningRotation());
    SmartDashboard.putNumber("BR encoder", m_backRight.getTurningRotation());

    kPIDp = SmartDashboard.getNumber("Path kP", kPIDp);
    kPIDd = SmartDashboard.getNumber("Path kD", kPIDd);
    kThreshold = SmartDashboard.getNumber("Path Threshold", kThreshold);
    kRotPIDp = SmartDashboard.getNumber("Rot kP", kRotPIDp);
    kRotPIDd = SmartDashboard.getNumber("Rot kD", kRotPIDd);
    kTurnThreshold = SmartDashboard.getNumber("Rot Threshold", kTurnThreshold);
    SmartDashboard.putNumber("Rot kP", kRotPIDp);
    SmartDashboard.putNumber("Rot kD", kRotPIDd);
    SmartDashboard.putNumber("Rot Threshold", kTurnThreshold);
    SmartDashboard.putNumber("Path kP", kPIDp);
    SmartDashboard.putNumber("Path kD", kPIDd);
    SmartDashboard.putNumber("Path Threshold", kThreshold);

    // Setting PID constants

    m_odometryPIDx.setP(kPIDp);
    m_odometryPIDx.setD(kPIDd);
    m_odometryPIDx.setTolerance(kThreshold);

    m_odometryPIDy.setP(kPIDp);
    m_odometryPIDy.setD(kPIDd);
    m_odometryPIDy.setTolerance(kThreshold);

    m_odometryPIDr.setP(kRotPIDp);
    m_odometryPIDr.setD(kRotPIDd);
    m_odometryPIDr.setTolerance(kTurnThreshold);
  }
}
