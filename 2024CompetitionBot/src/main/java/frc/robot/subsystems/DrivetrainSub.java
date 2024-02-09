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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.logging.Logger;
import com.ctre.phoenix6.Orchestra;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.Constants;
// import edu.wpi.first.wpilibj.AnalogInput;

public class DrivetrainSub extends SubsystemBase {
  private static Logger m_logger = Logger.getLogger(DrivetrainSub.class.getName());


  //Analog sensors
  //private final AnalogInput m_frontDistanceSensor = new AnalogInput(Constants.AnalogInIds.kFrontDistanceSenor);

  private final Orchestra orca1 = new Orchestra();
  private final Orchestra orca2 = new Orchestra();
  private final Orchestra orca3 = new Orchestra();
  private final Orchestra orca4 = new Orchestra();
  private double m_orientationOffsetDegrees = 0;

  // Speed multipliers
  public static final double kMaxDriveSpeed = 100.0; // In m/s
  public static final double kMaxTurnSpeed = 30.0; // was 50

  //public static final double kMaxSpeed = 10000.0;// meters per second

  // Locations of Swerve Modules relative to the center of the robot
  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.318); // I have no idea why these are 0.381
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.318);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.318);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.318);
  private final ShuffleboardTab m_shuffleboardTab = Shuffleboard.getTab("DriveTrain");
  private final GenericEntry m_sbYPOS, m_sbXPOS, m_sbTargetXPOS, m_sbTargetYPOS, m_sbTargetROT, m_sbGYRO, m_sbYaw,
      m_sbRoll, m_sbPitch, m_sbFLEncoder,
      m_sbFREncoder, m_sbBLEncoder, m_sbBREncoder, m_sbRotKP, m_sbRotKD, m_sbRotThreshold, m_sbPathKP, m_sbPathKD,
      m_sbPathThreshold;


  // PID value setting
  private double kPIDp = 0.4;
  private double kPIDd = 0.0;
  private double kThreshold = 0.05;

  private double kRotPIDp = 0.1;
  private double kRotPIDd = 0.0;
  private double kTurnThreshold = 1.0;

  private Pose2d targetPos = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
  private Rotation2d previousRotation;
  private PIDController m_odometryPIDx = new PIDController(kPIDp, 0.0, kPIDd); // X and Y PIDs
  private PIDController m_odometryPIDy = new PIDController(kPIDp, 0.0, kPIDd);
  private PIDController m_odometryPIDr = new PIDController(kRotPIDp, 0.0, kRotPIDd); // Rotational PID
  private PIDController m_drivePIDr = new PIDController(0.05, 0.0, 0.0);

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

    m_odometryPIDr.setTolerance(1.0, 3.0); // In degrees
    m_odometryPIDr.enableContinuousInput(-180.0, 180.0);

    m_drivePIDr.setTolerance(1.0);
    m_drivePIDr.enableContinuousInput(-180.0, 180.0);

    previousRotation = getRotation();

    m_sbXPOS = m_shuffleboardTab.add("XPOS", 0.0).getEntry();
    m_sbYPOS = m_shuffleboardTab.add("YPOS", 0.0).getEntry();
    m_sbTargetXPOS = m_shuffleboardTab.add("TARGET XPOS", 0.0).getEntry();
    m_sbTargetYPOS = m_shuffleboardTab.add("TARGET YPOS", 0.0).getEntry();
    m_sbTargetROT = m_shuffleboardTab.add("TARGET ROT", 0.0).getEntry();

    m_sbGYRO = m_shuffleboardTab.add("GYRO", 0.0).getEntry();
    m_sbYaw = m_shuffleboardTab.add("Yaw", 0.0).getEntry();
    m_sbRoll = m_shuffleboardTab.add("Roll", 0.0).getEntry();
    m_sbPitch = m_shuffleboardTab.add("Pitch", 0.0).getEntry();

    m_sbFLEncoder = m_shuffleboardTab.add("FL encoder", 0.0).getEntry();
    m_sbFREncoder = m_shuffleboardTab.add("FR encoder", 0.0).getEntry();
    m_sbBLEncoder = m_shuffleboardTab.add("BL encoder", 0.0).getEntry();
    m_sbBREncoder = m_shuffleboardTab.add("BR encoder", 0.0).getEntry();

    m_sbRotKP = m_shuffleboardTab.add("Rot kP", 0.0).getEntry();
    m_sbRotKD = m_shuffleboardTab.add("Rot kD", 0.0).getEntry();
    m_sbRotThreshold = m_shuffleboardTab.add("Rot Threshold", 0.0).getEntry();
    m_sbPathKP = m_shuffleboardTab.add("Path kP", 0.0).getEntry();
    m_sbPathKD = m_shuffleboardTab.add("Path kD", 0.0).getEntry();
    m_sbPathThreshold = m_shuffleboardTab.add("Path Threshold", 0.0).getEntry();

  }

  public Rotation2d getRotation() {
    return m_gyro.getRotation2d();
  }

  public double getRotationDegrees() {
    return MathUtil.inputModulus(getRotation().getDegrees() + m_orientationOffsetDegrees, -180.0, 180.0);
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

  public void driveHoldAngle(double xSpeed, double ySpeed, double rotationSpeed,
      double periodSeconds) { // For driving with a joystick
    double drivePower = Math.sqrt(xSpeed * xSpeed + ySpeed * ySpeed);
    xSpeed *= kMaxDriveSpeed;
    ySpeed *= kMaxDriveSpeed;

    double newRotationSpeed = rotationSpeed;

    //var swerveStates = m_kinematics.toSwerveModuleStates(speedS); // Get swerve states
    // X and Y are swapped because it drives sideways for some reason
    double prevDegrees = MathUtil.inputModulus(previousRotation.getDegrees(), -180.0, 180.0);
    if(rotationSpeed == 0.0) {
      if((xSpeed != 0.0 || ySpeed != 0.0)) {
        newRotationSpeed = MathUtil.clamp(
            m_drivePIDr.calculate(getRotationDegrees(),
                prevDegrees),
            -1.0, 1.0);
      }
    } else {
      previousRotation = getRotation();
    }
    newRotationSpeed *= -kMaxTurnSpeed; // This is negative so it's CCW Positive
    SmartDashboard.putNumber("Rotation Speed", newRotationSpeed); // TODO: Remove this
    var swerveStates = m_kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, newRotationSpeed, m_gyro.getRotation2d()),
        periodSeconds)); // Get swerve states

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
    /**
     * SmartDashboard.putNumber("XPOS", xPos);
     * SmartDashboard.putNumber("YPOS", yPos);
     * SmartDashboard.putNumber("TARGET XPOS", targetPos.getX());
     * SmartDashboard.putNumber("TARGET YPOS", targetPos.getY());
     * SmartDashboard.putNumber("TARGET ROT",
     * MathUtil.inputModulus(targetPos.getRotation().getDegrees(), -180.0, 180.0));
     * 
     * SmartDashboard.putNumber("GYRO", getRotationDegrees());
     * SmartDashboard.putNumber("Yaw", m_gyro.getAngle() % 360);
     * SmartDashboard.putNumber("Roll", m_gyro.getRoll());
     * SmartDashboard.putNumber("Pitch", m_gyro.getPitch());
     * 
     * SmartDashboard.putNumber("FL encoder", m_frontLeft.getTurningRotation());
     * SmartDashboard.putNumber("FR encoder", m_frontRight.getTurningRotation());
     * SmartDashboard.putNumber("BL encoder", m_backLeft.getTurningRotation());
     * SmartDashboard.putNumber("BR encoder", m_backRight.getTurningRotation());
     * 
     * kPIDp = SmartDashboard.getNumber("Path kP", kPIDp);
     * kPIDd = SmartDashboard.getNumber("Path kD", kPIDd);
     * kThreshold = SmartDashboard.getNumber("Path Threshold", kThreshold);
     * kRotPIDp = SmartDashboard.getNumber("Rot kP", kRotPIDp);
     * kRotPIDd = SmartDashboard.getNumber("Rot kD", kRotPIDd);
     * kTurnThreshold = SmartDashboard.getNumber("Rot Threshold", kTurnThreshold);
     * SmartDashboard.putNumber("Rot kP", kRotPIDp);
     * SmartDashboard.putNumber("Rot kD", kRotPIDd);
     * SmartDashboard.putNumber("Rot Threshold", kTurnThreshold);
     * SmartDashboard.putNumber("Path kP", kPIDp);
     * SmartDashboard.putNumber("Path kD", kPIDd);
     * SmartDashboard.putNumber("Path Threshold", kThreshold);
     * //SmartDashboard.putNumber("Sensor Distance", getFrontDistance());
     **/

    SmartDashboard.putNumber("Held Angle", MathUtil.inputModulus(previousRotation.getDegrees(), -180.0, 180.0));

    double rot = MathUtil.inputModulus(targetPos.getRotation().getDegrees(), -180.0, 180.0);
    m_sbXPOS.setDouble(xPos);
    m_sbYPOS.setDouble(yPos);
    m_sbTargetXPOS.setDouble(targetPos.getX());
    m_sbTargetYPOS.setDouble(targetPos.getY());
    m_sbTargetROT.setDouble(rot);

    m_sbGYRO.setDouble(getRotationDegrees());
    m_sbYaw.setDouble(m_gyro.getAngle() % 360);
    m_sbRoll.setDouble(m_gyro.getRoll());
    m_sbPitch.setDouble(m_gyro.getPitch());

    m_sbFLEncoder.setDouble(m_frontLeft.getTurningRotation());
    m_sbFREncoder.setDouble(m_frontRight.getTurningRotation());
    m_sbBLEncoder.setDouble(m_backLeft.getTurningRotation());
    m_sbBREncoder.setDouble(m_backRight.getTurningRotation());

    kPIDp = SmartDashboard.getNumber("Path kP", kPIDp);
    kPIDd = SmartDashboard.getNumber("Path kD", kPIDd);
    kThreshold = SmartDashboard.getNumber("Path Threshold", kThreshold);
    kRotPIDp = SmartDashboard.getNumber("Rot kP", kRotPIDp);
    kRotPIDd = SmartDashboard.getNumber("Rot kD", kRotPIDd);
    kTurnThreshold = SmartDashboard.getNumber("Rot Threshold", kTurnThreshold);
    m_sbRotKP.setDouble(kRotPIDp);
    m_sbRotKD.setDouble(kRotPIDd);
    m_sbRotThreshold.setDouble(kTurnThreshold);
    m_sbPathKP.setDouble(kPIDp);
    m_sbPathKD.setDouble(kPIDd);
    m_sbPathThreshold.setDouble(kThreshold);

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

  public void fun() {
    if(!orca1.isPlaying() || !orca2.isPlaying()) { // TalonFX developers had a skill issue and forgot to implement multiple tracks
      orca1.stop();
      orca2.stop();
      orca3.stop();
      orca4.stop();
      orca1.clearInstruments();
      orca2.clearInstruments();
      orca3.clearInstruments();
      orca1.addInstrument(m_frontLeft.m_steeringMotor);
      orca2.addInstrument(m_frontRight.m_steeringMotor);
      orca3.addInstrument(m_backLeft.m_steeringMotor);
      orca4.addInstrument(m_backRight.m_steeringMotor);
      orca1.loadMusic("dat3n_downA.chrp");
      orca2.loadMusic("dat3n_downB.chrp");
      orca3.loadMusic("dat3n_downC.chrp");
      orca4.loadMusic("dat3n_downD.chrp");
      orca1.play();
      orca2.play();
      orca3.play();
      orca4.play();
    } else {
      orca1.stop();
      orca2.stop();
      orca3.stop();
      orca4.stop();
    }
  }


  public void init() {
    resetGyro();
    resetOdometry();

  }

  // public double getFrontDistance() {
  //    //TODO: Convert voltage to distance in meters
  //   return m_frontDistanceSensor.getAverageVoltage();
  // }
}
