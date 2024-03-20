// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;
import com.ctre.phoenix6.Orchestra;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
// import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.RobotContainer;


public class DrivetrainSub extends SubsystemBase {
  private static Logger m_logger = Logger.getLogger(DrivetrainSub.class.getName());

  private final Orchestra orca1 = new Orchestra();
  private final Orchestra orca2 = new Orchestra();
  private final Orchestra orca3 = new Orchestra();
  private final Orchestra orca4 = new Orchestra();

  // Speed multipliers
  public static final double kMaxDriveSpeed = 2.0; // In m/s
  public static final double kMaxTurnSpeed = 9; // was 50

  // Locations of Swerve Modules relative to the center of the robot
  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.318); // I have no idea why these are 0.381
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.318);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.318);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.318);
  private final ShuffleboardTab m_shuffleboardTab = Shuffleboard.getTab("DriveTrain");
  private final GenericEntry m_sbYPOS, m_sbXPOS, m_sbTargetXPOS, m_sbTargetYPOS, m_sbTargetROT, m_sbGYRO, m_sbYaw,
      m_sbRoll, m_sbPitch, m_sbFLEncoder,
      m_sbFREncoder, m_sbBLEncoder, m_sbBREncoder, m_sbRotKP, m_sbRotKD, m_sbRotThreshold, m_sbPathKP, m_sbPathKD,
      m_sbPathThreshold, m_sbSerialNumber, m_sbRobotName;

  // PID value setting
  private double kPIDp = 0.4;
  private double kPIDd = 0.0;
  private double kThreshold = 0.05;

  private double kRotPIDp = 0.005;
  private double kRotPIDd = 0.0;
  private double kTurnThreshold = 1.0;

  private Pose2d targetPos = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
  private PIDController m_odometryPIDx = new PIDController(kPIDp, 0.0, kPIDd); // X and Y PIDs
  private PIDController m_odometryPIDy = new PIDController(kPIDp, 0.0, kPIDd);
  private PIDController m_odometryPIDr = new PIDController(kRotPIDp, 0.0, kRotPIDd); // Rotational PID
  private PIDController m_drivePIDr = new PIDController(0.05, 0.0, 0.0);

  // Swerve Modules that control the motors
  private final SwerveModule m_frontLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_backLeft;
  private final SwerveModule m_backRight;

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // Kinematics controls movement, Odemetry tracks position
  private final SwerveDriveKinematics m_kinematics;
  private final SwerveDriveOdometry m_odometry;


  /** Creates a new DrivetrainSub. */
  public DrivetrainSub() {
    String robotName;
    double AbsoluteEncoderOffsetFL;
    double AbsoluteEncoderOffsetFR;
    double AbsoluteEncoderOffsetBL;
    double AbsoluteEncoderOffsetBR;
    if(Constants.Drivetrain.serialNumber.equals(Constants.RobotSpecific.PracticeSerialNumber)) {
      robotName = "Practice";
      AbsoluteEncoderOffsetFL = Constants.RobotSpecific.Practice.kAbsoluteEncoderOffsetFL;
      AbsoluteEncoderOffsetFR = Constants.RobotSpecific.Practice.kAbsoluteEncoderOffsetFR;
      AbsoluteEncoderOffsetBL = Constants.RobotSpecific.Practice.kAbsoluteEncoderOffsetBL;
      AbsoluteEncoderOffsetBR = Constants.RobotSpecific.Practice.kAbsoluteEncoderOffsetBR;
    } else if(Constants.Drivetrain.serialNumber.equals(Constants.RobotSpecific.CompetitionSerialNumber)) {
      robotName = "Competition";
      AbsoluteEncoderOffsetFL = Constants.RobotSpecific.Competition.kAbsoluteEncoderOffsetFL;
      AbsoluteEncoderOffsetFR = Constants.RobotSpecific.Competition.kAbsoluteEncoderOffsetFR;
      AbsoluteEncoderOffsetBL = Constants.RobotSpecific.Competition.kAbsoluteEncoderOffsetBL;
      AbsoluteEncoderOffsetBR = Constants.RobotSpecific.Competition.kAbsoluteEncoderOffsetBR;
    } else {
      robotName = "Unknown";
      AbsoluteEncoderOffsetFL = Constants.RobotSpecific.Unknown.kAbsoluteEncoderOffsetFL;
      AbsoluteEncoderOffsetFR = Constants.RobotSpecific.Unknown.kAbsoluteEncoderOffsetFR;
      AbsoluteEncoderOffsetBL = Constants.RobotSpecific.Unknown.kAbsoluteEncoderOffsetBL;
      AbsoluteEncoderOffsetBR = Constants.RobotSpecific.Unknown.kAbsoluteEncoderOffsetBR;
    }

    m_frontLeft =
        new SwerveModule(Constants.CanIds.kDriveMotorFL, Constants.CanIds.kSteeringMotorFL, Constants.CanIds.kEncoderFL,
            AbsoluteEncoderOffsetFL);
    m_frontRight =
        new SwerveModule(Constants.CanIds.kDriveMotorFR, Constants.CanIds.kSteeringMotorFR, Constants.CanIds.kEncoderFR,
            AbsoluteEncoderOffsetFR);
    m_backLeft =
        new SwerveModule(Constants.CanIds.kDriveMotorBL, Constants.CanIds.kSteeringMotorBL, Constants.CanIds.kEncoderBL,
            AbsoluteEncoderOffsetBL);
    m_backRight =
        new SwerveModule(Constants.CanIds.kDriveMotorBR, Constants.CanIds.kSteeringMotorBR, Constants.CanIds.kEncoderBR,
            AbsoluteEncoderOffsetBR);
    m_kinematics =
        new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
    m_odometry =
        new SwerveDriveOdometry(m_kinematics, getYawRotation2d(), new SwerveModulePosition[] {
            m_frontLeft.getPosition(), m_frontRight.getPosition(), m_backLeft.getPosition(),
            m_backRight.getPosition()});

    resetGyroYaw(0);
    m_odometryPIDx.setTolerance(kThreshold); // In meters
    m_odometryPIDy.setTolerance(kThreshold); // In meters

    m_odometryPIDr.setTolerance(1.0, 3.0); // In degrees
    m_odometryPIDr.enableContinuousInput(-180.0, 180.0);

    m_drivePIDr.setTolerance(1.0);
    m_drivePIDr.enableContinuousInput(-180.0, 180.0);

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
    m_sbSerialNumber = m_shuffleboardTab.add("Serial Number", "None").getEntry();
    m_sbRobotName = m_shuffleboardTab.add("Robot Name", "None").getEntry();

    m_sbRobotName.setString(robotName);

    AutoBuilder.configureHolonomic(this::getOdometryPose2d, this::resetOdometry, this::getChassisSpeeds,
        this::driveChassisSpeeds, Constants.Drivetrain.kPathingConfig,
        () -> {
          return DriverStation.getAlliance().get() == Alliance.Red;
        }, this);
  }

  public void init() {
    m_logger.info("Initializing DrivetrainSub");
    resetGyroYaw(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Only run the odometry when the driver's station has Auto selected (doesn't have to be enabled)
    if(DriverStation.isAutonomous()) {
      updateOdometry();
      double xPos = m_odometry.getPoseMeters().getX();
      double yPos = m_odometry.getPoseMeters().getY();
      if(!RobotContainer.disableShuffleboardPrint) {
        m_sbXPOS.setDouble(xPos);
        m_sbYPOS.setDouble(yPos);
      }
    }
    m_sbYaw.setDouble(getYawRotation2d().getDegrees());
    m_sbFLEncoder.setDouble(m_frontLeft.getTurningEncoder());
    m_sbFREncoder.setDouble(m_frontRight.getTurningEncoder());
    m_sbBLEncoder.setDouble(m_backLeft.getTurningEncoder());
    m_sbBREncoder.setDouble(m_backRight.getTurningEncoder());

    if(!RobotContainer.disableShuffleboardPrint) {
      // SmartDashboard.putNumber("Held Angle", MathUtil.inputModulus(previousRotation.getDegrees(), -180.0, 180.0));

      double rot = MathUtil.inputModulus(targetPos.getRotation().getDegrees(), -180.0, 180.0);
      m_sbTargetXPOS.setDouble(targetPos.getX());
      m_sbTargetYPOS.setDouble(targetPos.getY());
      m_sbTargetROT.setDouble(rot);

      m_sbGYRO.setDouble(getYawRotationDegrees());
      // m_sbYaw.setDouble(m_gyro.getAngle());
      m_sbRoll.setDouble(m_gyro.getRoll());
      m_sbPitch.setDouble(m_gyro.getPitch());
      m_sbSerialNumber.setString(Constants.Drivetrain.serialNumber);

      // ChassisSpeeds currentSpeeds = getChassisSpeeds();
      // double vx = currentSpeeds.vxMetersPerSecond;
      // double vy = currentSpeeds.vyMetersPerSecond;
      // double currentSpeed = Math.sqrt(vx * vx + vy * vy);
      // SmartDashboard.putNumber("Speed (m/s)", currentSpeed);
      // kPIDp = SmartDashboard.getNumber("Path kP", kPIDp);
      // kPIDd = SmartDashboard.getNumber("Path kD", kPIDd);
      // kThreshold = SmartDashboard.getNumber("Path Threshold", kThreshold);
      // kRotPIDp = SmartDashboard.getNumber("Rot kP", kRotPIDp);
      // kRotPIDd = SmartDashboard.getNumber("Rot kD", kRotPIDd);
      // kTurnThreshold = SmartDashboard.getNumber("Rot Threshold", kTurnThreshold);

      m_sbRotKP.setDouble(kRotPIDp);
      m_sbRotKD.setDouble(kRotPIDd);
      m_sbRotThreshold.setDouble(kTurnThreshold);
      m_sbPathKP.setDouble(kPIDp);
      m_sbPathKD.setDouble(kPIDd);
      m_sbPathThreshold.setDouble(kThreshold);

    }
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

  public Rotation2d getYawRotation2d() {
    return m_gyro.getRotation2d();
  }

  public double getYawRotationDegrees() {
    return MathUtil.inputModulus(getYawRotation2d().getDegrees(), -180.0, 180.0);
  }

  public void resetGyroYaw(double angle) {
    m_gyro.setAngleAdjustment(-angle - Constants.Drivetrain.kGyroPhysicalOffsetAngle); // NavX is oriented 90deg off of front
    m_gyro.reset();
    resetOdometry(); // Feed in rotation here too
  }

  public void postAutoResetYaw() {
    if(DriverStation.getAlliance().get() == Alliance.Red) {
      double previousAngleAdjustment = m_gyro.getAngleAdjustment();
      double angleAdjustment = previousAngleAdjustment;
      if(previousAngleAdjustment <= 0) {
        angleAdjustment += 180;
      } else {
        angleAdjustment -= 180;
      }
      m_logger.fine("Adj " + m_gyro.getAngleAdjustment() + " crrent " + angleAdjustment);
      m_gyro.setAngleAdjustment(angleAdjustment);
      m_gyro.reset();
      resetOdometry();
    }
  }

  public float getRollRotationDegrees() {
    // proto type bot roll is navx pitch
    return m_gyro.getPitch();
  }

  public void drive(double xSpeed, double ySpeed, double rotationSpeed, double periodSeconds) { // Period should be time period between whenever this is called
    xSpeed *= kMaxDriveSpeed;
    ySpeed *= kMaxDriveSpeed;
    rotationSpeed *= kMaxTurnSpeed; // This is negative so it's CCW Positive
    //var swerveStates = m_kinematics.toSwerveModuleStates(speedS); // Get swerve states
    // X and Y are swapped because it drives sideways for some reason
    var swerveStates = m_kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, getYawRotation2d()), periodSeconds)); // Get swerve states

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveStates, kMaxDriveSpeed); // Keep motors below max speed (Might not need to be used)

    // Drive motors
    m_frontLeft.setState(swerveStates[0]);
    m_frontRight.setState(swerveStates[1]);
    m_backLeft.setState(swerveStates[2]);
    m_backRight.setState(swerveStates[3]);
  }

  public ChassisSpeeds getChassisSpeeds() { // Returns current chassis speeds (for Pathing)
    return m_kinematics.toChassisSpeeds(new SwerveModuleState[] {m_frontLeft.getState(), m_frontRight.getState(),
        m_backLeft.getState(), m_backRight.getState()});
  }

  public void driveStates(SwerveModuleState[] swerveStates) { // Specifically for trajectories
    m_frontLeft.setState(swerveStates[0]);
    m_frontRight.setState(swerveStates[1]);
    m_backLeft.setState(swerveStates[2]);
    m_backRight.setState(swerveStates[3]);
  }

  public void driveChassisSpeeds(ChassisSpeeds speeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(speeds, 0.02); // Everyone uses 0.02 for the period for some reason
    SwerveModuleState[] targetStates = m_kinematics.toSwerveModuleStates(targetSpeeds);
    driveStates(targetStates);
  }

  public void translateOdometry(Translation2d pos) { // Set target position 
    targetPos = new Pose2d(pos.getX(), pos.getY(), getYawRotation2d());
  }

  public void translateOdometry(Pose2d pos) { // Set target position and rotation (degrees)
    targetPos = new Pose2d(pos.getTranslation(), pos.getRotation());
  }

  public double getRotationPIDPowerDegrees(double target) {
    return MathUtil.clamp(
        m_odometryPIDr.calculate(getYawRotationDegrees(),
            MathUtil.inputModulus(target, -180.0, 180.0)),
        -1.0, 1.0);
  }

  public boolean updateOdometryTransform() { // Returns true when at position
    //double rotationDifference = m_odometryPIDr.getPositionError(); // In degrees
    double xPower = MathUtil.clamp(m_odometryPIDx.calculate(getPos().getX(), targetPos.getX()), -0.5, 0.5);
    double yPower = MathUtil.clamp(m_odometryPIDy.calculate(getPos().getY(), targetPos.getY()), -0.5, 0.5);
    double rotationPower = getRotationPIDPowerDegrees(targetPos.getRotation().getDegrees());
    drive(xPower, yPower, rotationPower, 0.02);
    return ((m_odometryPIDx.atSetpoint() && m_odometryPIDy.atSetpoint()) && m_odometryPIDr.atSetpoint());
  }

  public boolean updateOdometryTransform(double maxPower, boolean passthrough) { // Returns true when at position
    //double rotationDifference = m_odometryPIDr.getPositionError(); // In degrees
    double rotationClamp = 1.0;
    double xPower = MathUtil.clamp(m_odometryPIDx.calculate(getPos().getX(), targetPos.getX()), -maxPower, maxPower);
    double yPower = MathUtil.clamp(m_odometryPIDy.calculate(getPos().getY(), targetPos.getY()), -maxPower, maxPower);
    double rotationPower =
        MathUtil.clamp(
            m_odometryPIDr.calculate(getYawRotationDegrees(),
                MathUtil.inputModulus(targetPos.getRotation().getDegrees(), -180.0, 180.0)),
            -rotationClamp, rotationClamp);
    drive(xPower, yPower, rotationPower, 0.02);
    if(passthrough) {
      return ((m_odometryPIDx.getPositionError() < m_odometryPIDx.getPositionTolerance()
          && (m_odometryPIDy.getPositionError() < m_odometryPIDy.getPositionTolerance())));
    } else {
      return ((m_odometryPIDx.atSetpoint() && m_odometryPIDy.atSetpoint()));
    }
  }

  public void updateOdometry() {
    m_odometry.update(getYawRotation2d(), new SwerveModulePosition[] {
        m_frontLeft.getPosition(), m_frontRight.getPosition(), m_backLeft.getPosition(), m_backRight.getPosition()});
  }

  public void resetOdometry() {
    Pose2d zero = new Pose2d(new Translation2d(0.0, 0.0), getYawRotation2d());
    m_odometry.resetPosition(getYawRotation2d(), new SwerveModulePosition[] {
        m_frontLeft.getPosition(), m_frontRight.getPosition(), m_backLeft.getPosition(), m_backRight.getPosition()},
        zero);
  }

  public void resetOdometry(Pose2d pos) {
    resetGyroYaw(pos.getRotation().getDegrees());
    m_odometry.resetPosition(getYawRotation2d(), new SwerveModulePosition[] {
        m_frontLeft.getPosition(), m_frontRight.getPosition(), m_backLeft.getPosition(), m_backRight.getPosition()},
        pos);
  }

  public Translation2d getPos() { // Get position from odometry
    return m_odometry.getPoseMeters().getTranslation();
  }

  public Pose2d getOdometryPose2d() {
    return m_odometry.getPoseMeters();
  }

  public double getTurningEncoderFL() {
    return m_frontLeft.getTurningEncoder();
  }

  public double getTurningEncoderFR() {
    return m_frontRight.getTurningEncoder();
  }

  public double getTurningEncoderBL() {
    return m_backLeft.getTurningEncoder();
  }

  public double getTurningEncoderBR() {
    return m_backRight.getTurningEncoder();
  }

  public void fun() {
    if(Robot.inTestMode) { // Only run this in test mode
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
        orca1.loadMusic("shpitaA_up.chrp");
        orca2.loadMusic("shpitaB_up.chrp");
        orca3.loadMusic("shpitaC_up.chrp");
        orca4.loadMusic("shpitaD_up.chrp");
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
  }

  public void nofun() {
    if(orca1.isPlaying() || orca2.isPlaying()) {
      orca1.stop();
      orca2.stop();
      orca3.stop();
      orca4.stop();
    }
  }
}
