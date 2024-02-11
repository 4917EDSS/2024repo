// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
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
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class DrivetrainSub extends SubsystemBase {
  private static Logger m_logger = Logger.getLogger(DrivetrainSub.class.getName());

  //Analog sensors
  //private final AnalogInput m_frontDistanceSensor = new AnalogInput(Constants.AnalogInIds.kFrontDistanceSenor);

  private double m_orientationOffsetDegrees = 0;

  // Speed multipliers
  public static final double kMaxDriveSpeed = 100.0; // In m/s   // TODO: Need real values
  public static final double kMaxTurnSpeed = 30.0; // was 50     // TODO: Need real values

  // Locations of Swerve Modules relative to the center of the robot
  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.318);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.318);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.318);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.318);

  private final ShuffleboardTab m_shuffleboardTab = Shuffleboard.getTab("DriveTrain");
  private final GenericEntry m_sbYPOS, m_sbXPOS, m_sbGYRO, m_sbYaw, m_sbRoll, m_sbPitch, m_sbFLEncoder, m_sbFREncoder,
      m_sbBLEncoder, m_sbBREncoder, m_sbRotKP, m_sbRotKD, m_sbRotThreshold;

  // PID value setting
  private double kRotPIDp = 0.005;
  private double kRotPIDd = 0.0;
  private double kTurnThreshold = 1.0;
  private PIDController m_odometryPidR = new PIDController(kRotPIDp, 0.0, kRotPIDd); // Rotational PID

  // Swerve Modules that control the motors
  private final SwerveModule m_frontLeft =
      new SwerveModule(Constants.CanIds.kDriveMotorFL, Constants.CanIds.kSteeringMotorFL, Constants.CanIds.kEncoderFL,
          Constants.Drivetrain.kAbsoluteEncoderOffsetFL);
  private final SwerveModule m_frontRight =
      new SwerveModule(Constants.CanIds.kDriveMotorFR, Constants.CanIds.kSteeringMotorFR, Constants.CanIds.kEncoderFR,
          Constants.Drivetrain.kAbsoluteEncoderOffsetFR);
  private final SwerveModule m_backLeft =
      new SwerveModule(Constants.CanIds.kDriveMotorBL, Constants.CanIds.kSteeringMotorBL, Constants.CanIds.kEncoderBL,
          Constants.Drivetrain.kAbsoluteEncoderOffsetBL);
  private final SwerveModule m_backRight =
      new SwerveModule(Constants.CanIds.kDriveMotorBR, Constants.CanIds.kSteeringMotorBR, Constants.CanIds.kEncoderBR,
          Constants.Drivetrain.kAbsoluteEncoderOffsetBR);

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // Kinematics controls movement, Odemetry tracks position
  public final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d(), new SwerveModulePosition[] {
          m_frontLeft.getPosition(), m_frontRight.getPosition(), m_backLeft.getPosition(), m_backRight.getPosition()});


  /** Creates a new DrivetrainSub. */
  public DrivetrainSub() {
    m_gyro.reset();
    m_gyro.setAngleAdjustment(90);

    m_odometryPidR.setTolerance(1.0, 3.0); // In degrees
    m_odometryPidR.enableContinuousInput(-180.0, 180.0);

    m_sbXPOS = m_shuffleboardTab.add("XPOS", 0.0).getEntry();
    m_sbYPOS = m_shuffleboardTab.add("YPOS", 0.0).getEntry();

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

    // Configure the AutoBuilder (see https://pathplanner.dev/pplib-getting-started.html)
    // This needs to be done last
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            4.5, // Max module speed, in m/s
            0.496, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          if(alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
  }

  public void init() {
    m_logger.info("Initializing DrivetrainSub");
    resetGyro();
    resetOdometry();

    // TODO:  Do we need to call each swerve module's init()?
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry(); // TODO: Move this to an autonomous periodic so it isn't running during teleop
    double xPos = m_odometry.getPoseMeters().getX();
    double yPos = m_odometry.getPoseMeters().getY();

    m_sbXPOS.setDouble(xPos);
    m_sbYPOS.setDouble(yPos);

    m_sbGYRO.setDouble(getRotationDegrees());
    m_sbYaw.setDouble(m_gyro.getAngle() % 360);
    m_sbRoll.setDouble(m_gyro.getRoll());
    m_sbPitch.setDouble(m_gyro.getPitch());

    m_sbFLEncoder.setDouble(m_frontLeft.getTurningEncoder());
    m_sbFREncoder.setDouble(m_frontRight.getTurningEncoder());
    m_sbBLEncoder.setDouble(m_backLeft.getTurningEncoder());
    m_sbBREncoder.setDouble(m_backRight.getTurningEncoder());

    kRotPIDp = SmartDashboard.getNumber("Rot kP", kRotPIDp);
    kRotPIDd = SmartDashboard.getNumber("Rot kD", kRotPIDd);
    kTurnThreshold = SmartDashboard.getNumber("Rot Threshold", kTurnThreshold);
    m_sbRotKP.setDouble(kRotPIDp);
    m_sbRotKD.setDouble(kRotPIDd);
    m_sbRotThreshold.setDouble(kTurnThreshold);

    // Setting PID constants
    m_odometryPidR.setP(kRotPIDp);
    m_odometryPidR.setD(kRotPIDd);
    m_odometryPidR.setTolerance(kTurnThreshold);
  }


  public void resetGyro() {
    m_gyro.reset();
  }

  public float getRoll() {
    // proto type bot roll is navx pitch
    return m_gyro.getPitch();
  }

  // For auto driving using PathPlanner
  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    var swerveStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveStates, kMaxDriveSpeed); // Keep motors below max speed (Might not need to be used)
  }

  // For tele-op driving
  public void drive(double xSpeed, double ySpeed, double rotationSpeed, double periodSeconds) { // Period should be time period between whenever this is called
    xSpeed *= kMaxDriveSpeed;
    ySpeed *= kMaxDriveSpeed;
    rotationSpeed *= kMaxTurnSpeed; // This is negative so it's CCW Positive
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

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return m_kinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState());
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

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getRotation2d(), new SwerveModulePosition[] {
        m_frontLeft.getPosition(), m_frontRight.getPosition(), m_backLeft.getPosition(), m_backRight.getPosition()},
        pose);
  }

  ////
  // The following methods support rotation-locked tele-op swerve driving
  public Rotation2d getRotation() {
    return m_gyro.getRotation2d();
  }

  public double getRotationDegrees() {
    return MathUtil.inputModulus(getRotation().getDegrees() + m_orientationOffsetDegrees, -180.0, 180.0);
  }

  public double getRotationPIDPowerDegrees(double target) {
    return MathUtil.clamp(
        m_odometryPidR.calculate(getRotationDegrees(),
            MathUtil.inputModulus(target, -180.0, 180.0)),
        -1.0, 1.0);
  }
  ////
}
