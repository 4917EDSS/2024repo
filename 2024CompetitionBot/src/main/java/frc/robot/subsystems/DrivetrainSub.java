// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Logger;
import com.ctre.phoenix6.Orchestra;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.Constants;
// import edu.wpi.first.wpilibj.AnalogInput;


public class DrivetrainSub extends SubsystemBase {
  // TODO: Remove both classes and all relating functions when trajectory is finished
  public class Point {
    public Pose2d point;
    public boolean passthough = false; // Don't slow down at this point

    public Point(Pose2d p, boolean pass) {
      point = p;
      passthough = pass;
    }
  }
  public class Path {
    private ArrayList<Point> points = new ArrayList<Point>();
    private double pTolerance = kThreshold;
    private double pMaxspeed = 1.0;
    public int index = 0;

    public Path(double tolarance, double maxSpeed) {
      pTolerance = tolarance;
      pMaxspeed = maxSpeed;
    }

    void addPoint(Pose2d p) {
      Point t = new Point(p, false);
      points.add(t);
    }

    void addPoint(double x, double y, Rotation2d r) {
      Pose2d temp = new Pose2d(x, y, r);
      Point t = new Point(temp, false);
      points.add(t);
    }

    void addPoint(double x, double y, Rotation2d r, boolean pass) {
      Pose2d temp = new Pose2d(x, y, r);
      Point t = new Point(temp, pass);
      points.add(t);
    }
  }

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

  private double kRotPIDp = 0.005;
  private double kRotPIDd = 0.0;
  private double kTurnThreshold = 1.0;

  private Pose2d targetPos = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
  private Rotation2d previousRotation;
  public PIDController m_odometryPIDx = new PIDController(kPIDp, 0.0, kPIDd); // X and Y PIDs
  public PIDController m_odometryPIDy = new PIDController(kPIDp, 0.0, kPIDd);
  private PIDController m_odometryPIDr = new PIDController(kRotPIDp, 0.0, kRotPIDd); // Rotational PID
  private PIDController m_drivePIDr = new PIDController(0.05, 0.0, 0.0);

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

    m_sbFLEncoder.setDouble(m_frontLeft.getTurningEncoder());
    m_sbFREncoder.setDouble(m_frontRight.getTurningEncoder());
    m_sbBLEncoder.setDouble(m_backLeft.getTurningEncoder());
    m_sbBREncoder.setDouble(m_backRight.getTurningEncoder());

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

  public void driveStates(SwerveModuleState[] swerveStates) { // Specifically for trajectories
    m_frontLeft.setState(swerveStates[0]);
    m_frontRight.setState(swerveStates[1]);
    m_backLeft.setState(swerveStates[2]);
    m_backRight.setState(swerveStates[3]);
  }

  public Path generateTestPath() { // TODO: Remove after finishing trajectory
    Path tesPath = new Path(0.2, 0.2); // 10cm tolerance with 20% speed
    Rotation2d currentRotation = getRotation();
    tesPath.addPoint(0.0 + getPos().getX(), 0.0 + getPos().getY(), currentRotation);
    tesPath.addPoint(0.0 + getPos().getX(), 0.5 + getPos().getY(), currentRotation, true);

    tesPath.addPoint(0.0 + getPos().getX(), 1.0 + getPos().getY(), currentRotation);
    tesPath.addPoint(0.5 + getPos().getX(), 1.0 + getPos().getY(), currentRotation, true);

    tesPath.addPoint(1.0 + getPos().getX(), 1.0 + getPos().getY(), currentRotation);
    tesPath.addPoint(1.0 + getPos().getX(), 0.5 + getPos().getY(), currentRotation, true);

    tesPath.addPoint(1.0 + getPos().getX(), 0.0 + getPos().getY(), currentRotation);
    tesPath.addPoint(0.5 + getPos().getX(), 0.0 + getPos().getY(), currentRotation, true);

    tesPath.addPoint(0.0 + getPos().getX(), 0.0 + getPos().getY(), currentRotation);
    return tesPath;
  }

  public void startPath(Path p) {
    p.index = 0;
    m_odometryPIDx.setTolerance(p.pTolerance);
    m_odometryPIDy.setTolerance(p.pTolerance);
    translateOdometry(p.points.get(p.index).point);
  }

  public boolean runPath(Path p) {
    if(p.index >= p.points.size()) {
      return true;
    }
    if(p.index == p.points.size() - 1) {
      m_odometryPIDx.setTolerance(kThreshold);
      m_odometryPIDy.setTolerance(kThreshold);
    } else {
      m_odometryPIDx.setTolerance(p.pTolerance);
      m_odometryPIDy.setTolerance(p.pTolerance);
    }

    if(updateOdometryTransform(p.pMaxspeed, p.points.get(p.index).passthough)) {
      p.index += 1;
      if(p.index < p.points.size()) {
        translateOdometry(p.points.get(p.index).point);
      }
    }
    return false;
  }

  public void translateOdometry(Translation2d pos) { // Set target position 
    targetPos = new Pose2d(pos.getX(), pos.getY(), m_gyro.getRotation2d());
  }

  public void translateOdometry(Pose2d pos) { // Set target position and rotation (degrees)
    targetPos = new Pose2d(pos.getTranslation(), pos.getRotation());
  }

  public double getRotationPIDPowerDegrees(double target) {
    return MathUtil.clamp(
        m_odometryPIDr.calculate(getRotationDegrees(),
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
            m_odometryPIDr.calculate(getRotationDegrees(),
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

  public Pose2d getOdometryPose2d() {
    return m_odometry.getPoseMeters();
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


  // public double getFrontDistance() {
  //    //TODO: Convert voltage to distance in meters
  //   return m_frontDistanceSensor.getAverageVoltage();
  // }
}
