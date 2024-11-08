// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class DrivetrainSub extends SubsystemBase {
  private static Logger m_logger = Logger.getLogger(DrivetrainSub.class.getName());

  // Speed multipliers
  // TODO ONCMP
  // Test both of these extensively with a good battery. Turn them way up, and see how fast we can actually get
  // the robot turning, and driving, then update these.
  public static final double kMaxTurnSpeed = 9.0; // In radians/s

  // Locations of Swerve Modules relative to the center of the robot
  //Includes translation 2d offset (Is 0.381 the robot width sqrt(2*length^2)? Possibly in meters?)
  //Translation 2d has an x and y component, which together represent a two dimensional vector or point.
  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.318); // I have no idea why these are 0.381
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.318);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.318);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.318);
  //Creates a shuffleboard tab and adds entries to it
  private final ShuffleboardTab m_shuffleboardTab = Shuffleboard.getTab("DriveTrain");
  private final GenericEntry m_sbYPOS, m_sbXPOS, m_sbTargetXPOS, m_sbTargetYPOS, m_sbTargetROT, m_sbGYRO, m_sbYaw,
      m_sbRoll, m_sbPitch, m_sbFLEncoder,
      m_sbFREncoder, m_sbBLEncoder, m_sbBREncoder, m_sbRotKP, m_sbRotKD, m_sbRotThreshold, m_sbPathKP, m_sbPathKD,
      m_sbPathThreshold, m_sbSerialNumber, m_sbRobotName;

  /*
   * WIP
   * PID explanation (Uses cruise control as a metaphor):
   * PID contains three components, proportional, integral, and derivative. The PID takes in a setpoint value, which is
   * a goal defined by the user.
   * The setpoint is the equivalent of the set speed in a cruise control system. The process value is the value being
   * manipulated by the PID, like
   * the actual speed of the car. The output is the system used to control the process value, like the throttle.
   * Finally, the error is value used by
   * the PID to determine how it wants to manipulate the output, it's represented by the setpoint minus the process
   * value (set speed minus actual
   * speed).
   * 
   * The controller itself, as was previously stated, is made up of three components. These components are modified by
   * the gain, which is just a
   * multiplication factor. The user sets the gain values for each component seperately. They impact how much effect
   * each component has on the
   * output.
   * 
   * Proportional - The proportional is used for very coarse adjustment. It has a large, immediate impact on the output,
   * which lessens as the error approaches the setpoint. The proportional is calculated by multiplying the proportional
   * gain by the error.
   * 
   * Integral - The integral is a more fine adjustment to start, but then gradually accumulates. Every cycle, the
   * integral gain is multipled
   * the error value and cycle time, then added to the integral total. In otherwords, the longer it takes to reach the
   * setpoint, the greater
   * the impact of the integral.
   * 
   * Derivative - The derivative is intended to "predict" where the process value is headed, and oppose the propotional
   * and integral to
   * prevent overshoot. It's calculated by subtracting the previous error from the current error value, multiplying the
   * result by the
   * derivative gain, then dividing it by the cycle time.
   * 
   * Output - The output is found by adding the P, I, and D values together.
   */

  // PID value setting
  private double kPIDp = 0.4; //Proportional value
  private double kPIDd = 0.0; //Derivative value
  private double kThreshold = 0.05; //Error threshold of the robot's position (acceptable distance from the goal in meters). Used by PID

  private double kRotPIDp = 0.005; //Proportional value (rotational)
  private double kRotPIDd = 0.0; //Derivative value (rotational)
  private double kTurnThreshold = 1.0; //Error threshold of the robot's rotational position (acceptable rotational distance from the goal in degrees). Unused as of now.

  //Create target positions coordinate with Pose2d (translation2d+rotation2d)
  //Rotation2d is a single radian value in this case, it is a continuous value
  private Pose2d targetPos = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
  //Constructs new PID controllers using the previously set values. One PID for x translation, one for y, one for chassis rotation.
  //and one for lob pass specifically
  private PIDController m_odometryPIDx = new PIDController(kPIDp, 0.0, kPIDd); // X and Y PIDs
  private PIDController m_odometryPIDy = new PIDController(kPIDp, 0.0, kPIDd);
  private PIDController m_odometryPIDr = new PIDController(kRotPIDp, 0.0, kRotPIDd); // Rotational PID
  private PIDController m_rotateToLobPID = new PIDController(0.005, 0.0, 0.0); // Rotational PID


  // Swerve Modules that control the motors, each one is a unique swerve module object.
  private final SwerveModule m_frontLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_backLeft;
  private final SwerveModule m_backRight;

  //Uses the Kauai labs gyro class to create a new gyro object
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // Kinematics controls movement, Odemetry tracks position.
  //The kinematics object will be used later in the construction of the SwerveDriveModule and SwerveDriveKinematics objects.
  //The odometry object will be mainly used for robot positioning during the autonomous period
  private final SwerveDriveKinematics m_kinematics;
  private final SwerveDriveOdometry m_odometry;


  /** Creates a new DrivetrainSub. */
  public DrivetrainSub() {
    //Initializes a variable used for determining robot specific values, and varibalbes for holding the encoder offsets
    String robotName;
    double AbsoluteEncoderOffsetFL;
    double AbsoluteEncoderOffsetFR;
    double AbsoluteEncoderOffsetBL;
    double AbsoluteEncoderOffsetBR;
    //Gets the correct encoder values from constants and assigns each one to its respective variable
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

    //Creates the swerve module objects, taking in drive motor, steering motor, and encoder objects as parameters as well as the encoder offset
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
    //Assigns the kinematics object a new SwerveDriveKinematics object which is comprised of the previously created swerve module objects
    //This is used for interacting with the chassis as a whole, instead each swerve module indepenently
    m_kinematics =
        new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
    //Assigns the odometry object a new SwerveDriveOdometry object which is comprised of the kinematics object, the robot's rotation, and array
    //of all the swerve module positions (module rotation and wheel rotation)
    m_odometry =
        new SwerveDriveOdometry(m_kinematics, getYawRotation2d(), new SwerveModulePosition[] {
            m_frontLeft.getPosition(), m_frontRight.getPosition(), m_backLeft.getPosition(),
            m_backRight.getPosition()});

    //Resets the gyro yaw value
    resetGyroYaw(0);

    //Sets the tolerances of the PID for the x, y, rotational values, and note lobing
    //Constrains the rotational PID to a fixed input range (-180, 180 degrees)
    m_odometryPIDx.setTolerance(kThreshold); // In meters
    m_odometryPIDy.setTolerance(kThreshold); // In meters

    m_odometryPIDr.setTolerance(1.0, 3.0); // In degrees
    m_odometryPIDr.enableContinuousInput(-180.0, 180.0);

    m_rotateToLobPID.setTolerance(1.0, 3.0); // In degrees
    m_rotateToLobPID.enableContinuousInput(-180.0, 180.0);


    //Adds the listed values to the drivetrain tab on the smart dashboard
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

    //Configures pathplanner to work with a holonominc drivetrain (swerve). Provides the robots x,y position, a method to reset the odometry?,
    //the current chassis speed, a goal chassis speed, and a pathing configuarion object (check kPathConfig definition for more infotmation).
    //Returns a boolean based on the alliance colour
    AutoBuilder.configureHolonomic(this::getOdometryPose2d, this::resetOdometry, this::getChassisSpeeds,
        this::driveChassisSpeeds, Constants.Drivetrain.kPathingConfig,
        () -> {
          return DriverStation.getAlliance().get() == Alliance.Red;
        }, this);
  }

  //Adds info to logs and resets gyro
  public void init() {
    m_logger.info("Initializing DrivetrainSub");
    resetGyroYaw(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Only run the odometry when the driver's station has Auto selected (doesn't have to be enabled)
    //if(DriverStation.isAutonomous()) {
    //Runs the reset odometry method
    updateOdometry();
    //Sets the odometry values to the robot's current position
    double xPos = m_odometry.getPoseMeters().getX();
    double yPos = m_odometry.getPoseMeters().getY();
    //if(!RobotContainer.disableShuffleboardPrint) {
    //Sets shuffleboard values for x, y, rotation, and encoder values
    m_sbXPOS.setDouble(xPos);
    m_sbYPOS.setDouble(yPos);
    //}
    //}
    m_sbYaw.setDouble(getYawRotation2d().getDegrees());
    m_sbFLEncoder.setDouble(m_frontLeft.getTurningEncoder());
    m_sbFREncoder.setDouble(m_frontRight.getTurningEncoder());
    m_sbBLEncoder.setDouble(m_backLeft.getTurningEncoder());
    m_sbBREncoder.setDouble(m_backRight.getTurningEncoder());
    //Sets variables for chassis speed and robot velocity
    ChassisSpeeds currentSpeeds = getChassisSpeeds();
    double vx = currentSpeeds.vxMetersPerSecond;
    double vy = currentSpeeds.vyMetersPerSecond;
    double currentSpeed = Math.sqrt(vx * vx + vy * vy);
    //Adds speed and motor power to smart dashboard
    SmartDashboard.putNumber("Speed (m/s)", currentSpeed);
    SmartDashboard.putNumber("FL Power", m_frontLeft.testGetPower());
    SmartDashboard.putNumber("FR Power", m_frontRight.testGetPower());
    SmartDashboard.putNumber("BL Power", m_backLeft.testGetPower());
    SmartDashboard.putNumber("BR Power", m_backRight.testGetPower());

    if(true) {
      // SmartDashboard.putNumber("Held Angle", MathUtil.inputModulus(previousRotation.getDegrees(), -180.0, 180.0));


      //Sets target position, roll, pitch, gyro angle, and serial number values for shuffleboard
      double rot = MathUtil.inputModulus(targetPos.getRotation().getDegrees(), -180.0, 180.0);
      m_sbTargetXPOS.setDouble(targetPos.getX());
      m_sbTargetYPOS.setDouble(targetPos.getY());
      m_sbTargetROT.setDouble(rot);

      m_sbGYRO.setDouble(getYawRotationDegrees());
      // m_sbYaw.setDouble(m_gyro.getAngle());
      m_sbRoll.setDouble(m_gyro.getRoll());
      m_sbPitch.setDouble(m_gyro.getPitch());
      m_sbSerialNumber.setString(Constants.Drivetrain.serialNumber);


      // kPIDp = SmartDashboard.getNumber("Path kP", kPIDp);
      // kPIDd = SmartDashboard.getNumber("Path kD", kPIDd);
      // kThreshold = SmartDashboard.getNumber("Path Threshold", kThreshold);
      // kRotPIDp = SmartDashboard.getNumber("Rot kP", kRotPIDp);
      // kRotPIDd = SmartDashboard.getNumber("Rot kD", kRotPIDd);
      // kTurnThreshold = SmartDashboard.getNumber("Rot Threshold", kTurnThreshold);

      //Sets PID values for shuffleboard
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

  //Method to get yaw rotation2d using gyro angle
  public Rotation2d getYawRotation2d() {
    return m_gyro.getRotation2d();
  }

  //Method to get yaw angle degrees using previously created yaw rotation2d method
  public double getYawRotationDegrees() {
    return MathUtil.inputModulus(getYawRotation2d().getDegrees(), -180.0, 180.0);
  }

  //Method to reset gyro angle
  public void resetGyroYaw(double angle) {
    m_gyro.setAngleAdjustment(-angle - Constants.Drivetrain.kGyroPhysicalOffsetAngle); // NavX is oriented 90deg off of front
    m_gyro.reset();
    resetOdometry(); // Feed in rotation here too
  }

  //Used to adjust gyro angle post auto if the robot starts on the blue alliance. This is necessary because all autos are the same
  //regrardless of alliance, so the robot always assumes it starts on red. This would cause the robot to be inverted when on the bue alliacne.
  //It adds 180 to the gyro if the angle is less than 0 since we use values between -180 and 180 doing this above 0 would cause the gyro to
  //pass 180, and mess up the code. This is the opposite for angles greater than 0
  public void postAutoResetYaw() {
    if(DriverStation.getAlliance().get() != Alliance.Blue) {
      double previousAngleAdjustment = m_gyro.getAngleAdjustment();
      double angleAdjustment = previousAngleAdjustment;
      if(previousAngleAdjustment <= 0) {
        angleAdjustment += 180;
      } else {
        angleAdjustment -= 180;
      }
      //Updates logs, sets Gyro, and resets odometry
      m_logger.fine("Adj " + m_gyro.getAngleAdjustment() + " current " + angleAdjustment);
      m_gyro.setAngleAdjustment(angleAdjustment);
      resetOdometry();
    }
  }

  //Method to get robot rotation using the gyro's pitch
  public float getRollRotationDegrees() {
    // proto type bot roll is navx pitch
    return m_gyro.getPitch();
  }

  //Creates drive method using x, y and rotation speeds, and the period (0.02 seconds)
  public void drive(double xSpeed, double ySpeed, double rotationSpeed, double periodSeconds) { // Period should be time period between whenever this is called
    //Sets the max x and y speeds with the kMaxChassisSpeed constant
    xSpeed *= Constants.Drivetrain.kMaxChassisSpeed;
    ySpeed *= Constants.Drivetrain.kMaxChassisSpeed;
    //Sets max rotation speed with the kMaxTurnSpeed constant
    rotationSpeed *= kMaxTurnSpeed; // This is negative so it's CCW Positive
    //var swerveStates = m_kinematics.toSwerveModuleStates(speedS); // Get swerve states
    // X and Y are swapped because it drives sideways for some reason

    //Takes in chassis speed (x, y, rotation) and converts it to swerve module states (wheel speed and angle)
    var swerveStates = m_kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, getYawRotation2d()), periodSeconds)); // Get swerve states

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveStates, Constants.Drivetrain.kMaxChassisSpeed); // Keep motors below max speed (Might not need to be used)

    // Drive motors
    //Assigns drive motors swerve module states
    m_frontLeft.setState(swerveStates[0]);
    m_frontRight.setState(swerveStates[1]);
    m_backLeft.setState(swerveStates[2]);
    m_backRight.setState(swerveStates[3]);
  }

  //Gets chassis speed using swerve module states
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

  //Sets target chassis speeds to the normalized wheel speeds, then assigns the target states (which use the target speeds) to the
  //trajectory specific drive states
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

  //Calculates PID power to reach a target position using the current rotation, and the target location
  //Accomodates the continuous nature of the rotation using the inputMouulus method.
  //Clamps the power between 1 and -1
  public double getRotationPIDPowerDegrees(double target) {
    return MathUtil.clamp(
        m_odometryPIDr.calculate(getYawRotationDegrees(),
            MathUtil.inputModulus(target, -180.0, 180.0)),
        -1.0, 1.0);
  }

  //Preforms a similar calculation to the previous method, with the addition of what appears to be an error threshold
  public double getLobRotationPower(double target) {
    // SmartDashboard.putNumber("difference", Math.abs(getYawRotationDegrees() - target));
    // SmartDashboard.putNumber("speed", Math.abs(m_gyro.getRate()));
    if(Math.abs(getYawRotationDegrees() - target) < 8 * Math.abs(m_gyro.getRate())) {
      return 0;
    }
    return MathUtil.clamp(
        m_rotateToLobPID.calculate(getYawRotationDegrees(),
            MathUtil.inputModulus(target, -180.0, 180.0)),
        -1.0, 1.0);
  }

  //Calculates the required power to reach the target position using PIDs and 0.5/-0.5 clamps, then calls the drive command using the power values
  public boolean updateOdometryTransform() { // Returns true when at position
    //double rotationDifference = m_odometryPIDr.getPositionError(); // In degrees
    double xPower = MathUtil.clamp(m_odometryPIDx.calculate(getPos().getX(), targetPos.getX()), -0.5, 0.5);
    double yPower = MathUtil.clamp(m_odometryPIDy.calculate(getPos().getY(), targetPos.getY()), -0.5, 0.5);
    double rotationPower = getRotationPIDPowerDegrees(targetPos.getRotation().getDegrees());
    drive(xPower, yPower, rotationPower, 0.02);
    return ((m_odometryPIDx.atSetpoint() && m_odometryPIDy.atSetpoint()) && m_odometryPIDr.atSetpoint());
  }

  //Same as previous except it takes in max power and passthough parameters, 
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

  //Updates swerve drive odometry using the swerve module positions and the gyro angle
  public void updateOdometry() {
    m_odometry.update(getYawRotation2d(), new SwerveModulePosition[] {
        m_frontLeft.getPosition(), m_frontRight.getPosition(), m_backLeft.getPosition(),
        m_backRight.getPosition()});
  }

  //Resets the odomety by creating a new Pose2d with zero values, then setting the swerve module positions equal to that Pose2d
  public void resetOdometry() {
    Pose2d zero = new Pose2d(new Translation2d(0.0, 0.0), getYawRotation2d());
    m_odometry.resetPosition(getYawRotation2d(), new SwerveModulePosition[] {
        m_frontLeft.getPosition(), m_frontRight.getPosition(), m_backLeft.getPosition(),
        m_backRight.getPosition()},
        zero);
  }

  //Same as previous method expet the Pose2d is taken in as a parameter instead of being immedietly set to 0
  public void resetOdometry(Pose2d pos) {
    resetGyroYaw(pos.getRotation().getDegrees());
    m_odometry.resetPosition(getYawRotation2d(), new SwerveModulePosition[] {
        m_frontLeft.getPosition(), m_frontRight.getPosition(), m_backLeft.getPosition(),
        m_backRight.getPosition()},
        pos);
  }

  public Translation2d getPos() { // Get position from odometry
    return m_odometry.getPoseMeters().getTranslation();
  }

  //The next five methods get Pose2d values and encoder values
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

  public void runMotor(double power) {
    m_frontLeft.testSetPower(power);
    m_frontRight.testSetPower(power);
    m_backLeft.testSetPower(power);
    m_backRight.testSetPower(power);
  }

  //For playing music, thanks Aidan
}
