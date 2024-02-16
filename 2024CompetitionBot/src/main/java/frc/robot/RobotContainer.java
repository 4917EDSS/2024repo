// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimbCmdSetHeightCmd;
import frc.robot.commands.DrivePathCmd;
import frc.robot.commands.DriveToRelativePositionCmd;
import frc.robot.commands.DriverFieldRelativeDriveCmd;
import frc.robot.commands.KillAllCmd;
import frc.robot.commands.ShooterWithJoystickCmd;
import frc.robot.commands.TestLedsCmd;
import frc.robot.subsystems.ClimbSub;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.LedSub;
import frc.robot.subsystems.LedSub.LedColour;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.VisionSub;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final LedSub m_ledSub = new LedSub();
  private final ClimbSub m_climbSub = new ClimbSub();
  private final DrivetrainSub m_drivetrainSub = new DrivetrainSub();
  private final IntakeSub m_intakeSub = new IntakeSub();
  private final ShooterSub m_shooterSub = new ShooterSub(m_ledSub);
  private final VisionSub m_visionSub = new VisionSub();

  private boolean m_isRedAlliance = true;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS4Controller m_driverController =
      new CommandPS4Controller(OperatorConstants.kDriverControllerPort);
  private final CommandPS4Controller m_operatorController =
      new CommandPS4Controller(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_drivetrainSub.setDefaultCommand(
        new DriverFieldRelativeDriveCmd(m_drivetrainSub, m_driverController));
    m_shooterSub.setDefaultCommand(new ShooterWithJoystickCmd(m_operatorController, m_shooterSub));

    // Configure the button bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Driver controller bindings
    m_driverController.L3()
        .onTrue(new KillAllCmd(m_climbSub, m_drivetrainSub, m_intakeSub, m_shooterSub));
    m_driverController.R3()
        .onTrue(new KillAllCmd(m_climbSub, m_drivetrainSub, m_intakeSub, m_shooterSub));

    // m_driverController.square() FREE

    m_driverController.cross()
        .onTrue(new ClimbCmdSetHeightCmd(Constants.Climb.kHeightHookLowered, 0.5,
            m_drivetrainSub,
            m_climbSub));

    m_driverController.circle()
        .onTrue(new ClimbCmdSetHeightCmd(Constants.Climb.kHeightTallHookRaised, 0.5,
            m_drivetrainSub,
            m_climbSub));

    m_driverController.triangle()
        .onTrue(new ClimbCmdSetHeightCmd(Constants.Climb.kHeightShortHookRaised, 0.5,
            m_drivetrainSub,
            m_climbSub));
    m_driverController.share()
        .onTrue(new InstantCommand(() -> m_drivetrainSub.resetGyro(), m_drivetrainSub));

    m_driverController.options().onTrue(new TestLedsCmd(m_ledSub, LedColour.YELLOW));

    m_driverController.touchpad().onTrue(new TestLedsCmd(m_ledSub, LedColour.BLUE));

    m_driverController.PS().onTrue(new InstantCommand(() -> m_drivetrainSub.fun(), m_drivetrainSub));

    // m_driverController.povRight()
    //                 .onTrue(new DriveToRelativePositionCmd(m_drivetrainSub,
    //                                 new Pose2d(2.0, 0.0, Rotation2d.fromDegrees(90.0))));
    m_driverController.povRight().onTrue(new DrivePathCmd(m_drivetrainSub));

    m_driverController.povDown()
        .onTrue(new DriveToRelativePositionCmd(m_drivetrainSub,
            new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(-90.0))));

    m_driverController.povLeft()
        .onTrue(new DriveToRelativePositionCmd(m_drivetrainSub,
            new Pose2d(-2.0, 0.0, Rotation2d.fromDegrees(-90.0))));

    m_driverController.povUp()
        .onTrue(new DriveToRelativePositionCmd(m_drivetrainSub,
            new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(90.0))));


    // Operator controller bindings
    m_operatorController.L3()
        .onTrue(new KillAllCmd(m_climbSub, m_drivetrainSub, m_intakeSub, m_shooterSub));
    m_operatorController.R3()
        .onTrue(new KillAllCmd(m_climbSub, m_drivetrainSub, m_intakeSub, m_shooterSub));

    m_operatorController.square()
        .onTrue(new StartEndCommand(() -> m_shooterSub.spinUpperFeeder(-0.25),
            () -> m_shooterSub.spinUpperFeeder(0.0), m_shooterSub));

    m_operatorController.cross()
        .onTrue(new StartEndCommand(() -> m_shooterSub.spinLowerFeeder(-0.25),
            () -> m_shooterSub.spinLowerFeeder(0.0), m_shooterSub));

    // m_operatorController.circle()
    m_operatorController.circle()
        .onTrue(new StartEndCommand(() -> m_shooterSub.spinLowerFeeder(0.25),
            () -> m_shooterSub.spinUpperFeeder(0.0), m_shooterSub));

    // m_operatorController.triangle()
    m_operatorController.triangle()
        .onTrue(new StartEndCommand(() -> m_shooterSub.spinUpperFeeder(0.25),
            () -> m_shooterSub.spinUpperFeeder(0.0), m_shooterSub));

    // m_operatorController.L1()
    m_operatorController.L1()
        .onTrue(new StartEndCommand(() -> m_climbSub.setClimbPowerLeft(0.25),
            () -> m_climbSub.setClimbPowerLeft(0.0), m_climbSub));

    // m_operatorController.R1()
    m_operatorController.R1()
        .onTrue(new StartEndCommand(() -> m_climbSub.setClimbPowerRight(0.25),
            () -> m_climbSub.setClimbPowerRight(0.0), m_climbSub));

    // m_operatorController.L2()
    m_operatorController.L2()
        .onTrue(new StartEndCommand(() -> m_climbSub.setClimbPowerLeft(-0.25),
            () -> m_climbSub.setClimbPowerLeft(0.0), m_climbSub));

    // m_operatorController.R2()
    m_operatorController.R2()
        .onTrue(new StartEndCommand(() -> m_climbSub.setClimbPowerRight(-0.25),
            () -> m_climbSub.setClimbPowerRight(0.0), m_climbSub));

    // m_operatorController.share()

    // m_operatorController.options()

    // m_operatorController.PS()

    // m_operatorController.touchpad()

    // m_operatorController.povUp()
    m_operatorController.povUp()
        .onTrue(new StartEndCommand(() -> m_intakeSub.setIntakeMotors(0.25),
            () -> m_intakeSub.setIntakeMotors(0.0), m_intakeSub));

    // m_operatorController.povRight()

    // m_operatorController.povDown()
    m_operatorController.povDown()
        .onTrue(new StartEndCommand(() -> m_intakeSub.setIntakeMotors(-0.25),
            () -> m_intakeSub.setIntakeMotors(0.0), m_intakeSub));

    // m_operatorController.povLeft()

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PathPlannerAuto("Test Auto"); // Takes in Auto file name
  }

  /*
   * public Command getTrajectoryCommand() { // Using trajectory library
   * Rotation2d currentRotation = m_drivetrainSub.getRotation();
   * // Example path (needs to be Pose2ds)
   * List<Pose2d> points = new ArrayList<Pose2d>();
   * points
   * .add(new Pose2d(0.0 + m_drivetrainSub.getPos().getX(), 0.0 + m_drivetrainSub.getPos().getY(),
   * currentRotation));
   * points
   * .add(new Pose2d(0.0 + m_drivetrainSub.getPos().getX(), 1.0 + m_drivetrainSub.getPos().getY(),
   * currentRotation));
   * points
   * .add(new Pose2d(1.0 + m_drivetrainSub.getPos().getX(), 1.0 + m_drivetrainSub.getPos().getY(),
   * currentRotation));
   * points
   * .add(new Pose2d(1.0 + m_drivetrainSub.getPos().getX(), 0.0 + m_drivetrainSub.getPos().getY(),
   * currentRotation));
   * points
   * .add(new Pose2d(0.0 + m_drivetrainSub.getPos().getX(), 0.0 + m_drivetrainSub.getPos().getY(),
   * currentRotation));
   * 
   * 
   * //SwerveDriveKinematicsConstraint kinematicsConstraint =
   * // new SwerveDriveKinematicsConstraint(m_kinematics, kMaxDriveSpeed); // Makes sure the trajectory isn't
   * calculated above max speed
   * TrajectoryConfig tConfig =
   * new TrajectoryConfig(DrivetrainSub.kMaxDriveSpeed, 10.0).setKinematics(m_drivetrainSub.m_kinematics);
   * 
   * Trajectory testTrajectory = TrajectoryGenerator.generateTrajectory(points, tConfig); // In meters
   * var thetaController =
   * new ProfiledPIDController(0.1, 0.0, 0.0, new TrapezoidProfile.Constraints(Math.PI, Math.PI)); // TODO: Get
   * real
   * values for this (max angular speed, max angular acceleration)
   * thetaController.enableContinuousInput(-Math.PI, Math.PI); // TODO: Find out if this should be continuous
   * SwerveControllerCommand swerveCommand = new SwerveControllerCommand(testTrajectory,
   * m_drivetrainSub::getOdometryPose2d,
   * m_drivetrainSub.m_kinematics, m_drivetrainSub.m_odometryPIDx, m_drivetrainSub.m_odometryPIDy,
   * thetaController,
   * m_drivetrainSub::driveStates, m_drivetrainSub);
   * 
   * m_drivetrainSub.resetOdometry(testTrajectory.getInitialPose());
   * 
   * return swerveCommand.andThen(() -> m_drivetrainSub.drive(0, 0, 0, 0.02));
   * }
   */

  // intialize the sub systems
  // TODO couple initialize need to be done
  public void initSubsystems() {
    // int sensorArray[] = new int[2];
    // sensorArray = m_shooterSub.RS232Listen(); ////////////////////////TODO Remove
    m_ledSub.init();
    m_climbSub.init();
    m_drivetrainSub.init();
    m_intakeSub.init();
    m_shooterSub.init();
    m_visionSub.init();

    if(DriverStation.getAlliance().isPresent()) {
      if(DriverStation.getAlliance().get() == Alliance.Red) {
        m_isRedAlliance = true;
      } else if(DriverStation.getAlliance().get() == Alliance.Blue) {
        m_isRedAlliance = false;
      }
    }
  }
}

