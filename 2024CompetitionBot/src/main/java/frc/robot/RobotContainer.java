// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlignVisionCmd;
import frc.robot.commands.ClimbSetHeightCmd;
import frc.robot.commands.DriveFieldRelativeCmd;
import frc.robot.commands.DrivePathCmd;
import frc.robot.commands.DriveToRelativePositionCmd;
import frc.robot.commands.KillAllCmd;
import frc.robot.commands.NoteIntakeGrp;
import frc.robot.commands.ShooterAmpShotGrp;
import frc.robot.commands.ShooterFlywheelCmd;
import frc.robot.commands.ShooterPivotCmd;
import frc.robot.commands.ShooterPrepGrp;
import frc.robot.commands.ShooterShootCmd;
import frc.robot.commands.ShooterWithJoystickCmd;
import frc.robot.commands.TestLedsCmd;
import frc.robot.commands.ZeroPivotNoFlywheelGrp;
import frc.robot.subsystems.ArduinoSub;
import frc.robot.subsystems.ClimbSub;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.FeederSub;
import frc.robot.subsystems.FlywheelSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.LedSub;
import frc.robot.subsystems.LedSub.LedColour;
import frc.robot.subsystems.LedSub.LedZones;
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
  private final ShooterSub m_shooterSub = new ShooterSub();
  private final VisionSub m_visionSub = new VisionSub();
  private final FlywheelSub m_flywheelSub = new FlywheelSub();
  private final FeederSub m_feederSub = new FeederSub();
  private final ArduinoSub m_arduinoSub = new ArduinoSub();

  private boolean m_isRedAlliance = true;

  private final CommandPS4Controller m_driverController =
      new CommandPS4Controller(OperatorConstants.kDriverControllerPort);
  private final CommandPS4Controller m_operatorController =
      new CommandPS4Controller(OperatorConstants.kOperatorControllerPort);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Subsystem default commands
    m_drivetrainSub.setDefaultCommand(
        new DriveFieldRelativeCmd(m_driverController, m_drivetrainSub));
    m_shooterSub.setDefaultCommand(
        new ShooterWithJoystickCmd(m_operatorController, m_shooterSub, m_feederSub, m_intakeSub));

    // Configure the button bindings
    configureBindings();

    // TODO: Add autonomous commands here
    NamedCommands.registerCommand("ExampleAuto", new PrintCommand("RUNNING A PATH COMMAND!"));

    // Put manual robot initialize button on SmartDashboard
    SmartDashboard.putData("RobotInit", new InstantCommand(() -> initSubsystems()));
  }


  /**
   * Use this method to define your trigger->command mappings.
   */
  private void configureBindings() {
    // ======================================== Driver controller bindings ========================================

    // This basically takes over the robot right now
    m_driverController.square()
        .whileTrue(new AlignVisionCmd(m_drivetrainSub, m_visionSub, m_shooterSub, m_feederSub, m_flywheelSub,
            m_driverController,
            m_operatorController));

    m_driverController.cross()
        .onTrue(new ClimbSetHeightCmd(Constants.Climb.kHeightHookLowered, 0.5,
            m_drivetrainSub,
            m_climbSub));

    m_driverController.circle()
        .onTrue(new ClimbSetHeightCmd(Constants.Climb.kHeightTallHookRaised, 0.5,
            m_drivetrainSub,
            m_climbSub));

    m_driverController.triangle()
        .onTrue(new ClimbSetHeightCmd(Constants.Climb.kHeightShortHookRaised, 0.5,
            m_drivetrainSub,
            m_climbSub));

    m_driverController.L1().onTrue(new InstantCommand(() -> m_arduinoSub.updateLED(9, 255, 0, 0)));

    m_driverController.R1().onTrue(new InstantCommand(() -> m_arduinoSub.updateLED(9, 0, 255, 0)));

    m_driverController.L2().onTrue(new InstantCommand(() -> m_arduinoSub.updateLED(9, 0, 0, 255)));

    m_driverController.R2().onTrue(new ShooterShootCmd(m_shooterSub, m_flywheelSub, m_feederSub));

    m_driverController.share()
        .onTrue(new InstantCommand(() -> m_drivetrainSub.resetGyro(), m_drivetrainSub));

    m_driverController.options().onTrue(new TestLedsCmd(m_ledSub, LedColour.YELLOW));

    m_driverController.PS().onTrue(new InstantCommand(() -> m_drivetrainSub.fun(), m_drivetrainSub));

    m_driverController.touchpad().onTrue(new TestLedsCmd(m_ledSub, LedColour.BLUE));

    m_driverController.povRight().onTrue(new DrivePathCmd(m_drivetrainSub));

    m_driverController.povUp()
        .onTrue(new DriveToRelativePositionCmd(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(90.0)), m_drivetrainSub));

    m_driverController.povDown()
        .onTrue(new DriveToRelativePositionCmd(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(-90.0)), m_drivetrainSub));

    m_driverController.povLeft()
        .onTrue(new DriveToRelativePositionCmd(new Pose2d(0.0, -1.0, Rotation2d.fromDegrees(0.0)), m_drivetrainSub));

    m_driverController.L3()
        .onTrue(new KillAllCmd(m_climbSub, m_drivetrainSub, m_intakeSub, m_shooterSub,
            m_flywheelSub));

    m_driverController.R3()
        .onTrue(new KillAllCmd(m_climbSub, m_drivetrainSub, m_intakeSub, m_shooterSub,
            m_flywheelSub));


    // ======================================== Operator controller bindings ========================================

    m_operatorController.square()
        .onTrue(new ShooterPrepGrp(Constants.Shooter.kAngleAutoLine, m_shooterSub,
            m_flywheelSub));

    m_operatorController.cross()
        .onTrue(new ShooterPrepGrp(Constants.Shooter.kAngleSubwooferSpeaker, m_shooterSub,
            m_flywheelSub));

    m_operatorController.circle()
        .onTrue(new ShooterPrepGrp(Constants.Shooter.kAnglePodium, m_shooterSub,
            m_flywheelSub));


    m_operatorController.triangle()
        .onTrue(new ZeroPivotNoFlywheelGrp(m_shooterSub, m_flywheelSub));

    //m_operatorController.triangle()

    m_operatorController.L1().onTrue(new ClimbSetHeightCmd(0, 0.2, m_drivetrainSub, m_climbSub));

    m_operatorController.R1().onTrue(new ClimbSetHeightCmd(0.5334, 0.2, m_drivetrainSub, m_climbSub));

    m_operatorController.L2().onTrue(new NoteIntakeGrp(m_intakeSub, m_shooterSub, m_feederSub));

    m_operatorController.R2().onTrue(new ShooterShootCmd(m_shooterSub, m_flywheelSub, m_feederSub));

    m_operatorController.share().onTrue(new ClimbSetHeightCmd(0.2286, 0.2, m_drivetrainSub, m_climbSub)); //228.6

    // m_operatorController.options()
    //     .onTrue(new ShooterPrepGrp(Constants.Shooter.kAngleWingLine, m_shooterSub,
    //         m_flywheelSub));
    m_operatorController.options().onTrue(new ShooterFlywheelCmd(m_flywheelSub));

    m_operatorController.PS().whileTrue(
        new StartEndCommand(() -> m_climbSub.setClimbPower(1.0, 1.0), () -> m_climbSub.setClimbPower(0.0, 0.0)));

    m_operatorController.touchpad().whileTrue(
        new StartEndCommand(() -> m_climbSub.setClimbPower(-1.0, -1.0), () -> m_climbSub.setClimbPower(0.0, 0.0)));

    m_operatorController.povUp().onTrue(new ShooterAmpShotGrp(m_shooterSub, m_feederSub));

    //m_operatorController.povRight()

    m_operatorController.povDown()
        .onTrue(new ShooterPrepGrp(Constants.Shooter.kAngleAmp, m_shooterSub, m_flywheelSub));

    //m_operatorController.povLeft()
    m_operatorController.povRight().onTrue(new ShooterPivotCmd(90.0, m_shooterSub));
    m_operatorController.povLeft().onTrue(new ShooterPivotCmd(180.0, m_shooterSub));

    m_operatorController.L3()
        .onTrue(new KillAllCmd(m_climbSub, m_drivetrainSub, m_intakeSub, m_shooterSub,
            m_flywheelSub));

    m_operatorController.R3()
        .onTrue(new KillAllCmd(m_climbSub, m_drivetrainSub, m_intakeSub, m_shooterSub,
            m_flywheelSub));
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

  public void disabledPeriodic() {

    if(m_shooterSub.isPivotAtForwardLimit()) {
      m_ledSub.setZoneColour(LedZones.DIAG_SHOOTERFWD_LIMIT, LedColour.GREEN);
    } else {
      m_ledSub.setZoneColour(LedZones.DIAG_SHOOTERFWD_LIMIT, LedColour.RED);
    }

    if(m_shooterSub.isPivotAtReverseLimit()) {
      m_ledSub.setZoneColour(LedZones.DIAG_SHOOTERREV_LIMIT, LedColour.GREEN);
    } else {
      m_ledSub.setZoneColour(LedZones.DIAG_SHOOTERREV_LIMIT, LedColour.RED);
    }

    if(m_climbSub.isLeftAtLimit()) {
      m_ledSub.setZoneColour(LedZones.DIAG_CLIMBL_LIMIT, LedColour.GREEN);
    } else {
      m_ledSub.setZoneColour(LedZones.DIAG_CLIMBL_LIMIT, LedColour.RED);
    }

    if(m_climbSub.isRightAtLimit()) {
      m_ledSub.setZoneColour(LedZones.DIAG_CLIMBR_LIMIT, LedColour.GREEN);
    } else {
      m_ledSub.setZoneColour(LedZones.DIAG_CLIMBR_LIMIT, LedColour.RED);
    }

    // if(m_shooterSub.getPivotAngle() > 0) {
    //   m_ledSub.setZoneRGB(LedZones.DIAG_SHOOTER_ENC, 0, (int) (m_shooterSub.getPivotAngle() / 24000.0 * 255.0), 0);
    // } else if(m_shooterSub.getPivotAngle() < 0) {
    //   m_ledSub.setZoneRGB(LedZones.DIAG_SHOOTER_ENC, (int) (m_shooterSub.getPivotAngle() / -20000.0 * 255.0), 0, 0);
    // } else {
    //   m_ledSub.setZoneRGB(LedZones.DIAG_SHOOTER_ENC, 0, 0, 0);
    // }

  }


  // Intialize the sub systems
  public void initSubsystems() {
    // int sensorArray[] = new int[2];
    // /* sensorArray = */ m_arduinoSub.RS232Listen(); ////////////////////////TODO: Unknown? Remove
    m_ledSub.init();
    m_climbSub.init();
    m_drivetrainSub.init();
    m_intakeSub.init();
    m_shooterSub.init();
    m_visionSub.init();
    m_flywheelSub.init();
    m_arduinoSub.init();

    if(DriverStation.getAlliance().isPresent()) {
      if(DriverStation.getAlliance().get() == Alliance.Red) {
        m_isRedAlliance = true;
      } else if(DriverStation.getAlliance().get() == Alliance.Blue) {
        m_isRedAlliance = false;
      }
      m_visionSub.setAlliance(m_isRedAlliance);
    }
  }
}

