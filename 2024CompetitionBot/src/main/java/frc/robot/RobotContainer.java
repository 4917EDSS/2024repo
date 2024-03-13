// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlignVisionGrp;
import frc.robot.commands.BackSpinIntakeCmd;
import frc.robot.commands.ClimbSetHeightCmd;
import frc.robot.commands.DriveFieldRelativeCmd;
import frc.robot.commands.DrivePathCmd;
import frc.robot.commands.DriveToRelativePositionCmd;
import frc.robot.commands.IntakeNoteGrp;
import frc.robot.commands.IntakeUntilNoteInCmd;
import frc.robot.commands.KillAllCmd;
import frc.robot.commands.PivotToAprilTagCmd;
import frc.robot.commands.ShooterAmpShotCmd;
import frc.robot.commands.ShooterAmpShotGrp;
import frc.robot.commands.ShooterFlywheelCmd;
import frc.robot.commands.ShooterPivotCmd;
import frc.robot.commands.ShooterPrepGrp;
import frc.robot.commands.ShooterShootCmd;
import frc.robot.commands.ShooterWithJoystickCmd;
import frc.robot.commands.TestLedsCmd;
import frc.robot.commands.UpIntakeGrp;
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
import frc.robot.subsystems.PowerSub;
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
  private final PowerSub m_powerSub = new PowerSub();

  private boolean m_isRedAlliance = true;

  // Disables large amount of prints from DrivetrainSub, ShooterSub, PowerSub, and VisionSub
  // Fixes a lot of CommandLoop overruns from prints
  public static boolean disableShuffleboardPrint = false;

  private final CommandPS4Controller m_driverController =
      new CommandPS4Controller(OperatorConstants.kDriverControllerPort);
  private final CommandPS4Controller m_operatorController =
      new CommandPS4Controller(OperatorConstants.kOperatorControllerPort);

  SendableChooser<Command> m_Chooser = new SendableChooser<>();


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
    NamedCommands.registerCommand("IntakeNoteGrp",
        new IntakeNoteGrp(m_shooterSub, m_intakeSub, m_feederSub, m_arduinoSub, m_ledSub));
    NamedCommands.registerCommand("ShooterShootCmd",
        new ShooterShootCmd(m_flywheelSub, m_feederSub, m_arduinoSub, m_shooterSub, m_ledSub));
    NamedCommands.registerCommand("ShooterPrepGrpTouchingSpeaker",
        new ShooterPrepGrp(Constants.Shooter.kAngleSubwooferSpeaker, m_shooterSub, m_flywheelSub, m_feederSub,
            m_arduinoSub));
    NamedCommands.registerCommand("ShooterPrepGrpFromStage",
        new ShooterPrepGrp(Constants.Shooter.kAngleAutoLine + 3, m_shooterSub, m_flywheelSub, m_feederSub,
            m_arduinoSub));
    NamedCommands.registerCommand("ShooterPrepGrpFromSpeaker",
        new ShooterPrepGrp(Constants.Shooter.kAngleAutoLine + 2, m_shooterSub, m_flywheelSub, m_feederSub,
            m_arduinoSub));
    NamedCommands.registerCommand("ShooterPrepGrpFromAmp",
        new ShooterPrepGrp(Constants.Shooter.kAngleAutoLine + 3, m_shooterSub, m_flywheelSub, m_feederSub,
            m_arduinoSub));
    NamedCommands.registerCommand("AmpShot",
        new ShooterAmpShotGrp(m_shooterSub, m_feederSub, m_arduinoSub, m_ledSub));
    NamedCommands.registerCommand("PivotToAprilTagCmd", new PivotToAprilTagCmd(m_visionSub, m_shooterSub)); //this command isFinished return false
    NamedCommands.registerCommand("ShooterFlywheelCmd",
        new ShooterFlywheelCmd(m_flywheelSub));
    NamedCommands.registerCommand("OffsetYaw45",
        new InstantCommand(() -> m_drivetrainSub.resetGyroYaw(45), m_drivetrainSub));

    // Put manual robot initialize button on SmartDashboard
    SmartDashboard.putData("RobotInit", new InstantCommand(() -> initSubsystems()));

    autoChooserSetup();
  }


  /**
   * Use this method to define your trigger->command mappings.
   */
  private void configureBindings() {
    // ======================================== Driver controller bindings ========================================

    // This basically takes over the robot right now
    m_driverController.square()
        .onTrue(new AlignVisionGrp(m_drivetrainSub, m_visionSub, m_shooterSub, m_feederSub, m_flywheelSub, m_ledSub,
            m_driverController, m_operatorController, m_arduinoSub, m_intakeSub));
    //m_driverController.cross()

    //m_driverController.circle()

    //m_driverController.triangle()

    m_driverController.L1().onTrue(new InstantCommand(() -> m_arduinoSub.updateLED(9, 255, 0, 0))); // TODO: Remove this test code

    m_driverController.R1().onTrue(new InstantCommand(() -> m_arduinoSub.updateLED(9, 0, 255, 0))); // TODO: Remove this test code

    m_driverController.L2().onTrue(new InstantCommand(() -> m_arduinoSub.updateLED(9, 0, 0, 255))); // TODO: Remove this test code

    m_driverController.R2()
        .onTrue(new ShooterShootCmd(m_flywheelSub, m_feederSub, m_arduinoSub, m_shooterSub, m_ledSub));

    m_driverController.share()
        .onTrue(new InstantCommand(() -> m_drivetrainSub.resetGyroYaw(0), m_drivetrainSub));

    //m_driverController.options()
    m_driverController.PS().onTrue(new InstantCommand(() -> m_drivetrainSub.fun(), m_drivetrainSub));

    m_driverController.touchpad().onTrue(new TestLedsCmd(m_ledSub, LedColour.BLUE)); // TODO: Remove this test code

    m_driverController.povRight().onTrue(new DrivePathCmd(m_drivetrainSub)); // TODO: Remove this test code

    m_driverController.povUp()
        .onTrue(new DriveToRelativePositionCmd(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(90.0)), m_drivetrainSub)); // TODO: Remove this test code

    m_driverController.povDown()
        .onTrue(new DriveToRelativePositionCmd(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(-90.0)), m_drivetrainSub)); // TODO: Remove this test code

    m_driverController.povLeft()
        .onTrue(new DriveToRelativePositionCmd(new Pose2d(0.0, -1.0, Rotation2d.fromDegrees(0.0)), m_drivetrainSub)); // TODO: Remove this test code

    m_driverController.L3()
        .onTrue(new KillAllCmd(m_climbSub, m_drivetrainSub, m_intakeSub, m_feederSub, m_shooterSub, m_flywheelSub));

    m_driverController.R3()
        .onTrue(new KillAllCmd(m_climbSub, m_drivetrainSub, m_intakeSub, m_feederSub, m_shooterSub, m_flywheelSub));


    // ======================================== Operator controller bindings ========================================
    m_operatorController.square()
        .onTrue(new ShooterPrepGrp(Constants.Shooter.kAngleAutoLine, m_shooterSub, m_flywheelSub, m_feederSub,
            m_arduinoSub));

    m_operatorController.cross()
        .onTrue(new ShooterPrepGrp(Constants.Shooter.kAngleSubwooferSpeaker, m_shooterSub, m_flywheelSub, m_feederSub,
            m_arduinoSub));

    m_operatorController.circle()
        .onTrue(new UpIntakeGrp(m_shooterSub, m_feederSub, m_arduinoSub, m_ledSub, m_intakeSub));

    m_operatorController.triangle()
        .onTrue(
            new ZeroPivotNoFlywheelGrp(m_shooterSub, m_flywheelSub, m_feederSub, m_intakeSub, m_arduinoSub, m_ledSub));

    m_operatorController.L1()
        .onTrue(new ClimbSetHeightCmd(Constants.Climb.kHeightHookLowered, 1.0, m_drivetrainSub, m_climbSub));

    m_operatorController.R1()
        .onTrue(new ClimbSetHeightCmd(Constants.Climb.kHeightShortHookRaised, 1.0, m_drivetrainSub, m_climbSub));

    m_operatorController.L2().onTrue(new IntakeNoteGrp(m_shooterSub, m_intakeSub, m_feederSub, m_arduinoSub, m_ledSub));

    m_operatorController.R2()
        .onTrue(new ShooterShootCmd(m_flywheelSub, m_feederSub, m_arduinoSub, m_shooterSub, m_ledSub));

    m_operatorController.share()
        .onTrue(new ClimbSetHeightCmd(Constants.Climb.kHeightTallHookRaised, 1.0, m_drivetrainSub, m_climbSub)); //228.6

    m_operatorController.options().onTrue(new ShooterFlywheelCmd(m_flywheelSub));//ShooterPrepGrp(Constants.Shooter.kAnglePassing, m_shooterSub,
    //m_flywheelSub));

    m_operatorController.PS().whileTrue(
        new StartEndCommand(() -> m_climbSub.setClimbPower(1.0, 1.0), () -> m_climbSub.setClimbPower(0.0, 0.0)));

    m_operatorController.touchpad().whileTrue(
        new StartEndCommand(() -> m_climbSub.setClimbPower(-1.0, -1.0), () -> m_climbSub.setClimbPower(0.0, 0.0)));

    m_operatorController.povUp().onTrue(new ShooterAmpShotGrp(m_shooterSub, m_feederSub, m_arduinoSub, m_ledSub));

    //m_operatorController.povRight()

    m_operatorController.povDown()
        .onTrue(new ShooterPivotCmd(Constants.Shooter.kAnglePreAmp, m_shooterSub));

    m_operatorController.povLeft().onTrue(new ShooterPivotCmd(Constants.Shooter.kAngleSourceIntake, m_shooterSub));

    m_operatorController.L3()
        .onTrue(new KillAllCmd(m_climbSub, m_drivetrainSub, m_intakeSub, m_feederSub, m_shooterSub, m_flywheelSub));

    m_operatorController.R3()
        .onTrue(new KillAllCmd(m_climbSub, m_drivetrainSub, m_intakeSub, m_feederSub, m_shooterSub, m_flywheelSub));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return new PathPlannerAuto("JustRunAuto"); // Takes in Auto file name
    return m_Chooser.getSelected();
  }

  void autoChooserSetup() {
    m_Chooser.addOption("Test JustRunAuto", new PathPlannerAuto("Test JustRunAuto"));
    m_Chooser.addOption("Test New Auto", new PathPlannerAuto("Test New Auto"));
    m_Chooser.addOption("Test Demo Auto", new PathPlannerAuto("Test Demo Auto"));
    m_Chooser.addOption("Five Note Score Auto", new PathPlannerAuto("Five Note Score Auto"));
    m_Chooser.addOption("5NoteStealAuto", new PathPlannerAuto("5NoteStealAuto"));
    m_Chooser.addOption("4NoteAuto", new PathPlannerAuto("4NoteAuto"));
    m_Chooser.addOption("2 Note Steal and Score", new PathPlannerAuto("2 Note Steal and Score"));
    m_Chooser.addOption("2 Amp 2 Speaker Auto", new PathPlannerAuto("2 Amp 2 Speaker Auto"));

    SmartDashboard.putData("auto choices", m_Chooser);
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

    int FL = (int) (Math.abs(m_drivetrainSub.getTurningEncoderFL()) / Math.PI * 255.0);
    int FR = (int) (Math.abs(m_drivetrainSub.getTurningEncoderFR()) / Math.PI * 255.0);
    int BL = (int) (Math.abs(m_drivetrainSub.getTurningEncoderBL()) / Math.PI * 255.0);
    int BR = (int) (Math.abs(m_drivetrainSub.getTurningEncoderBR()) / Math.PI * 255.0);
    
        
    m_ledSub.setZoneRGB(LedZones.DIAG_PIVOT_ENC, 0, (int) (m_shooterSub.getPivotAngle() / 50 * 255.0), 0); //

    m_ledSub.setZoneRGB(LedZones.DIAG_FL_STEERING_ENC, FL % 2 == 0 ? 255 : 0,  FL % 3 ==  0? 255 :0, FL % 5 == 0 ? 255 : 0); //

    m_ledSub.setZoneRGB(LedZones.DIAG_FR_STEERING_ENC, FR % 2 == 0 ? 255 : 0,  FR % 3 ==  0? 255 :0, FR % 5 == 0 ? 255 : 0); //

    m_ledSub.setZoneRGB(LedZones.DIAG_BL_STEERING_ENC, BL % 2 == 0 ? 255 : 0,  BL % 3 ==  0? 255 :0, BL % 5 == 0 ? 255 : 0); //

    m_ledSub.setZoneRGB(LedZones.DIAG_BR_STEERING_ENC, BR % 2 == 0 ? 255 : 0,  BR % 3 ==  0? 255 :0, BR % 5 == 0 ? 255 : 0); //

    // m_ledSub.setZoneRGB(LedZones.DIAG_FR_STEERING_ENC,
    //     (int) (Math.abs(m_drivetrainSub.getTurningEncoderFR()) / Math.PI * 255.0), 0, 0); //

    // m_ledSub.setZoneRGB(LedZones.DIAG_BL_STEERING_ENC,
    //     (int) (Math.abs(m_drivetrainSub.getTurningEncoderBL()) / Math.PI * 255.0), 0, 0); //

    // m_ledSub.setZoneRGB(LedZones.DIAG_BR_STEERING_ENC,
    //     (int) (Math.abs(m_drivetrainSub.getTurningEncoderBR()) / Math.PI * 255.0), 0, 0); //


    // } else if(m_shooterSub.getPivotAngle() < 0) {
    //   m_ledSub.setZoneRGB(LedZones.DIAG_SHOOTER_ENC, (int) (m_shooterSub.getPivotAngle() / -20000.0 * 255.0), 0, 0);
    // } else {
    //   m_ledSub.setZoneRGB(LedZones.DIAG_SHOOTER_ENC, 0, 0, 0);
    // }

  }


  // Intialize the sub systems
  public void initSubsystems() {
    m_ledSub.init();
    m_climbSub.init();
    m_drivetrainSub.init();
    m_intakeSub.init();
    m_shooterSub.init();
    m_visionSub.init();
    m_flywheelSub.init();
    m_arduinoSub.init();
    m_powerSub.init();

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

