// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlignVisionGrp;
import frc.robot.commands.AmpShotPrepCmd;
import frc.robot.commands.ClimbSetHeightCmd;
import frc.robot.commands.DriveFieldRelativeCmd;
import frc.robot.commands.ExpelAmpNoteCmd;
import frc.robot.commands.FastIntakeNoteGrp;
import frc.robot.commands.FastShooterPrepGrp;
import frc.robot.commands.GameCmd;
import frc.robot.commands.IntakeNoteGrp;
import frc.robot.commands.IntakeWithJoystickCmd;
import frc.robot.commands.KillAllCmd;
import frc.robot.commands.NoteVisionAlignCmd;
import frc.robot.commands.PivotToAprilTagCmd;
import frc.robot.commands.ShooterFlywheelCmd;
import frc.robot.commands.ShooterIntakeGrp;
import frc.robot.commands.ShooterPivotCmd;
import frc.robot.commands.ShooterPrepGrp;
import frc.robot.commands.ShooterShootCmd;
import frc.robot.commands.PivotWithJoystickCmd;
import frc.robot.commands.QuickVisionAlignGrp;
import frc.robot.commands.ReintakeAmpNoteCmd;
import frc.robot.commands.SourceIntakeGrp;
import frc.robot.commands.ZeroPivotNoFlywheelGrp;
import frc.robot.subsystems.ArduinoSub;
import frc.robot.subsystems.ClimbSub;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.FeederSub;
import frc.robot.subsystems.FlywheelSub;
import frc.robot.subsystems.LedSub;
import frc.robot.subsystems.LedSub.LedColour;
import frc.robot.subsystems.LedSub.LedZones;
import frc.robot.subsystems.PowerSub;
import frc.robot.subsystems.PivotSub;
import frc.robot.subsystems.VisionSub;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ArduinoSub m_arduinoSub = new ArduinoSub();
  private final ClimbSub m_climbSub = new ClimbSub();
  private final DrivetrainSub m_drivetrainSub = new DrivetrainSub();
  private final FeederSub m_feederSub = new FeederSub();
  private final FlywheelSub m_flywheelSub = new FlywheelSub();
  private final LedSub m_ledSub = new LedSub(m_arduinoSub);
  private final PowerSub m_powerSub = new PowerSub();
  private final PivotSub m_pivotSub = new PivotSub();
  private final VisionSub m_visionSub = new VisionSub();

  // Disables large amount of prints from DrivetrainSub, pivotSub, PowerSub, and VisionSub
  // Fixes a lot of CommandLoop overruns from prints
  public static boolean disableShuffleboardPrint = true;

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
    m_pivotSub.setDefaultCommand(
        new PivotWithJoystickCmd(m_operatorController, m_pivotSub));
    m_feederSub.setDefaultCommand(
        new IntakeWithJoystickCmd(m_operatorController, m_feederSub));

    // Configure the button bindings
    configureBindings();

    // Add autonomous commands needed by Path Planner here
    NamedCommands.registerCommand("IntakeNoteGrp",
        new FastIntakeNoteGrp(m_arduinoSub, m_feederSub, m_flywheelSub, m_ledSub, m_pivotSub));
    NamedCommands.registerCommand("ShooterIntakeGrp",
        new ShooterIntakeGrp(m_feederSub, m_flywheelSub, 1.0));
    NamedCommands.registerCommand("ShooterShootCmd",
        new ShooterShootCmd(m_arduinoSub, m_feederSub, m_flywheelSub, m_ledSub));
    NamedCommands.registerCommand("ShooterPrepGrpTouchingSpeaker",
        new FastShooterPrepGrp(Constants.Shooter.kAngleSubwooferSpeaker, m_arduinoSub, m_feederSub, m_flywheelSub,
            m_pivotSub));
    NamedCommands.registerCommand("QuickVisionAlignGrp", new QuickVisionAlignGrp(m_drivetrainSub, m_feederSub,
        m_flywheelSub, m_ledSub, m_pivotSub, m_visionSub, m_arduinoSub));
    NamedCommands.registerCommand("ShooterPrepGrpFromStage",
        new FastShooterPrepGrp(56.25, m_arduinoSub, m_feederSub, m_flywheelSub, m_pivotSub)); // Was at 66.25 before vision
    NamedCommands.registerCommand("ShooterPrepGrpFromSpeaker",
        new FastShooterPrepGrp(50.5, m_arduinoSub, m_feederSub, m_flywheelSub, m_pivotSub)); // Was at 60.5 before vision
    NamedCommands.registerCommand("ShooterPrepGrpFromAmp",
        new FastShooterPrepGrp(64.25, m_arduinoSub, m_feederSub, m_flywheelSub, m_pivotSub));
    NamedCommands.registerCommand("ShooterPrepGrp10Degrees",
        new FastShooterPrepGrp(10, m_arduinoSub, m_feederSub, m_flywheelSub, m_pivotSub));
    NamedCommands.registerCommand("AmpShot",
        new ExpelAmpNoteCmd(m_feederSub, m_ledSub));
    NamedCommands.registerCommand("PivotToAprilTagCmd",
        new PivotToAprilTagCmd(m_pivotSub, m_visionSub)); //this command isFinished return false
    NamedCommands.registerCommand("ShooterFlywheelCmd",
        new ShooterFlywheelCmd(Constants.Flywheel.kFlywheelShootVelocity, m_flywheelSub));
    NamedCommands.registerCommand("OffsetYaw45",
        new InstantCommand(() -> m_drivetrainSub.resetGyroYaw(45), m_drivetrainSub));
    NamedCommands.registerCommand("VisionAlignCmdGrp",
        new AlignVisionGrp(m_driverController, m_operatorController, m_arduinoSub, m_drivetrainSub, m_feederSub,
            m_flywheelSub, m_ledSub, m_pivotSub, m_visionSub));
    NamedCommands.registerCommand("NoteVisionAlignCmd",
        new NoteVisionAlignCmd(m_visionSub, m_drivetrainSub));
    NamedCommands.registerCommand("ZeroPivot",
        new ShooterPivotCmd(0, m_pivotSub));

    autoChooserSetup();
  }


  /**
   * Use this method to define your trigger->command mappings.
   */
  private void configureBindings() {
    // ======================================== Driver controller bindings ========================================

    // This basically takes over the robot right now
    m_driverController.L1().whileTrue(new NoteVisionAlignCmd(m_visionSub, m_drivetrainSub));

    // m_driverController.square()

    // m_driverController.cross()

    //m_driverController.circle()

    //m_driverController.triangle()

    //m_driverController.L1()

    m_driverController.R1().onTrue(new ExpelAmpNoteCmd(m_feederSub, m_ledSub));

    m_driverController.L2()
        .onTrue(new AlignVisionGrp(m_driverController, m_operatorController, m_arduinoSub, m_drivetrainSub, m_feederSub,
            m_flywheelSub, m_ledSub, m_pivotSub, m_visionSub));

    m_driverController.R2()
        .onTrue(new ShooterShootCmd(m_arduinoSub, m_feederSub, m_flywheelSub, m_ledSub));

    m_driverController.share()
        .onTrue(new InstantCommand(() -> m_drivetrainSub.resetGyroYaw(0), m_drivetrainSub));

    //m_driverController.options()

    m_driverController.PS().onTrue(new InstantCommand(() -> m_drivetrainSub.fun(), m_drivetrainSub)); // Will only actually run in test mode

    m_driverController.touchpad().onTrue(new GameCmd(m_driverController, m_arduinoSub)); // Will only actually run in test mode

    //m_driverController.povUp()

    //m_driverController.povRight()

    //m_driverController.povDown()

    //m_driverController.povLeft()

    m_driverController.L3()
        .onTrue(new KillAllCmd(m_climbSub, m_drivetrainSub, m_feederSub, m_flywheelSub, m_pivotSub));

    m_driverController.R3()
        .onTrue(new KillAllCmd(m_climbSub, m_drivetrainSub, m_feederSub, m_flywheelSub, m_pivotSub));


    // ======================================== Operator controller bindings ========================================
    m_operatorController.square()
        .onTrue(new ShooterPrepGrp(Constants.Shooter.kAngleAutoLine, Constants.Flywheel.kFlywheelShootVelocity,
            m_arduinoSub, m_feederSub, m_flywheelSub,
            m_pivotSub, m_ledSub));

    m_operatorController.cross()
        .onTrue(new ShooterPrepGrp(Constants.Shooter.kAngleSubwooferSpeaker, Constants.Flywheel.kFlywheelShootVelocity,
            m_arduinoSub, m_feederSub, m_flywheelSub,
            m_pivotSub, m_ledSub));

    m_operatorController.circle()
        .onTrue(new SourceIntakeGrp(m_arduinoSub, m_feederSub, m_flywheelSub, m_ledSub, m_pivotSub));

    m_operatorController.triangle()
        .onTrue(new ZeroPivotNoFlywheelGrp(m_arduinoSub, m_feederSub, m_flywheelSub, m_ledSub, m_pivotSub));

    m_operatorController.L1()
        .onTrue(new ClimbSetHeightCmd(Constants.Climb.kHeightHookLowered, 1.0, m_climbSub, m_drivetrainSub));

    m_operatorController.R1()
        .onTrue(new ClimbSetHeightCmd(Constants.Climb.kHeightShortHookRaised, 1.0, m_climbSub, m_drivetrainSub));

    m_operatorController.L2()
        .onTrue(new IntakeNoteGrp(m_arduinoSub, m_feederSub, m_flywheelSub, m_ledSub, m_pivotSub));

    m_operatorController.R2()
        .onTrue(new ShooterShootCmd(m_arduinoSub, m_feederSub, m_flywheelSub, m_ledSub));

    m_operatorController.share()
        .onTrue(new ClimbSetHeightCmd(Constants.Climb.kHeightTallHookRaised, 1.0, m_climbSub, m_drivetrainSub));

    m_operatorController.options()
        .onTrue(new ShooterFlywheelCmd(Constants.Flywheel.kFlywheelShootVelocity, m_flywheelSub));

    m_operatorController.PS().whileTrue(
        new StartEndCommand(() -> m_climbSub.setClimbPower(-1.0, -1.0), () -> m_climbSub.setClimbPower(0.0, 0.0)));

    m_operatorController.touchpad().whileTrue(
        new StartEndCommand(() -> m_climbSub.setClimbPower(1.0, 1.0), () -> m_climbSub.setClimbPower(0.0, 0.0)));

    m_operatorController.povUp().onTrue(new ExpelAmpNoteCmd(m_feederSub, m_ledSub));

    m_operatorController.povRight()
        .onTrue(new ParallelCommandGroup(new ReintakeAmpNoteCmd(m_arduinoSub, m_feederSub),
            new ShooterPivotCmd(60.0, m_pivotSub), new InstantCommand(
                () -> m_flywheelSub.enableFlywheel(Constants.Flywheel.kFlywheelShootVelocity), m_flywheelSub)));


    m_operatorController.povDown()
        .onTrue(new SequentialCommandGroup(
            new ParallelCommandGroup(new InstantCommand(() -> m_flywheelSub.disableFlywheel()),
                new ShooterPivotCmd(227.0, m_pivotSub),
                new SequentialCommandGroup(new WaitCommand(0.3),
                    new AmpShotPrepCmd(m_arduinoSub, m_feederSub))),
            new InstantCommand(() -> m_ledSub.setZoneColour(LedZones.ALL, LedColour.GREEN))));

    m_operatorController.povLeft().onTrue(
        new ShooterPrepGrp(Constants.Shooter.kAnglePassing, Constants.Flywheel.kFlywheelLobVelocity, m_arduinoSub,
            m_feederSub, m_flywheelSub, m_pivotSub, m_ledSub));

    m_operatorController.L3()
        .onTrue(new KillAllCmd(m_climbSub, m_drivetrainSub, m_feederSub, m_flywheelSub, m_pivotSub));

    m_operatorController.R3()
        .onTrue(new KillAllCmd(m_climbSub, m_drivetrainSub, m_feederSub, m_flywheelSub, m_pivotSub));
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
    m_Chooser.addOption("4NoteAuto", new PathPlannerAuto("4NoteAuto"));
    m_Chooser.addOption("With Vision 4NoteAuto", new PathPlannerAuto("With Vision 4NoteAuto"));
    m_Chooser.addOption("3NoteAuto Leave", new PathPlannerAuto("3NoteAuto Leave"));
    m_Chooser.addOption("Default Auto", new PathPlannerAuto("Default Auto"));
    m_Chooser.addOption("Shoot and Leave Auto", new PathPlannerAuto("Shoot and Leave Auto"));
    m_Chooser.addOption("No Vision Shoot and Leave Auto", new PathPlannerAuto("No Vision Shoot and Leave Auto"));
    m_Chooser.addOption("2 Far Notes Under Stage", new PathPlannerAuto("2 Far Notes Under Stage"));
    m_Chooser.addOption("1meterAuto", new PathPlannerAuto("1meterAuto"));
    m_Chooser.addOption("StealBecauseWeAreMean", new PathPlannerAuto("StealBecauseWeAreMean"));

    SmartDashboard.putData("auto choices", m_Chooser);
  }

  public void setLEDs(double encoderValue, LedZones zone) {
    if(encoderValue < -Math.PI / 3.0) {
      encoderValue += Math.PI / 3.0;
      encoderValue /= 2.0 / 3.0 * Math.PI;
      encoderValue *= -255.0;
      int intEncoderValue = (int) (encoderValue);
      m_ledSub.setZoneRGB(zone, intEncoderValue, 255 - intEncoderValue, 0);
    } else if(encoderValue > Math.PI / 3.0) {
      encoderValue -= Math.PI / 3.0;
      encoderValue /= 2.0 / 3.0 * Math.PI;
      encoderValue *= 255.0;
      int intEncoderValue = (int) (encoderValue);
      m_ledSub.setZoneRGB(zone, intEncoderValue, 0, 255 - intEncoderValue);
    } else {
      encoderValue += Math.PI / 3.0;
      encoderValue /= 2.0 / 3.0 * Math.PI;
      encoderValue *= 255.0;
      int intEncoderValue = (int) (encoderValue);
      m_ledSub.setZoneRGB(zone, 0, 255 - intEncoderValue, intEncoderValue);
    }
  }


  public void disabledPeriodic() {

    if(m_pivotSub.isPivotAtForwardLimit()) {
      m_ledSub.setZoneColour(LedZones.DIAG_SHOOTERFWD_LIMIT, LedColour.GREEN);
    } else {
      m_ledSub.setZoneColour(LedZones.DIAG_SHOOTERFWD_LIMIT, LedColour.RED);
    }

    if(m_pivotSub.isPivotAtReverseLimit()) {
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

    if(m_pivotSub.getPivotAngle() < 2) {
      m_ledSub.setZoneColour(LedZones.DIAG_PIVOT_ENC, LedColour.GREEN);
    } else if(m_pivotSub.getPivotAngle() < 10) {
      m_ledSub.setZoneColour(LedZones.DIAG_PIVOT_ENC, LedColour.BLUE);
    } else if(m_pivotSub.getPivotAngle() < 40) {
      m_ledSub.setZoneColour(LedZones.DIAG_PIVOT_ENC, LedColour.YELLOW);
    } else if(m_pivotSub.getPivotAngle() < 270) {
      m_ledSub.setZoneColour(LedZones.DIAG_PIVOT_ENC, LedColour.ORANGE);
    } else {
      m_ledSub.setZoneColour(LedZones.DIAG_PIVOT_ENC, LedColour.RED);
    }

    double FL = (m_drivetrainSub.getTurningEncoderFL());
    double FR = (m_drivetrainSub.getTurningEncoderFR());
    double BL = (m_drivetrainSub.getTurningEncoderBL());
    double BR = (m_drivetrainSub.getTurningEncoderBR());


    setLEDs(FL, LedZones.DIAG_FL_STEERING_ENC);
    setLEDs(FR, LedZones.DIAG_FR_STEERING_ENC);
    setLEDs(BL, LedZones.DIAG_BL_STEERING_ENC);
    setLEDs(BR, LedZones.DIAG_BR_STEERING_ENC);

    /*
     * m_ledSub.setZoneRGB(LedZones.DIAG_FL_STEERING_ENC, FL % 2 == 0 ? 255 : 0, FL % 3 == 0 ? 255 : 0,
     * FL % 5 == 0 ? 255 : 0); //
     * 
     * m_ledSub.setZoneRGB(LedZones.DIAG_FR_STEERING_ENC, FR % 2 == 0 ? 255 : 0, FR % 3 == 0 ? 255 : 0,
     * FR % 5 == 0 ? 255 : 0); //
     * 
     * m_ledSub.setZoneRGB(LedZones.DIAG_BL_STEERING_ENC, BL % 2 == 0 ? 255 : 0, BL % 3 == 0 ? 255 : 0,
     * BL % 5 == 0 ? 255 : 0); //
     * 
     * m_ledSub.setZoneRGB(LedZones.DIAG_BR_STEERING_ENC, BR % 2 == 0 ? 255 : 0, BR % 3 == 0 ? 255 : 0,
     * BR % 5 == 0 ? 255 : 0); //
     */
    // m_ledSub.setZoneRGB(LedZones.DIAG_FR_STEERING_ENC,
    //     (int) (Math.abs(m_drivetrainSub.getTurningEncoderFR()) / Math.PI * 255.0), 0, 0); //

    // m_ledSub.setZoneRGB(LedZones.DIAG_BL_STEERING_ENC,
    //     (int) (Math.abs(m_drivetrainSub.getTurningEncoderBL()) / Math.PI * 255.0), 0, 0); //

    // m_ledSub.setZoneRGB(LedZones.DIAG_BR_STEERING_ENC,
    //     (int) (Math.abs(m_drivetrainSub.getTurningEncoderBR()) / Math.PI * 255.0), 0, 0); //


    // } else if(m_pivotSub.getPivotAngle() < 0) {
    //   m_ledSub.setZoneRGB(LedZones.DIAG_SHOOTER_ENC, (int) (m_pivotSub.getPivotAngle() / -20000.0 * 255.0), 0, 0);
    // } else {
    //   m_ledSub.setZoneRGB(LedZones.DIAG_SHOOTER_ENC, 0, 0, 0);
    // }

  }


  // Intialize the sub systems
  public void initSubsystems() {
    m_ledSub.init();
    m_climbSub.init();
    m_drivetrainSub.init();
    m_pivotSub.init();
    m_visionSub.init();
    m_flywheelSub.init();
    m_arduinoSub.init();
    m_powerSub.init();
  }

  public void testInitSubsystems() {
    m_arduinoSub.init();
  }

  public void disableTest() {
    m_drivetrainSub.nofun();
  }

  public void postAutoInit() {
    m_drivetrainSub.postAutoResetYaw();
  }
}

