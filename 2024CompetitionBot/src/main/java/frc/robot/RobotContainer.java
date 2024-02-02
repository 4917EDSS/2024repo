// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ShooterUpperFeederCmd;
import frc.robot.commands.ShooterFlywheelCmd;
import frc.robot.commands.ShooterPivotCmd;
import frc.robot.commands.TestLedsCmd;
import frc.robot.subsystems.ClimbSub;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.LedSub;
import frc.robot.subsystems.LedSub.LedColour;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.subsystems.VisionSub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.LedSub;
import frc.robot.subsystems.LedSub.LedColour;
import frc.robot.subsystems.LedSub.LedZones;
import frc.robot.subsystems.ShooterSub;
import frc.robot.commands.ClimbCmdSetHeightCmd;
import frc.robot.commands.DriveToRelativePositionCmd;
import frc.robot.commands.KillAllCmd;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems and commands are defined here...
        private final IntakeSub m_intakeSub = new IntakeSub();
        private final LedSub m_ledSub = new LedSub();
        private final VisionSub m_visionSub = new VisionSub();
        private final DrivetrainSub m_drivetrainSub = new DrivetrainSub();
        private final ShooterSub m_shooterSub = new ShooterSub();
        private final ClimbSub m_climbSub = new ClimbSub();

        // Replace with CommandPS4Controller or CommandJoystick if needed
        private final CommandPS4Controller m_driverController =
                        new CommandPS4Controller(OperatorConstants.kDriverControllerPort);
        private final CommandPS4Controller m_operatorController =
                        new CommandPS4Controller(OperatorConstants.kOperatorControllerPort);

        /** The container for the robot. Contains subsystems, OI devices, and commands. */
        public RobotContainer() {
                // Configure the trigger bindings
                configureBindings();
                m_visionSub.setPipeline(2); // Apriltag vision
                m_drivetrainSub.setDefaultCommand(
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                // Deadband is applied here because it causes problems for autos
                                new RunCommand(
                                                () -> m_drivetrainSub.driveHoldAngle(
                                                                (Math.abs(m_driverController.getLeftX()) < 0.07 ? 0.0
                                                                                : m_driverController.getLeftX()),
                                                                (Math.abs(m_driverController.getLeftY()) < 0.07 ? 0.0
                                                                                : -m_driverController.getLeftY()),
                                                                (Math.abs(m_driverController.getRightX()) < 0.07 ? 0.0
                                                                                : -m_driverController.getRightX()),
                                                                0.02), // this is the duration fo thh timestep the speeds should be applied to. Should probably be changed 
                                                m_drivetrainSub));
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

                //m_driverController.cross().onTrue(new PrintCommand("Cross Pressed!"));
                //m_driverController.cross().onTrue(new RunCommand(() -> m_drivetrainSub.resetRelativePos(), m_drivetrainSub));
                m_driverController.share()
                                .onTrue(new InstantCommand(() -> m_drivetrainSub.resetGyro(), m_drivetrainSub));
                m_driverController.povRight()
                                .onTrue(new DriveToRelativePositionCmd(m_drivetrainSub,
                                                new Pose2d(2.0, 0.0, Rotation2d.fromDegrees(90.0))));
                m_driverController.povLeft()
                                .onTrue(new DriveToRelativePositionCmd(m_drivetrainSub,
                                                new Pose2d(-2.0, 0.0, Rotation2d.fromDegrees(-90.0))));

                m_driverController.povUp()
                                .onTrue(new DriveToRelativePositionCmd(m_drivetrainSub,
                                                new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(90.0))));
                m_driverController.povDown()
                                .onTrue(new DriveToRelativePositionCmd(m_drivetrainSub,
                                                new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(-90.0))));


                //m_driverController.triangle().onTrue(LedColour.GREEN);

                //Change to operator controller
                m_driverController.touchpad().onTrue(new TestLedsCmd(m_ledSub, LedColour.BLUE));
                m_driverController.options().onTrue(new TestLedsCmd(m_ledSub, LedColour.YELLOW));

                //m_driverController.L2().onTrue(new PrintCommand("focus canning"));
                // m_driverController.R1().onTrue(new ShooterFlywheelCmd(m_shooterSub));
                //m_driverController.R2().onTrue(new ShooterPivotCmd(m_shooterSub));
                //m_driverController.R3().onTrue(new ShooterFeederCmd(m_shooterSub));

                //here we are making the climb
                m_driverController.cross()
                                .onTrue(new ClimbCmdSetHeightCmd(Constants.ClimbConstants.kHookLowered, 0.5,
                                                m_drivetrainSub,
                                                m_climbSub));
                m_driverController.circle()
                                .onTrue(new ClimbCmdSetHeightCmd(Constants.ClimbConstants.kTallHookRaised, 0.5,
                                                m_drivetrainSub,
                                                m_climbSub));
                m_driverController.triangle()
                                .onTrue(new ClimbCmdSetHeightCmd(Constants.ClimbConstants.kShortHookRaised, 0.5,
                                                m_drivetrainSub,
                                                m_climbSub));
                m_driverController.L3()
                                .onTrue(new KillAllCmd(m_climbSub, m_drivetrainSub, m_intakeSub, m_shooterSub));
                m_driverController.R3()
                                .onTrue(new KillAllCmd(m_climbSub, m_drivetrainSub, m_intakeSub, m_shooterSub));
                m_driverController.L2()
                                .whileTrue(
                                                new StartEndCommand(() -> m_climbSub.setClimbPowerLeft(-1.0),
                                                                () -> m_climbSub.setClimbPowerLeft(0.0)));
                m_driverController.L1()
                                .whileTrue(
                                                new StartEndCommand(() -> m_climbSub.setClimbPowerLeft(1.0),
                                                                () -> m_climbSub.setClimbPowerLeft(0.0)));
                m_driverController.R1()
                                .whileTrue(
                                                new StartEndCommand(() -> m_climbSub.setClimbPowerRight(1.0),
                                                                () -> m_climbSub.setClimbPowerRight(0.0)));
                m_driverController.R2()
                                .whileTrue(
                                                new StartEndCommand(() -> m_climbSub.setClimbPowerRight(-1.0),
                                                                () -> m_climbSub.setClimbPowerRight(0.0)));

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // An example command will be run in autonomous
                return new PrintCommand("No auto yet");
        }

        public void initSubsystems() {
                m_ledSub.init();
        }

        public void resetGyro() {
                m_drivetrainSub.resetGyro();
        }
}

