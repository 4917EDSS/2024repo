// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DrivetrainSub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  DrivetrainSub m_drivetrainSub = new DrivetrainSub();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS4Controller m_driverController =
      new CommandPS4Controller(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_drivetrainSub.setDefaultCommand(
        new InstantCommand(() -> m_drivetrainSub.setDrivePower(-m_driverController.getLeftY(), 0.05), m_drivetrainSub));

    // Configure the trigger bindings
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
    // Hold the wheels forward (if auto-rotate is enabled)
    m_driverController.square().onTrue(new InstantCommand(() -> m_drivetrainSub.setRotateMode(false)));
    // Hold the wheels in a circle so robot can rotate on itself (if auto-rotate is enabled)
    m_driverController.circle().onTrue(new InstantCommand(() -> m_drivetrainSub.setRotateMode(true)));
    // Drive forward full speed
    m_driverController.povUp().whileTrue(new StartEndCommand(() -> m_drivetrainSub.setDrivePower(1.0, 0),
        () -> m_drivetrainSub.setDrivePower(1.0, 0), m_drivetrainSub));
    // Drive backward full speed
    m_driverController.povDown().whileTrue(new StartEndCommand(() -> m_drivetrainSub.setDrivePower(-1.0, 0),
        () -> m_drivetrainSub.setDrivePower(-1.0, 0), m_drivetrainSub));
    // Drive forward at 50% speed
    m_driverController.povRight().whileTrue(new StartEndCommand(() -> m_drivetrainSub.setDrivePower(0.5, 0),
        () -> m_drivetrainSub.setDrivePower(0.5, 0), m_drivetrainSub));
    // Drive backward at 50% speed
    m_driverController.povLeft().whileTrue(new StartEndCommand(() -> m_drivetrainSub.setDrivePower(-0.5, 0),
        () -> m_drivetrainSub.setDrivePower(-0.5, 0), m_drivetrainSub));
    // Enable auto-rotate
    m_driverController.touchpad()
        .onTrue(new InstantCommand(() -> m_drivetrainSub.setAutoRotate(true), m_drivetrainSub));
    // Disable auto-rotate (so you can use the manual rotates below)
    m_driverController.PS().onTrue(new InstantCommand(() -> m_drivetrainSub.setAutoRotate(false), m_drivetrainSub));
    // Manually rotate all of the wheels forwards/counterclockwise (for testing)
    m_driverController.L1().whileTrue(new StartEndCommand(() -> m_drivetrainSub.setRotatePower(0.15),
        () -> m_drivetrainSub.setRotatePower(0.0), m_drivetrainSub));
    // Manually rotate all of the wheels backwards/clockwise (for testing)
    m_driverController.R1().whileTrue(new StartEndCommand(() -> m_drivetrainSub.setRotatePower(-0.15),
        () -> m_drivetrainSub.setRotatePower(0.0), m_drivetrainSub));
    // Reset the gyro
    m_driverController.share().onTrue(new InstantCommand(() -> m_drivetrainSub.resetGyro(), m_drivetrainSub));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PrintCommand("No Auto");
  }
}
