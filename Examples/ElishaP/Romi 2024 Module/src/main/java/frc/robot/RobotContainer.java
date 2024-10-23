// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DrivetrainSub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.Encoder;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandPS4Controller m_driverController =
      new CommandPS4Controller(OperatorConstants.kDriverControllerPort);
  // The robot's subsystems and commands are defined here...

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_drivetrainSub.setDefaultCommand(new RunCommand(
        () -> m_drivetrainSub.drive(-m_driverController.getLeftY(), -m_driverController.getRightY()),
        m_drivetrainSub));

    // Configure the button bindings
    configureButtonBindings();
  }

  private final DrivetrainSub m_drivetrainSub = new DrivetrainSub();

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_driverController.triangle().onTrue(new PrintCommand("Triangle Pressed!"));
    m_driverController.square().onTrue(new PrintCommand("Square Pressed!"));
    m_driverController.cross().onTrue(new PrintCommand("Cross Pressed!"));
    m_driverController.circle().onTrue(new PrintCommand("Circle Pressed!"));
    m_driverController.cross().whileTrue(new StartEndCommand(() -> m_drivetrainSub.drive(0.25, -0.25),
        () -> m_drivetrainSub.drive(0.0, 0.0), m_drivetrainSub));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new PrintCommand("No auto");
  }
}
