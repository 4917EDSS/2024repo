// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveFieldRelativeCmd;
import frc.robot.commands.DriveToRelativePositionCmd;
import frc.robot.subsystems.DrivetrainSub;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  private final DrivetrainSub m_drivetrainSub = new DrivetrainSub();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS4Controller m_driverController =
      new CommandPS4Controller(OperatorConstants.kDriverControllerPort);

  private final Pose2d firstPos = new Pose2d(0.0, -1.5, new Rotation2d(0.0));
  private final Pose2d secondPos = new Pose2d(-1.5, 0.0, new Rotation2d(0.0));
  private final Pose2d thirdPos = new Pose2d(0.0, 1.5, new Rotation2d(0.0));
  private final Pose2d fourthPos = new Pose2d(1.5, 0.0, new Rotation2d(0.0));

  private final Pose2d firstDiagonalPos = new Pose2d(-1.5, -3, new Rotation2d(180.0));
  private final Pose2d secondDiagonalPos = new Pose2d(1.5, 3, new Rotation2d(180.0));


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_drivetrainSub.setDefaultCommand(
        new DriveFieldRelativeCmd(m_driverController, m_drivetrainSub));
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. .
   */
  private void configureBindings() {
    m_driverController.cross()
        .whileTrue(new StartEndCommand(() -> m_drivetrainSub.runMotor(1.0), () -> m_drivetrainSub.runMotor(0.0),
            m_drivetrainSub));

    m_driverController.share()
        .onTrue(new InstantCommand(() -> m_drivetrainSub.resetGyroYaw(0), m_drivetrainSub));

    m_driverController.povUp()
        .onTrue(new SequentialCommandGroup(new DriveToRelativePositionCmd(firstPos, m_drivetrainSub),
            new DriveToRelativePositionCmd(secondPos, m_drivetrainSub),
            new DriveToRelativePositionCmd(thirdPos, m_drivetrainSub),
            new DriveToRelativePositionCmd(fourthPos, m_drivetrainSub)));

    m_driverController.povDown()
        .onTrue(new SequentialCommandGroup(new DriveToRelativePositionCmd(firstDiagonalPos, m_drivetrainSub),
            new DriveToRelativePositionCmd(secondDiagonalPos, m_drivetrainSub)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PrintCommand("No Autos");
  }
}
