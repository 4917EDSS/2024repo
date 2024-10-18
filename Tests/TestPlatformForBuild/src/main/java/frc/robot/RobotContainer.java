// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.JoystickCmd;
import frc.robot.subsystems.MotorControlSub;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final MotorControlSub m_motorControlSub = new MotorControlSub();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS4Controller m_driverController =
      new CommandPS4Controller(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_motorControlSub.setDefaultCommand(new JoystickCmd(m_driverController, m_motorControlSub));
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

    m_driverController.share().onTrue(new InstantCommand(() -> m_motorControlSub.setPowerMotor1(0.0)));

    m_driverController.povUp().onTrue(new InstantCommand(() -> m_motorControlSub.setPowerMotor1(1.0)));

    m_driverController.povDown().onTrue(new InstantCommand(() -> m_motorControlSub.setPowerMotor1(-1.0)));

    m_driverController.povRight().onTrue(new InstantCommand(() -> m_motorControlSub.setPowerMotor1(0.5)));

    m_driverController.povLeft().onTrue(new InstantCommand(() -> m_motorControlSub.setPowerMotor1(-0.5)));

    m_driverController.L1()
        .onTrue(new InstantCommand(() -> m_motorControlSub.setPowerMotor1(m_motorControlSub.m_motor1Power + 0.05)));

    m_driverController.L2()
        .onTrue(new InstantCommand(() -> m_motorControlSub.setPowerMotor1(m_motorControlSub.m_motor1Power - 0.05)));

    m_driverController.L3()
        .onTrue(new ParallelCommandGroup(new InstantCommand(() -> m_motorControlSub.setPowerMotor1(0)),
            new InstantCommand(() -> m_motorControlSub.setPowerMotor2(0))));

    m_driverController.options().onTrue(new InstantCommand(() -> m_motorControlSub.setPowerMotor2(0.0)));

    m_driverController.triangle().onTrue(new InstantCommand(() -> m_motorControlSub.setPowerMotor2(1.0)));

    m_driverController.cross().onTrue(new InstantCommand(() -> m_motorControlSub.setPowerMotor2(-1.0)));

    m_driverController.circle().onTrue(new InstantCommand(() -> m_motorControlSub.setPowerMotor2(0.5)));

    m_driverController.square().onTrue(new InstantCommand(() -> m_motorControlSub.setPowerMotor2(-0.5)));

    m_driverController.R1()
        .onTrue(new InstantCommand(() -> m_motorControlSub.setPowerMotor2(m_motorControlSub.m_motor2Power + 0.05)));

    m_driverController.R2()
        .onTrue(new InstantCommand(() -> m_motorControlSub.setPowerMotor2(m_motorControlSub.m_motor2Power - 0.05)));

    m_driverController.R3()
        .onTrue(new ParallelCommandGroup(new InstantCommand(() -> m_motorControlSub.setPowerMotor1(0)),
            new InstantCommand(() -> m_motorControlSub.setPowerMotor2(0))));

    m_driverController.PS()
        .onTrue(new InstantCommand(() -> m_motorControlSub.m_couple = (!m_motorControlSub.m_couple)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
