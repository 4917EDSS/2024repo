// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TestLedsCmd;
import frc.robot.subsystems.ClimbSub;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.LedSub;
import frc.robot.subsystems.LedSub.LedColour;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.subsystems.VisionSub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.LedSub;
import frc.robot.subsystems.LedSub.LedColour;
import frc.robot.subsystems.LedSub.LedZones;
import frc.robot.subsystems.ShooterSub;
import frc.robot.commands.ClimbCmdSetHeightCmd;

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

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_visionSub.setPipeline(2); // Apriltag vision
    m_drivetrainSub.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_drivetrainSub.drive(
                m_driverController.getLeftY(),
                m_driverController.getLeftX(),
                m_driverController.getRightX(),
                true,
                0.2), // this is the duration fo thh timestep the speeds should be applied to. Should probably be changed 
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

    m_driverController.cross().onTrue(new PrintCommand("Cross Pressed!"));


    //m_driverController.triangle().onTrue(LedColour.GREEN);

    //Change to operator controller
    m_driverController.L1().onTrue(new TestLedsCmd(m_ledSub, LedColour.BLUE));
    m_driverController.L2().onTrue(new TestLedsCmd(m_ledSub, LedColour.YELLOW));
    //m_driverController.L2().onTrue(new PrintCommand("focus canning"));

    //here we are making the climb
    m_driverController.square().onTrue(new ClimbCmdSetHeightCmd(m_climbSub, Constants.ClimbConstants.kHookScoring));
    m_driverController.circle().onTrue(new ClimbCmdSetHeightCmd(m_climbSub, Constants.ClimbConstants.kHookJustup));
    m_driverController.triangle().onTrue(new ClimbCmdSetHeightCmd(m_climbSub, Constants.ClimbConstants.kHookRaised));
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
}

