// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveWithJoystickCmd;
import frc.robot.commands.tests.RunTestsGrp;
import frc.robot.subsystems.KrakenSub;
import frc.robot.utils.GVersion;
import frc.robot.utils.TestManager;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final KrakenSub m_krakenSub = new KrakenSub();
  private final TestManager m_testManager = new TestManager();
  private final CommandPS4Controller m_driverController =
      new CommandPS4Controller(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_testManager.setTestCommand(new RunTestsGrp(m_krakenSub, m_testManager));
    m_krakenSub.setDefaultCommand(new DriveWithJoystickCmd(m_driverController, m_krakenSub));

    configureDashboardAboutTab();

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   */
  private void configureBindings() {
    m_driverController.cross()
        .whileTrue(new StartEndCommand(() -> m_krakenSub.runMotor(0.1), () -> m_krakenSub.runMotor(0.0), m_krakenSub));
    m_driverController.triangle()
        .whileTrue(new RunCommand(() -> m_krakenSub.resetPosition(), m_krakenSub));
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

  public void testPeriodic() {

  }

  private void configureDashboardAboutTab() {
    ShuffleboardTab tab;
    tab = Shuffleboard.getTab("About");

    ShuffleboardLayout layout;
    layout = tab.getLayout("Build Info", BuiltInLayouts.kList)
        .withProperties(Map.of("label position", "top"))
        .withSize(2, 4)
        .withPosition(0, 0);
    // WARNING: These "GVersion" values get created when you first build this project
    // It's okay for them to be red until then.  See build.gradle and utils/GVersion.java.
    layout.add("Project", GVersion.MAVEN_NAME);
    layout.add("Date", GVersion.BUILD_DATE);
    layout.add("Git Dirty?", GVersion.DIRTY); // 1 = Code contains changes not committed to Git, 0 = clean, -1 = problem
    layout.add("Git Branch", GVersion.GIT_BRANCH);
    layout.add("Git Commit Date", GVersion.GIT_DATE); // Doesn't mean much unless dirty = 0
    layout.add("Git Commit SHA", GVersion.GIT_SHA); // Doesn't mean much unless dirty = 0
  }
}
