// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tests;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.KrakenSub;
import frc.robot.utils.StateOfRobot;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunTestsGrp extends SequentialCommandGroup {

  /** Creates a new RunTestsGrp. */
  public RunTestsGrp(KrakenSub krakenSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> StateOfRobot.m_testsOverallResult = false), // Stay "red" until we pass all tests
        new InstantCommand(() -> StateOfRobot.m_newTestResults = true), // Tell RobotContainer to update the results status
        new TestKrakenSubCmd(krakenSub),
        new InstantCommand(() -> StateOfRobot.m_newTestResults = true) // Test done.  Tell RobotContainer to update the results status
    );
  }
}
