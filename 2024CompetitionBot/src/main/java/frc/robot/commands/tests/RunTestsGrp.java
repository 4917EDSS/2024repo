// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tests;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.FlywheelSub;
import frc.robot.utils.TestManager;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunTestsGrp extends SequentialCommandGroup {

  /** Creates a new RunTestsGrp. */
  public RunTestsGrp(DrivetrainSub drivetrainSub, FlywheelSub flywheelSub, TestManager testManager) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> testManager.resetTestStatuses()),
        new TestSwerveDriveMotorsCmd(drivetrainSub, testManager),
        new TestFlywheelMotorsCmd(flywheelSub, testManager),
        new InstantCommand(() -> testManager.updateOverallStatus()));

  }
}
