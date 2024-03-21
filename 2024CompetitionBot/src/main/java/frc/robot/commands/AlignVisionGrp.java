// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.ArduinoSub;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.FeederSub;
import frc.robot.subsystems.FlywheelSub;
import frc.robot.subsystems.LedSub;
import frc.robot.subsystems.PivotSub;
import frc.robot.subsystems.VisionSub;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignVisionGrp extends SequentialCommandGroup {
  /** Creates a new ShooterPrepGrp. */
  public AlignVisionGrp(CommandPS4Controller driverController, CommandPS4Controller operatorController,
      ArduinoSub arduinoSub, DrivetrainSub drivetrainSub, FeederSub feederSub, FlywheelSub flywheelSub, LedSub ledSub,
      PivotSub pivotSub, VisionSub visionSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // If pivot angle is less than 9, pivot to 10 degrees.  Otherwise, don't move the pivot
        new ConditionalCommand(
            new ShooterPivotCmd(10, pivotSub),
            new InstantCommand(),
            () -> pivotSub.getPivotAngle() < 9),
        new ExpelNoteABitCmd(arduinoSub, feederSub),
        new AlignVisionCmd(driverController, operatorController, drivetrainSub, feederSub, flywheelSub, ledSub,
            pivotSub, visionSub),
        new ShooterShootCmd(arduinoSub, feederSub, flywheelSub, ledSub),

        new ParallelDeadlineGroup(
            new ShooterPivotCmd(0, pivotSub),
            new DriveFieldRelativeCmd(driverController, drivetrainSub)));
  }
}
