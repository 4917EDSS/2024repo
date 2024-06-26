// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArduinoSub;
import frc.robot.subsystems.FeederSub;
import frc.robot.subsystems.FlywheelSub;
import frc.robot.subsystems.LedSub;
import frc.robot.subsystems.PivotSub;
import frc.robot.subsystems.LedSub.LedColour;
import frc.robot.subsystems.LedSub.LedZones;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShooterPrepGrp extends SequentialCommandGroup {
  /** Creates a new ShooterPrepGrp. */
  public ShooterPrepGrp(double pivotPosition, double flywheelVelocity, ArduinoSub arduinoSub, FeederSub feederSub,
      FlywheelSub flywheelSub,
      PivotSub pivotSub, LedSub ledSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ConditionalCommand(
            new ShooterPivotCmd(10, pivotSub),
            new InstantCommand(),
            () -> pivotSub.getPivotAngle() < 9),
        new ParallelCommandGroup(
            new ShooterFlywheelCmd(flywheelVelocity, flywheelSub),
            new ShooterPivotCmd(pivotPosition, pivotSub),
            new ExpelNoteABitCmd(arduinoSub, feederSub)),
        new InstantCommand(() -> ledSub.setZoneColour(LedZones.ALL, LedColour.ORANGE), ledSub));
  }
}
