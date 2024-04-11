// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArduinoSub;
import frc.robot.subsystems.FeederSub;
import frc.robot.subsystems.FlywheelSub;
import frc.robot.subsystems.LedSub;
import frc.robot.subsystems.LedSub.LedColour;
import frc.robot.subsystems.LedSub.LedZones;
import frc.robot.subsystems.PivotSub;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LobPrepGrp extends SequentialCommandGroup {
  /** Creates a new LobPrepGrp. */
  public LobPrepGrp(ArduinoSub arduinoSub, FeederSub feederSub, FlywheelSub flywheelSub, LedSub ledSub,
      PivotSub pivotSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new IntakeUntilNoteInCmd(arduinoSub, feederSub, flywheelSub, ledSub),
        new ParallelCommandGroup(
            new ShooterPivotCmd(Constants.Shooter.kAnglePassing, pivotSub),
            new SequentialCommandGroup(
                new WaitCommand(0.3),
                new InstantCommand(() -> flywheelSub.enableFlywheel(Constants.Flywheel.kFlywheelLobVelocity)),
                new ExpelNoteABitCmd(arduinoSub, feederSub),
                new InstantCommand(() -> ledSub.setZoneColour(LedZones.ALL, LedColour.BLUE)))));
  }
}
