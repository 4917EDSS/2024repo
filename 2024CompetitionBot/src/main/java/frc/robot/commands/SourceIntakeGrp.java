// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArduinoSub;
import frc.robot.subsystems.FeederSub;
import frc.robot.subsystems.FlywheelSub;
import frc.robot.subsystems.LedSub;
import frc.robot.subsystems.PivotSub;


public class SourceIntakeGrp extends SequentialCommandGroup {
  public SourceIntakeGrp(ArduinoSub arduinoSub, FeederSub feederSub, FlywheelSub flywheelSub, LedSub ledSub,
      PivotSub pivotSub) {
    addCommands(
        new ShooterPivotCmd(Constants.Shooter.kHighPickUp, pivotSub),
        new IntakeUntilNoteInCmd(arduinoSub, feederSub, flywheelSub, ledSub),
        new ShooterPivotCmd(50.0, pivotSub),
        new ExpelNoteABitCmd(arduinoSub, feederSub),
        new IntakeNoteFromSourceCmd(arduinoSub, feederSub, ledSub));
  }
}
