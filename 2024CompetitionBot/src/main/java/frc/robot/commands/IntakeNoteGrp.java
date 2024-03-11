// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArduinoSub;
import frc.robot.subsystems.FeederSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ShooterSub;

public class IntakeNoteGrp extends SequentialCommandGroup {
  /** Creates a new IntakeNoteGrp. */
  public IntakeNoteGrp(ShooterSub shooterSub, IntakeSub intakeSub, FeederSub feederSub, ArduinoSub arduinoSub) {
    addCommands(
        new ShooterPivotCmd(0, shooterSub),
        new IntakeUntilNoteInCmd(intakeSub, feederSub, arduinoSub),
        new ShooterPivotCmd(10.0, shooterSub),
        new ExpellNoteABitCmd(feederSub, arduinoSub)

    );

  }
}