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
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.IntakeSub;


public class UpIntakeGrp extends SequentialCommandGroup {
  public UpIntakeGrp(ShooterSub shooterSub, FeederSub feederSub, ArduinoSub arduinoSub,
      LedSub ledSub, IntakeSub intakeSub, FlywheelSub flywheelSub) {
    addCommands(
        new ShooterPivotCmd(Constants.Shooter.kHighPickUp, shooterSub),
        new IntakeUntilNoteInCmd(intakeSub, feederSub, arduinoSub, ledSub, flywheelSub),
        new ShooterPivotCmd(50.0, shooterSub),
        new ExpelNoteABitCmd(feederSub, arduinoSub),
        new IntakeNoteFromSourceCmd(feederSub, arduinoSub, ledSub));

  }
}
