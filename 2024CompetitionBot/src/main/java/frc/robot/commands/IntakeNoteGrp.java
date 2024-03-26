// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArduinoSub;
import frc.robot.subsystems.FeederSub;
import frc.robot.subsystems.FlywheelSub;
import frc.robot.subsystems.LedSub;
import frc.robot.subsystems.PivotSub;
import frc.robot.subsystems.LedSub.LedColour;
import frc.robot.subsystems.LedSub.LedZones;

public class IntakeNoteGrp extends SequentialCommandGroup {

  private final LedSub m_ledSub;
  private final FlywheelSub m_flywheelSub;

  public IntakeNoteGrp(ArduinoSub arduinoSub, FeederSub feederSub, FlywheelSub flywheelSub, LedSub ledSub,
      PivotSub pivotSub) {
    m_ledSub = ledSub;
    m_flywheelSub = flywheelSub;
    addCommands(new InstantCommand(() -> m_flywheelSub.brakeFlywheels()),
        new ShooterPivotCmd(0, pivotSub),
        new IntakeUntilNoteInCmd(arduinoSub, feederSub, flywheelSub, ledSub),
        new ShooterPivotCmd(10.0, pivotSub),
        new ExpelNoteABitCmd(arduinoSub, feederSub),
        new InstantCommand(() -> m_ledSub.setZoneColour(LedZones.ALL, LedColour.BLUE)),
        new InstantCommand(() -> m_flywheelSub.enableFlywheel(Constants.Flywheel.kFlywheelShootVelocity)));
  }
}
