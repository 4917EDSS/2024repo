// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.FlywheelSub;
import frc.robot.subsystems.LedSub;
import frc.robot.subsystems.PivotSub;
import frc.robot.subsystems.ArduinoSub;
import frc.robot.subsystems.FeederSub;


// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ZeroPivotNoFlywheelGrp extends ParallelCommandGroup {
  /** Creates a new ShooterPrepGrp. */
  public ZeroPivotNoFlywheelGrp(ArduinoSub arduinoSub, FeederSub feederSub, FlywheelSub flywheelSub, LedSub ledSub,
      PivotSub pivotSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new IntakeNoteFeederCmd(feederSub),
        new ShooterPivotCmd(0, 0.4, pivotSub),
        new InstantCommand(() -> flywheelSub.disableFlywheel()));
  }
}
