// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.FlywheelSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.LedSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.ArduinoSub;
import frc.robot.subsystems.FeederSub;


// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ZeroPivotNoFlywheelGrp extends SequentialCommandGroup {
  /** Creates a new ShooterPrepGrp. */
  public ZeroPivotNoFlywheelGrp(ShooterSub shooterSub, FlywheelSub flywheelSub, FeederSub feederSub,
      IntakeSub intakeSub, ArduinoSub arduinoSub, LedSub ledSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new IntakeUntilNoteInCmd(intakeSub, feederSub, arduinoSub, ledSub),
        new ShooterPivotCmd(1, shooterSub),
        new InstantCommand(() -> flywheelSub.disableFlywheel()));

  }
}
