// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArduinoSub;
import frc.robot.subsystems.FeederSub;
import frc.robot.subsystems.FlywheelSub;
import frc.robot.subsystems.PivotSub;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FastShooterPrepGrp extends ParallelCommandGroup {
  /** Creates a new ShooterPrepGrp. */
  public FastShooterPrepGrp(double pivotPosition, ArduinoSub arduinoSub, FeederSub feederSub, FlywheelSub flywheelSub,
      PivotSub pivotSub) {
    this(pivotPosition, Constants.Flywheel.kFlywheelShootVelocity, arduinoSub, feederSub, flywheelSub, pivotSub);
  }

  public FastShooterPrepGrp(double pivotPosition, double flywheelSpeed, ArduinoSub arduinoSub, FeederSub feederSub,
      FlywheelSub flywheelSub,
      PivotSub pivotSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ConditionalCommand(new FastShooterFlywheelCmd(flywheelSub),
            new ShooterFlywheelCmd(flywheelSpeed, flywheelSub),
            () -> (pivotPosition < 40.0)),
        new ShooterPivotCmd(pivotPosition, pivotSub),
        new SequentialCommandGroup(
            new WaitCommand(0.2),
            new ExpelNoteABitCmd(arduinoSub, feederSub)));
  }
}
