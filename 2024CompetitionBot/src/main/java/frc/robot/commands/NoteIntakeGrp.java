// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ShooterSub;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NoteIntakeGrp extends SequentialCommandGroup {
  /** Creates a new NoteIntakeGrp. */
  public NoteIntakeGrp(IntakeSub intakeSub, ShooterSub shooterSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ShooterPivotCmd(Constants.Shooter.kPositionIntake, shooterSub),
        new InstantCommand(() -> intakeSub.setIntakeMotors(Constants.Intake.kNoteIntakePower)),
        new InstantCommand(() -> shooterSub.spinBothFeeders(Constants.Shooter.kNoteLowerIntakePower,
            Constants.Shooter.kNoteUpperIntakePower)),
        new IntakeReverseWhenNoteInCmd(shooterSub, intakeSub),
        new PivotStopRollersWhenNoteInCmd(shooterSub));
  }
}