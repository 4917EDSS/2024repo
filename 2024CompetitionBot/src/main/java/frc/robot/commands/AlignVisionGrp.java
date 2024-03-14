// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.ArduinoSub;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.FeederSub;
import frc.robot.subsystems.FlywheelSub;
import frc.robot.subsystems.LedSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.VisionSub;
import frc.robot.subsystems.IntakeSub;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignVisionGrp extends SequentialCommandGroup {
  /** Creates a new ShooterPrepGrp. */
  public AlignVisionGrp(DrivetrainSub drivetrainSub, VisionSub visionSub, ShooterSub shooterSub, FeederSub feederSub,
      FlywheelSub flywheelSub, LedSub ledSub, CommandPS4Controller driverController,
      CommandPS4Controller operatorController, ArduinoSub arduinoSub, IntakeSub intakeSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ConditionalCommand(new ShooterPivotCmd(10, shooterSub), new InstantCommand(),
            () -> shooterSub.getPivotAngle() < 9),
        new ExpelNoteABitCmd(feederSub, arduinoSub),
        new AlignVisionCmd(drivetrainSub, visionSub, shooterSub, feederSub, flywheelSub, ledSub,
            driverController, operatorController),
        new ShooterShootCmd(flywheelSub, feederSub, arduinoSub, shooterSub, ledSub),
        new ShooterPivotCmd(0, shooterSub));


  }
}
