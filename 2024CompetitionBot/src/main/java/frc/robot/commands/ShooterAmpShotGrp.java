// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.FeederSub;
import frc.robot.subsystems.LedSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.Constants;
import frc.robot.subsystems.ArduinoSub;


// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShooterAmpShotGrp extends SequentialCommandGroup {
  /** Creates a new ShooterAmpShotGrp. */
  public ShooterAmpShotGrp(ShooterSub shooterSub, FeederSub feederSub, ArduinoSub arduinoSub, LedSub ledSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ShooterPivotCmd(Constants.Shooter.kAngleAmp, shooterSub),
        new ShooterAmpShotCmd(feederSub, arduinoSub, ledSub)

    );
  }
}
