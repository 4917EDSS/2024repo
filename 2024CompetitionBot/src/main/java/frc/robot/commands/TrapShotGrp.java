// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimbSub;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.PivotSub;

public class TrapShotGrp extends SequentialCommandGroup {
  public TrapShotGrp(DriveFieldRelativeCmd driveFieldRelativeCmd, ClimbSub climbSub, DrivetrainSub drivetrainSub,
      PivotSub pivotSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // Untested
        new TrapShotPrepGrp(climbSub, drivetrainSub, pivotSub),
        new TrapShotStepTwoGrp(driveFieldRelativeCmd, climbSub, drivetrainSub, pivotSub));
  }
}
