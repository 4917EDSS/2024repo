// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSub;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.ShooterSub;

public class TrapShotStepTwoGrp extends SequentialCommandGroup {
  public TrapShotStepTwoGrp(ShooterSub shooterSub, ClimbSub climbSub, DrivetrainSub drivetrainSub,
      DriveFieldRelativeCmd driveFieldRelativeCmd) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ClimbSetHeightCmd(Constants.Climb.kHeightGrabChain, Constants.Climb.kPower, drivetrainSub, climbSub));
    // new DriveFieldRelativeCmd());  drive back a amount 
    //move pivot to corect angle 
    //new ShooterPivotCmd(Constants.Shooter.kAngleTrapShot, shooterSub),
    //drive forward amount driven back
    //climb to trap height n
    //new ClimbSetHeightCmd(Constants.Climb.kGoToTrapShot, Constants.Climb.kPower, drivetrainSub, climbSub));

  }
}
