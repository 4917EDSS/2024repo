// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSub;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.ShooterSub;

public class TrapShotPrepGrp extends ParallelCommandGroup {
  /** Creates a new TrapShoot. */
  public TrapShotPrepGrp(ClimbSub climbSub, DrivetrainSub drivetrainSub, ShooterSub shooterSub) {
    addCommands(
        new ShooterPivotCmd(Constants.Shooter.kAngleTrap, shooterSub),
        new ClimbSetHeightCmd(Constants.Climb.kHeightShortHookRaised, Constants.Climb.kPower, climbSub, drivetrainSub));
  }
}
