// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.logging.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSub;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.FeederSub;
import frc.robot.subsystems.FlywheelSub;
import frc.robot.subsystems.ShooterSub;

public class KillAllCmd extends Command {
  private static Logger m_logger = Logger.getLogger(KillAllCmd.class.getName());

  private final FlywheelSub m_flywheelSub;

  public KillAllCmd(ClimbSub climbSub, DrivetrainSub drivetrainSub, FeederSub feederSub, FlywheelSub flywheelSub,
      ShooterSub shooterSub) {
    m_flywheelSub = flywheelSub;

    addRequirements(climbSub, drivetrainSub, feederSub, flywheelSub, shooterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_logger.fine("KillAllCmd - Init");
    m_flywheelSub.disableFlywheel();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_logger.fine("KillAllCmd - End" + (interrupted ? " (interrupted)" : ""));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
