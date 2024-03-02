// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.time.Duration;
import java.time.Instant;
import java.util.logging.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSub;

public class ShooterShootCmd extends Command {
  private static Logger m_logger = Logger.getLogger(ShooterShootCmd.class.getName());

  private final ShooterSub m_shooterSub;
  private Instant start;

  /** Creates a new ShooterShootCmd. */

  public ShooterShootCmd(ShooterSub shooterSub) {
    m_shooterSub = shooterSub;

    addRequirements(shooterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_logger.fine("ShooterShootCmd - Init");
    start = Instant.now();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    // Flywheel needs to spin at full power prior to m_shooterSub.spinBothFeeders being executed.
    // double driveOutput = m_FlyWheelPID.calculate(m_shooterSub.getFlywheelVelocity(), 4200); //10 is a target velocity we don't know what it is
    // m_shooterSub.spinFlywheel(driveOutput);
    m_shooterSub.spinBothFeeders(Constants.Shooter.kNoteLowerIntakePower,
        Constants.Shooter.kNoteUpperIntakePower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_logger.fine("ShooterShootCmd - End" + (interrupted ? " (interrupted)" : ""));
    m_shooterSub.spinBothFeeders(0, 0);
  }

  @Override
  public boolean isFinished() {
    if(m_shooterSub.isNoteAtPosition(Constants.Shooter.kNoteSensorAtFlywheel)) {
      start = Instant.now();
    }
    Instant end = Instant.now();
    Duration timeElapsed = Duration.between(start, end);
    return timeElapsed.toSeconds() > 0.5;
  }
}
