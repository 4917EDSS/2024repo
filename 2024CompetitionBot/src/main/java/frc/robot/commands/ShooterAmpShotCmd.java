// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.time.Duration;
import java.time.Instant;
import java.util.logging.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FeederSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.ArduinoSub;

public class ShooterAmpShotCmd extends Command {
  private static Logger m_logger = Logger.getLogger(ShooterAmpShotCmd.class.getName());

  private final ShooterSub m_shooterSub;
  private final FeederSub m_feederSub;
  private final ArduinoSub m_arduinoSub;
  private Instant start;

  public ShooterAmpShotCmd(ShooterSub shooterSub, FeederSub feederSub, ArduinoSub arduinoSub) {
    m_shooterSub = shooterSub;
    m_feederSub = feederSub;
    m_arduinoSub = arduinoSub;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSub, feederSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_logger.fine("ShooterAmpShotCmd - Init");
    m_feederSub.spinBothFeeders(Constants.Shooter.kNoteLowerAmpShotPower, Constants.Shooter.kNoteUpperAmpShotPower);
    start = Instant.now();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_logger.fine("ShooterAmpShotCmd - End" + (interrupted ? " (interrupted)" : ""));
    m_feederSub.spinBothFeeders(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_arduinoSub.isSensorTripped(Constants.Shooter.kNoteSensorFwFar)) {
      start = Instant.now();
    }
    Instant end = Instant.now();
    Duration timeElapsed = Duration.between(start, end);
    return timeElapsed.toSeconds() > 3.0;
  }
}
