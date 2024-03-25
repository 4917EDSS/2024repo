// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.time.Duration;
import java.time.Instant;
import java.util.logging.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArduinoSub;
import frc.robot.subsystems.FeederSub;
import frc.robot.subsystems.FlywheelSub;
import frc.robot.subsystems.LedSub;
import frc.robot.subsystems.LedSub.LedColour;
import frc.robot.subsystems.LedSub.LedZones;

public class ShooterShootCmd extends Command {
  private static Logger m_logger = Logger.getLogger(ShooterShootCmd.class.getName());

  private final ArduinoSub m_arduinoSub;
  private final FeederSub m_feederSub;
  private final FlywheelSub m_flywheelSub;
  private final LedSub m_ledSub;

  private Instant start;

  public ShooterShootCmd(ArduinoSub arduinoSub, FeederSub feederSub, FlywheelSub flywheelSub, LedSub ledSub) {
    m_arduinoSub = arduinoSub;
    m_feederSub = feederSub;
    m_flywheelSub = flywheelSub;
    m_ledSub = ledSub;

    addRequirements(feederSub, flywheelSub); // It's fine if two commands change LEDs
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_logger.fine("ShooterShootCmd - Init");
    start = Instant.now();
    m_flywheelSub.enableFlywheel(Constants.Flywheel.kFlywheelShootVelocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Flywheel should always be at targetr speed
    m_feederSub.spinBothFeeders(Constants.Shooter.kNoteLowerIntakePower,
        Constants.Shooter.kNoteUpperIntakePower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_logger.fine("ShooterShootCmd - End" + (interrupted ? " (interrupted)" : ""));
    m_feederSub.spinBothFeeders(0, 0);
    m_flywheelSub.disableFlywheel();
    m_ledSub.setZoneColour(LedZones.ALL, LedColour.GREEN);
  }

  @Override
  public boolean isFinished() {
    if(m_arduinoSub.isAnySensorTripped()) {
      start = Instant.now();
    }
    Instant end = Instant.now();
    Duration timeElapsed = Duration.between(start, end);
    return timeElapsed.toMillis() > 100;
  }
}
