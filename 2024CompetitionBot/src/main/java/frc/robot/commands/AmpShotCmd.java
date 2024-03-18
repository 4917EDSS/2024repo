// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.time.Duration;
import java.time.Instant;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArduinoSub;
import frc.robot.subsystems.FeederSub;
import frc.robot.subsystems.LedSub;
import frc.robot.subsystems.LedSub.LedColour;
import frc.robot.subsystems.LedSub.LedZones;

public class AmpShotCmd extends Command {
  /** Creates a new AmpShotCmd. */

  private final FeederSub m_feederSub;
  private final ArduinoSub m_arduinoSub;
  private final LedSub m_ledSub;
  private Instant start;

  public AmpShotCmd(FeederSub feederSub, ArduinoSub arduinoSub, LedSub ledSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_feederSub = feederSub;
    m_arduinoSub = arduinoSub;
    m_ledSub = ledSub;

    addRequirements(feederSub, arduinoSub, ledSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_feederSub.spinBothFeeders(-0.4, -0.4);
    start = Instant.now();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_arduinoSub.isSensorTripped(Constants.Shooter.kNoteSensorIntakeFar)) {
      m_feederSub.spinBothFeeders(-0.0, -0.1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feederSub.spinBothFeeders(0, 0);
    m_ledSub.setZoneColour(LedZones.ALL, LedColour.GREEN);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_arduinoSub.isAnySensorTripped()) {
      start = Instant.now();
    }
    Instant end = Instant.now();
    Duration timeElapsed = Duration.between(start, end);
    return timeElapsed.toMillis() > 80;
  }
}
