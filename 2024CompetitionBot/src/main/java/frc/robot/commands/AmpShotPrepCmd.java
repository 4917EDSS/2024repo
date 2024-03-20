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


public class AmpShotPrepCmd extends Command {
  private final ArduinoSub m_arduinoSub;
  private final FeederSub m_feederSub;
  private Instant start;

  public AmpShotPrepCmd(ArduinoSub arduinoSub, FeederSub feederSub) {
    m_arduinoSub = arduinoSub;
    m_feederSub = feederSub;

    addRequirements(feederSub);
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
