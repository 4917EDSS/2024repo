// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.logging.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArduinoSub;
import frc.robot.subsystems.FeederSub;
import frc.robot.subsystems.FlywheelSub;
import frc.robot.subsystems.LedSub;
import frc.robot.subsystems.LedSub.LedColour;
import frc.robot.subsystems.LedSub.LedZones;


public class IntakeUntilNoteInCmd extends Command {
  private static Logger m_logger = Logger.getLogger(IntakeUntilNoteInCmd.class.getName());

  private final ArduinoSub m_arduinoSub;
  private final FeederSub m_feederSub;
  private final FlywheelSub m_flywheelSub;
  private final LedSub m_LedSub;

  private boolean lowerSensorTripped = false;

  public IntakeUntilNoteInCmd(ArduinoSub arduinoSub, FeederSub feederSub, FlywheelSub flywheelSub, LedSub ledSub) {
    m_arduinoSub = arduinoSub;
    m_feederSub = feederSub;
    m_flywheelSub = flywheelSub;
    m_LedSub = ledSub;

    addRequirements(feederSub, flywheelSub); // It's fine if two commands change LEDs
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_logger.fine("IntakeUntilNoteInCmd - Init");
    m_feederSub.finishedNoteIntake = false;
    m_feederSub.setIntakeMotors(Constants.Intake.kNoteIntakePower);
    m_feederSub.spinBothFeeders(Constants.Shooter.kNoteLowerIntakePower, Constants.Shooter.kNoteUpperIntakePower);
    m_LedSub.setZoneColour(LedZones.ALL, LedColour.RED);
    m_flywheelSub.brakeFlywheels();
    lowerSensorTripped = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_arduinoSub.isAnySensorTripped()) {
      m_LedSub.setZoneColour(LedZones.ALL, LedColour.WHITE);
    } else {
      m_LedSub.setZoneColour(LedZones.ALL, LedColour.RED);
    }
    // Once the note has cleared the intake rollers, run those rollers in reverse to avoid controlling two notes
    // if(m_pivotSub.isNoteAtPosition(Constants.Shooter.kNoteSensorNearFlywheel)) {
    //   m_intakeSub.setIntakeMotors(Constants.Intake.kNoteExpelPower);
    // }
    // if(m_arduinoSub.isSensorTripped(Constants.Shooter.kNoteSensorFwFar)) {
    //   m_intakeSub.setIntakeMotors(Constants.Intake.kNoteExpelPower);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_logger.fine("IntakeUntilNoteInCmd - End" + (interrupted ? " (interrupted)" : ""));

    // Make sure we're running the intake rollers in reverse and the feed rollers are off
    m_feederSub.setIntakeMotors(0); // Possible reason why the notes were occasionally getting stuck.
    //m_intakeSub.setIntakeMotors(Constants.Intake.kNoteExpelPower);
    m_feederSub.spinBothFeeders(0, 0);
    m_flywheelSub.unbrakeFlywheels();
    if(!interrupted) {
      m_LedSub.setZoneColour(LedZones.ALL, LedColour.WHITE);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Once we hit the last sensor, stop the feed rollers
    // if(m_arduinoSub.isSensorTripped(Constants.Shooter.kNoteSensorAtFlywheel)) {
    //   return true;
    // }
    if(lowerSensorTripped) {
      if(m_arduinoSub.isSensorTripped(Constants.Shooter.kNoteSensorFwNear)) {
        m_feederSub.finishedNoteIntake = true;
        return true;
      }
      return false;
    } else {
      if(m_arduinoSub.isSensorTripped(Constants.Shooter.kNoteSensorIntakeNear)) {
        lowerSensorTripped = true;
      }
      return false;
    }
  }
}
