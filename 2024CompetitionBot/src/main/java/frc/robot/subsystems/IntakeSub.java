// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.LedSub.LedColour;
import frc.robot.subsystems.LedSub.LedZones;


public class IntakeSub extends SubsystemBase {
  private final LedSub m_LedSub = new LedSub();
  private final CANSparkMax m_intakeRollers =
      new CANSparkMax(Constants.CanIds.kIntakeRollers, CANSparkLowLevel.MotorType.kBrushless);
  // Line below most likely will not be used, but can be used as a working absolute encoder if necessary
  private final DigitalInput m_intakeLimitSwitch = new DigitalInput(Constants.DioIds.kIntakeLimitPort);

  private boolean m_noteWasIn = false;

  /** Creates a new Intake. */
  public IntakeSub() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // If note-was't-in and isNoteFullyIn
    // Flash green
    // Set orange
    if(!m_noteWasIn && isNoteFullyIn()) {
      m_LedSub.Flash(LedColour.GREEN);
      m_LedSub.setZoneColour(LedZones.GAME_PIECE, LedColour.ORANGE);
      m_noteWasIn = true;
    } else if(m_noteWasIn && !isNoteFullyIn()) {
      // If note-was-in and !isNoteFullyIn
      // Set green
      m_LedSub.setZoneColour(LedZones.GAME_PIECE, LedColour.GREEN);
      m_noteWasIn = false;
    }
  }

  public boolean isNoteFullyIn() {
    return !m_intakeLimitSwitch.get();
  }

  public void updateSmartDashboard() {
    SmartDashboard.putBoolean("Note In", isNoteFullyIn());
  }

  public void setIntakeMotors(double power) {
    m_intakeRollers.set(power);
  }

  public void init() {

  }
}

