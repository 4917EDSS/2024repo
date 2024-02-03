// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PwmIds;
import frc.robot.subsystems.LedSub.LedColour;
import frc.robot.subsystems.LedSub.LedZones;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import java.util.logging.Logger;
import com.revrobotics.CANSparkLowLevel;
import frc.robot.subsystems.LedSub;


public class IntakeSub extends SubsystemBase {
  private static Logger m_logger = Logger.getLogger(IntakeSub.class.getName());

  private boolean m_beOrange = false;
  private boolean flashGreen = false;
  private final LedSub m_LedSub = new LedSub();
  private final CANSparkMax m_intakeRollers =
      new CANSparkMax(Constants.CanIds.kIntakeRollers, CANSparkLowLevel.MotorType.kBrushless);
  // Line below most likely will not be used, but can be used as a working absolute encoder if necessary
  private final DigitalInput m_intakeLimitSwitch = new DigitalInput(Constants.DioIds.kIntakeLimitPort);

  /** Creates a new Intake. */
  public IntakeSub() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // updateSmartDashboard();
    if(getNoteFullyIn() && !m_beOrange) {
      flashGreen = true;
    }

    if(flashGreen) {
      m_LedSub.Flash(LedColour.GREEN);
      m_beOrange = true;
      flashGreen = false;
    }

    if(m_beOrange) {
      m_LedSub.setZoneColour(LedZones.DIAG_NOTE_INSIDE, LedColour.ORANGE);
    }

    if(!getNoteFullyIn()) {
      m_beOrange = false;
    }
  }

  public boolean getNoteFullyIn() {
    return !m_intakeLimitSwitch.get();
  }

  public void updateSmartDashboard() {
    SmartDashboard.putBoolean("Note In", getNoteFullyIn());
  }

  public void setIntakeMotors(double power) {
    m_intakeRollers.set(power);
  }

  public void init() {

  }
}

