// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkLowLevel;


public class IntakeSub extends SubsystemBase {
  private final CANSparkMax m_intakeRollers =
      new CANSparkMax(Constants.CanIds.kIntakeRollers, CANSparkLowLevel.MotorType.kBrushless);
  // Line below most likely will not be used, but can be used as a working absolute encoder if necessary
  private final DigitalInput m_intakeLimitSwitch = new DigitalInput(Constants.DioIds.kIntakeLimitPort);
  private final ShuffleboardTab m_shuffleboardTab = Shuffleboard.getTab("Intake");
  private final GenericEntry m_sbNoteIn;

  /** Creates a new Intake. */
  public IntakeSub() {
    m_sbNoteIn = m_shuffleboardTab.add("Note In", false).getEntry();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateShuffleBoard();
  }

  public boolean getNoteFullyIn() {
    return !m_intakeLimitSwitch.get();
  }

  public void updateShuffleBoard() {
    //SmartDashboard.putBoolean("Note In", getNoteFullyIn());
    m_sbNoteIn.setBoolean(getNoteFullyIn());
  }

  public void setIntakeMotors(double power) {
    m_intakeRollers.set(power);
  }

  public void init() {

  }
}

