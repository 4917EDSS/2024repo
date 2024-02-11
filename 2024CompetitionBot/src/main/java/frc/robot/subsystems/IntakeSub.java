// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSub extends SubsystemBase {
  private static Logger m_logger = Logger.getLogger(IntakeSub.class.getName());

  private final CANSparkMax m_intakeRollers =
      new CANSparkMax(Constants.CanIds.kIntakeRollers, CANSparkLowLevel.MotorType.kBrushless);


  /** Creates a new Intake. */
  public IntakeSub() {
    // This subsystem was going to be much more complicated but now it's just a set of rollers
    init();
  }

  public void init() {
    m_logger.info("Initializing IntakeSub");
    setIntakeMotors(0.0);
  }

  @Override
  public void periodic() {}

  // positive power intakes 
  public void setIntakeMotors(double power) {
    m_intakeRollers.set(power);
  }
}

