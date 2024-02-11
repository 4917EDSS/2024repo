// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DummySub extends SubsystemBase {
  /** Creates a new DummySub. */
  public DummySub() {
    // For all the stuff that's on the robot that needs to be initialized to make things happy
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void doNothing() {

  }
}
