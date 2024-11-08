// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tests;

import java.time.Duration;
import java.time.Instant;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.KrakenSub;
import frc.robot.utils.StateOfRobot;

/**
 * This test checks that the motor is communicating. Then it runs it for a given time. While
 * running, it monitors the current. When it stops, it checks the encoder position.
 * 
 * PASS = Motor is there, the current draw was within tolerance, and the final position is within
 * tolerance.
 * WARNING = Motor is there. Some minimal current was drawn. Some minimal change in position was
 * registered.
 * FAIL = All other cases.
 */
public class TestKrakenSubCmd extends Command {
  private final KrakenSub m_krakenSub;
  private Instant startTime;

  /** Creates a new TestKrakenSubCmd. */
  public TestKrakenSubCmd(KrakenSub krakenSub) {
    m_krakenSub = krakenSub;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(krakenSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Make sure the motor is there
    //TODO: Figure out how to check if the motor is even there before we start trying to use it

    // Reset the encoder and run the motor for a given time
    startTime = Instant.now();
    m_krakenSub.resetPosition();
    m_krakenSub.runMotor(Constants.Tests.kDriveMotorPower);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Nothing to do here but wait for the test to be over
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Before turning off the motor, read the current current (amps) and position
    double currentAmps = m_krakenSub.getAmps();
    double currentPosition = m_krakenSub.getPosition();

    // Stop the motor
    m_krakenSub.runMotor(0.0);

    // TODO: Finish this logic
    // Check to see if the measured current is good, ok or bad
    if(Math.abs(currentAmps - Constants.Tests.kDriveMotorExpectedAmps) < Constants.Tests.kDriveMotorAmpsTolerance) {

    } else if(currentAmps > Constants.Tests.kDriveMotorAmpsMinimum) {

    } else {

    }
    System.out.println("Got " + currentAmps + " (Target=" + Constants.Tests.kDriveMotorExpectedAmps + "+/-"
        + Constants.Tests.kDriveMotorAmpsTolerance + ")");

    // Check to see if the measured position is good, ok or bad
    if(Math.abs(
        currentPosition - Constants.Tests.kDriveMotorExpectedPosition) < Constants.Tests.kDriveMotorPositionTolerance) {

    } else if(currentPosition > Constants.Tests.kDriveMotorPositionMinimum) {

    } else {

    }
    System.out.println("Got " + currentPosition + " (Target=" + Constants.Tests.kDriveMotorExpectedPosition + "+/-"
        + Constants.Tests.kDriveMotorPositionTolerance + ")");


    // If not, check if they are at least over the minimums

    // Set the current test pass/fail status and set the global status on a fail

    StateOfRobot.m_testsOverallResult = true; // TODO: Fix

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Wait for the test time to have elapsed
    if(Duration.between(startTime, Instant.now()).toMillis() > Constants.Tests.kDriveMotorTimeMs) {
      return true;
    }
    return false;
  }
}
