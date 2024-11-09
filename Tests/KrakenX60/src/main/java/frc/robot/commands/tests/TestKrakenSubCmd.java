// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tests;

import java.time.Duration;
import java.time.Instant;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.KrakenSub;
import frc.robot.utils.TestManager;
import frc.robot.utils.TestManager.Result;

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
  private final TestManager m_testManager;
  private final int m_testId;
  private Instant m_startTime;
  private boolean m_abortTest = false;

  /** Creates a new TestKrakenSubCmd. */
  public TestKrakenSubCmd(KrakenSub krakenSub, TestManager testManager) {
    m_krakenSub = krakenSub;
    m_testManager = testManager;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(krakenSub);

    m_testId = m_testManager.registerNewTest("Kraken 1");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset the encoder and run the motor for a given time
    m_startTime = Instant.now();
    m_krakenSub.resetPosition();
    StatusCode sc = m_krakenSub.runMotor(Constants.Tests.kDriveMotorPower);

    if(sc != StatusCode.OK) {
      // Motor isn't working correctly.  Make sure it's off and end the test.
      m_abortTest = true;
      m_krakenSub.runMotor(0.0);
      String resultsText = "Motor error " + sc.value + " - " + sc.getName() + " - " + sc.getDescription();
      m_testManager.updateTestStatus(m_testId, TestManager.Result.kFail, resultsText);
      System.out.println("KrakenSub " + resultsText);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Nothing to do here but wait for the test to be over
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(m_abortTest) {
      // Already stopped the motor and reported the test results, just quit
      return;
    }

    if(interrupted) {
      m_krakenSub.runMotor(0.0);
      m_testManager.updateTestStatus(m_testId, Result.kFail, "Test interrupted");
      return;
    }

    // Before turning off the motor, read the current current (amps) and position
    double currentAmps = m_krakenSub.getAmps();
    double currentPosition = m_krakenSub.getPosition();

    // Stop the motor
    m_krakenSub.runMotor(0.0);

    // Check to see if the measured current is good, ok or bad
    TestManager.Result ampsResult = m_testManager.determineResult(currentAmps, Constants.Tests.kDriveMotorExpectedAmps,
        Constants.Tests.kDriveMotorAmpsTolerance, Constants.Tests.kDriveMotorAmpsMinimum);
    String ampsText = "Amps=" + currentAmps + " (Target=" + Constants.Tests.kDriveMotorExpectedAmps + "+/-"
        + Constants.Tests.kDriveMotorAmpsTolerance + ")";
    System.out.println("KrakenSub " + ampsText);

    // Check to see if the measured position is good, ok or bad
    TestManager.Result positionResult =
        m_testManager.determineResult(currentPosition, Constants.Tests.kDriveMotorExpectedPosition,
            Constants.Tests.kDriveMotorPositionTolerance, Constants.Tests.kDriveMotorPositionMinimum);
    String positionText =
        "Position=" + currentPosition + " (Target=" + Constants.Tests.kDriveMotorExpectedPosition + "+/-"
            + Constants.Tests.kDriveMotorPositionTolerance + ")";
    System.out.println("KrakenSub " + positionText);

    // Figure out the overall test result
    TestManager.Result testResult = TestManager.Result.kPass;
    if((ampsResult == TestManager.Result.kFail) || (positionResult == TestManager.Result.kFail)) {
      testResult = TestManager.Result.kFail;
    } else if((ampsResult == TestManager.Result.kWarn) || (positionResult == TestManager.Result.kWarn)) {
      testResult = TestManager.Result.kWarn;
    }

    // Update the test results
    m_testManager.updateTestStatus(m_testId, testResult, ampsText + " " + positionText);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Check if we should abort
    if(m_abortTest) {
      return true;
    }
    // Otherwise, wait for the test time to have elapsed
    else if(Duration.between(m_startTime, Instant.now()).toMillis() > Constants.Tests.kDriveMotorTimeMs) {
      return true;
    }
    return false;
  }
}
