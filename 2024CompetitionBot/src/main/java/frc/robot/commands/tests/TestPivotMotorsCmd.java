// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tests;

import java.time.Duration;
import java.time.Instant;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PivotSub;
import frc.robot.utils.TestManager;
import frc.robot.utils.TestManager.Result;

public class TestPivotMotorsCmd extends Command {
  private final PivotSub m_pivotSub;
  private final TestManager m_testManager;
  private final int m_testId;
  private Instant m_startTime;

  /** Creates a new TestPivotMotorsCmd. */
  public TestPivotMotorsCmd(PivotSub pivotsub, TestManager testManager) {
    m_pivotSub = pivotsub;
    m_testManager = testManager;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivotsub);
    m_testId = m_testManager.registerNewTest("Pivot Motor");

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = Instant.now();
    m_pivotSub.testMovePivot(Constants.Tests.kPivotMotorPower);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  // when the time ends
  @Override
  public void end(boolean interrupted) {
    if(interrupted) {
      m_pivotSub.testMovePivot(0);
      m_testManager.updateTestStatus(m_testId, Result.kFail, "Test interrupted"); // this is if the test gets interrupted
      return;
    }

    double currentAngle = m_pivotSub.getPivotAngle(); // get the angle

    m_pivotSub.testMovePivot(0); // Stop the motor


    // Check to see if the measured angle is good, ok or bad
    TestManager.Result angleResult =
        m_testManager.determineResult(currentAngle, Constants.Tests.kPivotMotorExpectedPosition,
            Constants.Tests.kPivotMotorPositionTolerance, Constants.Tests.kPivotMotorPositionMinimum);
    String angleText =
        "Position=" + currentAngle + "°" + " (Target=" + Constants.Tests.kPivotMotorExpectedPosition + "°" + "+/-"
            + Constants.Tests.kPivotMotorPositionTolerance + "°" + ")"; // create a string for motor (angle) information
    System.out.println("PivotSub " + angleResult); /*
                                                    * print out the motor (angle) information into the console, just
                                                    * in case the Shuffleboard ain't working
                                                    */

    // Figure out the overall test result
    TestManager.Result testResult = TestManager.Result.kPass;
    if(angleResult == TestManager.Result.kFail) {
      testResult = TestManager.Result.kFail;
    } else if(angleResult == TestManager.Result.kWarn) {
      testResult = TestManager.Result.kWarn;
    }

    // Update the test results
    m_testManager.updateTestStatus(m_testId, testResult, angleText);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Wait for the test time to have elapsed
    if(Duration.between(m_startTime, Instant.now()).toMillis() > Constants.Tests.kPivotMotorTimeMs) {
      return true;
    }
    return false;
  }
}
