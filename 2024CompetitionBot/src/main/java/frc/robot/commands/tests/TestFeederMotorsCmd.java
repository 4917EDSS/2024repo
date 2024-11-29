// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tests;

import java.time.Duration;
import java.time.Instant;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.utils.TestManager;
import frc.robot.utils.TestManager.Result;
import frc.robot.subsystems.FeederSub;


public class TestFeederMotorsCmd extends Command {
  private final FeederSub m_feederSub;
  private final TestManager m_testManager;
  private final int m_testId1;
  private final int m_testId2;
  private Instant m_startTime;

  /** Creates a new TestFeederMotors. */
  public TestFeederMotorsCmd(FeederSub feederSub, TestManager testManager) {
    m_feederSub = feederSub;
    m_testManager = testManager;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feederSub);
    m_testId1 = m_testManager.registerNewTest("Feeder 1");
    m_testId2 = m_testManager.registerNewTest("Feeder 2");

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = Instant.now();
    m_feederSub.testIntakeRollers(Constants.Tests.kIntakeRollers);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted) {
      m_feederSub.testIntakeRollers(0);
      m_testManager.updateTestStatus(m_testId1, Result.kFail, "Test interrupted");
      m_testManager.updateTestStatus(m_testId2, Result.kFail, "Test interrupted");
    }
    double[] currentPositions = {
        m_feederSub.getLowerFeeder(),
        m_feederSub.getUpperFeeder()
    };

    TestManager.Result positionResult =
        m_testManager.determineResult(m_testId1, Constants.Tests.kIntakeRollerExpectedPosition,
            Constants.Tests.kIntakeRollerPositionTolerance, Constants.Tests.kIntakeRollerPositionMinimum);


    TestManager.Result positionResult2 =
        m_testManager.determineResult(m_testId2, Constants.Tests.kIntakeRollerExpectedPosition,
            Constants.Tests.kIntakeRollerPositionTolerance, Constants.Tests.kIntakeRollerPositionMinimum);


    String positionText =
        "Position=" + currentPositions[m_testId1] + " (Target=" + Constants.Tests.kIntakeRollerExpectedPosition + "+/-"
            + Constants.Tests.kIntakeRollerPositionTolerance + ")";
    System.out.println("FeederSub " + positionText);

    TestManager.Result testResult = TestManager.Result.kPass;
    if((positionResult == TestManager.Result.kFail)) {
      testResult = TestManager.Result.kFail;
    } else if((positionResult == TestManager.Result.kWarn)) {
      testResult = TestManager.Result.kWarn;
    }


    // Update the test results
    m_testManager.updateTestStatus(m_testId1, testResult, positionText);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    {
      if(Duration.between(m_startTime, Instant.now()).toMillis() > Constants.Tests.kIntakeRollerTimeMs) {
        return true;
      }
      return false;
    }

  }

}

