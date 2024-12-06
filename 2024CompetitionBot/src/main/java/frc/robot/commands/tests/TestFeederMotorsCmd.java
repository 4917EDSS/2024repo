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
    m_testId1 = m_testManager.registerNewTest("Upper Feeder");
    m_testId2 = m_testManager.registerNewTest("Lower Feeder");

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = Instant.now();
    m_feederSub.testIntakeMotorPower(Constants.Tests.kIntakeRollers);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted) {
      m_feederSub.testIntakeMotorPower(0);
      m_testManager.updateTestStatus(m_testId1, Result.kFail, "Test interrupted");
      m_testManager.updateTestStatus(m_testId2, Result.kFail, "Test interrupted");
    }

    double[] currentVelocities = {
        m_feederSub.getUpperIntakeRollerVelocity(),
        m_feederSub.getLowerIntakeRollerVelocity()
    };
    TestManager.Result velocityResult =
        m_testManager.determineResult(m_testId1, Constants.Tests.kIntakeMotorExpectedVelocity,
            Constants.Tests.kIntakeMotorVelocityTolerance, Constants.Tests.kIntakeMotorVelocityMinimum);
    m_testManager.determineResult(m_testId2, Constants.Tests.kIntakeMotorExpectedVelocity,
        Constants.Tests.kIntakeMotorVelocityTolerance, Constants.Tests.kIntakeMotorVelocityMinimum);

    String velocityText =
        "Velocity=" + currentVelocities[m_testId1] + currentVelocities[m_testId2] + " (Target="
            + Constants.Tests.kIntakeMotorExpectedVelocity
            + "+/-"

            + Constants.Tests.kIntakeMotorExpectedVelocity + ")";
    System.out.println("DrivetrainSub " + velocityText);


    TestManager.Result testResult = TestManager.Result.kPass;
    if((velocityResult == TestManager.Result.kFail)) {
      testResult = TestManager.Result.kFail;
    } else if((velocityResult == TestManager.Result.kWarn)) {
      testResult = TestManager.Result.kWarn;
    }

    m_testManager.updateTestStatus(m_testId1, testResult, velocityText);
    m_testManager.updateTestStatus(m_testId2, testResult, velocityText);
  }
  // Update the test results

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Duration.between(m_startTime, Instant.now()).toMillis() > Constants.Tests.kIntakeMotorTimeMs)
      return true;

    return false;
  }


}

