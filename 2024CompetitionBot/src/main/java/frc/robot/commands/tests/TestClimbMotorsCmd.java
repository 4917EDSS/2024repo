// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tests;

import java.time.Duration;
import java.time.Instant;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSub;
import frc.robot.utils.TestManager;
import frc.robot.utils.TestManager.Result;

public class TestPivotMotorsCmd extends Command {
  private final ClimbSub m_climbSub;
  private final TestManager m_testManager;
  private final int m_testId;
  private Instant m_startTime;

  /** Creates a new TestClimbMotorsCmd. */
  public TestClimbMotorsCmd(ClimbSub climbsub, TestManager testManager) {
    m_climbSub = climbsub;
    m_testManager = testManager;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climbsub);
    m_testIds = new int[2];
    m_testIds[0] = m_testManager.registerNewTest("Climb L");
    m_testIds[1] = m_testManager.registerNewTest("Climb R");

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = Instant.now();
    m_climbSub.testClimbMotorPower(Constants.Tests.kClimbMotorPower);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  // when the time ends
  @Override
  public void end(boolean interrupted) {
    if(interrupted) {
      m_climbSub.testClimbMotorPower(0);

      // this is if the test gets interrupted
      m_testManager.updateTestStatus(m_testIds[0], Result.kFail, "Test interrupted"); 
      m_testManager.updateTestStatus(m_testIds[1], Result.kFail, "Test interrupted");
    }
      return;
    }

    // get the positions
    double[] currentPositions = {
      m_climbSub.getLeftHeight(),
      m_climbSub.getRightHeight()
    };

    m_climbSub.testClimbMotorPower(0); // Stop the motor


    // Check to see if the measured position is good, ok or bad
    for(int motorId = 0; motorId <= m_testIds.length; motorId++) {
      TestManager.Result positionResult =
          m_testManager.determineResult(m_testIds[motorId], Constants.Tests.kClimbMotorExpectedPosition,
              Constants.Tests.kClimbMotorPositionTolerance, Constants.Tests.kClimbMotorPositionMinimum);

      String positionText =
          "Position=" + currentPositions[motorId] + " (Target=" + Constants.Tests.kClimbMotorExpectedPosition + "+/-"
              + Constants.Tests.kClimbMotorPositionTolerance + ")";
      System.out.println("ClimbSub " + positionText);


    // Figure out the overall test result
    TestManager.Result testResult = TestManager.Result.kPass;
      if((positionResult == TestManager.Result.kFail)) {
        testResult = TestManager.Result.kFail;
      } else if((positionResult == TestManager.Result.kWarn)) {
        testResult = TestManager.Result.kWarn;
      }
      // Update the test results
      m_testManager.updateTestStatus(m_testIds[motorId], testResult, positionText);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Wait for the test time to have elapsed
    if(Duration.between(m_startTime, Instant.now()).toMillis() > Constants.Tests.kClimbMotorTimeMs) {
      return true;
    }
    return false;
  }
}
