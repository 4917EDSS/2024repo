// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tests;

import java.time.Duration;
import java.time.Instant;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FlywheelSub;
import frc.robot.utils.TestManager;
import frc.robot.utils.TestManager.Result;

public class TestFlywheelMotorsCmd extends Command {
  public final FlywheelSub m_flywheelSub;
  public final TestManager m_testManager;
  private Instant m_startTime;
  private final int[] m_testIds;

  /** Creates a new TestFlywheelMotorsCmd. */
  public TestFlywheelMotorsCmd(FlywheelSub flywheelSub, TestManager testManager) {

    m_testManager = testManager;
    m_flywheelSub = flywheelSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(flywheelSub);
    m_testIds = new int[2];
    m_testIds[0] = m_testManager.registerNewTest("Flywheel L");
    m_testIds[1] = m_testManager.registerNewTest("Flywheel R");

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = Instant.now();
    m_flywheelSub.testFlywheelMotorPower(Constants.Tests.kFlywheelMotorPower);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted) {
      m_flywheelSub.testFlywheelMotorPower(0);
      m_testManager.updateTestStatus(m_testIds[0], Result.kFail, "Test interrupted");
      m_testManager.updateTestStatus(m_testIds[1], Result.kFail, "Test interrupted");
      return;
    }
    double[] currentVelocities = {
        m_flywheelSub.getFlywheelVelocityL(),
        m_flywheelSub.getFlywheelVelocityR()
    };
    // Check to see if the measured velocity is good, ok or bad
    for(int motorId = 0; motorId < m_testIds.length; motorId++) {
      TestManager.Result velocityResult =
          m_testManager.determineResult(m_testIds[motorId], Constants.Tests.kFlywheelMotorExpectedVelocity,
              Constants.Tests.kFlywheelMotorVelocityTolerance, Constants.Tests.kFlywheelMotorVelocityMinimum);

      String velocityText =
          "Velocity=" + currentVelocities[motorId] + " (Target=" + Constants.Tests.kFlywheelMotorExpectedVelocity
              + "+/-"
              + Constants.Tests.kFlywheelMotorVelocityTolerance + ")";
      System.out.println("DrivetrainSub " + velocityText);

      // Figure out the overall test result
      TestManager.Result testResult = TestManager.Result.kPass;
      if((velocityResult == TestManager.Result.kFail)) {
        testResult = TestManager.Result.kFail;
      } else if((velocityResult == TestManager.Result.kWarn)) {
        testResult = TestManager.Result.kWarn;
      }
      // Update the test results
      m_testManager.updateTestStatus(m_testIds[motorId], testResult, velocityText);
    }


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Duration.between(m_startTime, Instant.now()).toMillis() > Constants.Tests.kFlywheelMotorTimeMs) {
      return true;
    }
    return false;
  }
}
