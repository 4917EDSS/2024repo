// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// TODO change all "position" words to "power" because this test is for power not position

package frc.robot.commands.tests;

import java.time.Duration;
import java.time.Instant;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.utils.TestManager;
import frc.robot.utils.TestManager.Result;

public class TestSwerveDriveMotorsCmd extends Command {
  private final DrivetrainSub m_drivetrainSub;
  private final TestManager m_testManager;
  private final int[] m_testIds;
  private Instant m_startTime;

  /** Creates a new TestSwerveDriveMotorsCmd. */
  public TestSwerveDriveMotorsCmd(DrivetrainSub drivetrainsub, TestManager testManager) {
    m_drivetrainSub = drivetrainsub;
    m_testManager = testManager;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrainsub);

    m_testIds = new int[m_drivetrainSub.NUM_SWERVE_MODULES];
    m_testIds[m_drivetrainSub.MOTOR_FL] = m_testManager.registerNewTest("Drive FL");
    m_testIds[m_drivetrainSub.MOTOR_FR] = m_testManager.registerNewTest("Drive FR");
    m_testIds[m_drivetrainSub.MOTOR_BL] = m_testManager.registerNewTest("Drive BL");
    m_testIds[m_drivetrainSub.MOTOR_BR] = m_testManager.registerNewTest("Drive BR");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = Instant.now();
    m_drivetrainSub.testResetDriveEncoder(m_drivetrainSub.MOTOR_FL);
    m_drivetrainSub.testResetDriveEncoder(m_drivetrainSub.MOTOR_FR);
    m_drivetrainSub.testResetDriveEncoder(m_drivetrainSub.MOTOR_BL);
    m_drivetrainSub.testResetDriveEncoder(m_drivetrainSub.MOTOR_BR);

    m_drivetrainSub.testSetDriveMotorPower(m_drivetrainSub.MOTOR_FL, Constants.Tests.kDriveMotorPower);
    m_drivetrainSub.testSetDriveMotorPower(m_drivetrainSub.MOTOR_FR, Constants.Tests.kDriveMotorPower);
    m_drivetrainSub.testSetDriveMotorPower(m_drivetrainSub.MOTOR_BL, Constants.Tests.kDriveMotorPower);
    m_drivetrainSub.testSetDriveMotorPower(m_drivetrainSub.MOTOR_BR, Constants.Tests.kDriveMotorPower);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted) {
      m_drivetrainSub.testSetDriveMotorPower(m_drivetrainSub.MOTOR_FL, 0.0);
      m_drivetrainSub.testSetDriveMotorPower(m_drivetrainSub.MOTOR_FR, 0.0);
      m_drivetrainSub.testSetDriveMotorPower(m_drivetrainSub.MOTOR_BL, 0.0);
      m_drivetrainSub.testSetDriveMotorPower(m_drivetrainSub.MOTOR_BR, 0.0);
      m_testManager.updateTestStatus(m_testIds[m_drivetrainSub.MOTOR_FL], Result.kFail, "Test interrupted");
      m_testManager.updateTestStatus(m_testIds[m_drivetrainSub.MOTOR_FR], Result.kFail, "Test interrupted");
      m_testManager.updateTestStatus(m_testIds[m_drivetrainSub.MOTOR_BL], Result.kFail, "Test interrupted");
      m_testManager.updateTestStatus(m_testIds[m_drivetrainSub.MOTOR_BR], Result.kFail, "Test interrupted");
      return;
    }

    // Before turning off the motor, read the current current (amps) and position
    // SparkMax/NEO doesn't support reading amps
    // double[] currentAmps = {
    //     m_drivetrainSub.testGetDriveAmps(m_drivetrainSub.MOTOR_FL),
    //     m_drivetrainSub.testGetDriveAmps(m_drivetrainSub.MOTOR_FR),
    //     m_drivetrainSub.testGetDriveAmps(m_drivetrainSub.MOTOR_BL),
    //     m_drivetrainSub.testGetDriveAmps(m_drivetrainSub.MOTOR_BR)
    // };
    double[] currentPositions = {
        m_drivetrainSub.testGetDrivePosition(m_drivetrainSub.MOTOR_FL),
        m_drivetrainSub.testGetDrivePosition(m_drivetrainSub.MOTOR_FR),
        m_drivetrainSub.testGetDrivePosition(m_drivetrainSub.MOTOR_BL),
        m_drivetrainSub.testGetDrivePosition(m_drivetrainSub.MOTOR_BR)
    };

    // Stop the motor
    m_drivetrainSub.testSetDriveMotorPower(m_drivetrainSub.MOTOR_FL, 0.0);
    m_drivetrainSub.testSetDriveMotorPower(m_drivetrainSub.MOTOR_FR, 0.0);
    m_drivetrainSub.testSetDriveMotorPower(m_drivetrainSub.MOTOR_BL, 0.0);
    m_drivetrainSub.testSetDriveMotorPower(m_drivetrainSub.MOTOR_BR, 0.0);

    // Check to see if the measured position is good, ok or bad
    for(int motorId = 0; motorId <= m_drivetrainSub.NUM_SWERVE_MODULES; motorId++) {
      // Check to see if the measured current is good, ok or bad
      TestManager.Result ampsResult = Result.kPass; // Just pass it since we can't measure it
      // TestManager.Result ampsResult = m_testManager.determineResult(currentAmps[motorId], Constants.Tests.kDriveMotorExpectedAmps,
      //     Constants.Tests.kDriveMotorAmpsTolerance, Constants.Tests.kDriveMotorAmpsMinimum);
      // String ampsText = "Amps=" + currentAmps[motorId] + " (Target=" + Constants.Tests.kDriveMotorExpectedAmps + "+/-"
      //     + Constants.Tests.kDriveMotorAmpsTolerance + ")";
      String ampsText = "Amps not measureable";
      System.out.println("DrivetrainSub " + ampsText);

      TestManager.Result positionResult =
          m_testManager.determineResult(currentPositions[motorId], Constants.Tests.kDriveMotorExpectedPosition,
              Constants.Tests.kDriveMotorPositionTolerance, Constants.Tests.kDriveMotorPositionMinimum);
      String positionText =
          "Position=" + currentPositions[motorId] + " (Target=" + Constants.Tests.kDriveMotorExpectedPosition + "+/-"
              + Constants.Tests.kDriveMotorPositionTolerance + ")";
      System.out.println("DrivetrainSub " + positionText);

      // Figure out the overall test result
      TestManager.Result testResult = TestManager.Result.kPass;
      if((ampsResult == TestManager.Result.kFail) || (positionResult == TestManager.Result.kFail)) {
        testResult = TestManager.Result.kFail;
      } else if((ampsResult == TestManager.Result.kWarn) || (positionResult == TestManager.Result.kWarn)) {
        testResult = TestManager.Result.kWarn;
      }

      // Update the test results
      m_testManager.updateTestStatus(m_testIds[motorId], testResult, ampsText + " " + positionText);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Wait for the test time to have elapsed
    if(Duration.between(m_startTime, Instant.now()).toMillis() > Constants.Tests.kDriveMotorTimeMs) {
      return true;
    }
    return false;
  }
}
