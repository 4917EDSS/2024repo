// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tests;

import java.time.Instant;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.utils.TestManager;
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
    //m_feederSub.testIntakeRollers(Constants.Tests.kIntakeRollers);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted) {
      //test
    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
