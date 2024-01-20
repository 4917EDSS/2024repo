// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.time.Duration;
import java.time.Instant;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSub;

public class ShooterFlywheelCmd extends Command {
  private final ShooterSub m_ShooterSub;
  private Instant start;

  // PID Controllers

  private final PIDController m_FlyWheelPID = new PIDController(1.0, 0, 0); // TODO: Tune the Driving PID

  /** Creates a new FlywheelCmd. */
  public ShooterFlywheelCmd(ShooterSub shooterSub) {


    // Use addRequirements() here to declare subsystem dependencies.
    m_ShooterSub = shooterSub;
    addRequirements(shooterSub);
  }


  // Called when the command is initially scheduled. 
  @Override
  public void initialize() {
    start = Instant.now();
    m_ShooterSub.resetFlywheel();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double driveOutput = m_FlyWheelPID.calculate(m_ShooterSub.getFlywheelVelocity(), 10); //10 is a target velocity we don't know what it is
    m_ShooterSub.spinFlywheel(driveOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Instant end = Instant.now();
    Duration timeElapsed = Duration.between(start, end);
    return timeElapsed.toSeconds() > 5;
  }
}
