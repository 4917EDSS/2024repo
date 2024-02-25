// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.time.Duration;
import java.time.Instant;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSub;

public class ShooterFlywheelCmd extends Command {

  private final ShooterSub m_shooterSub;
  private final double m_targetShooterVelocity;
  private Instant start;

  // PID Controllers
  private final PIDController m_FlyWheelPID = new PIDController(1.0, 0, 0); // TODO: Tune the Driving PID

  /** Creates a new FlywheelCmd. */
  public ShooterFlywheelCmd(double targetShooterVelocity, ShooterSub shooterSub) {


    // Use addRequirements() here to declare subsystem dependencies.
    m_shooterSub = shooterSub;
    m_targetShooterVelocity = targetShooterVelocity;
  }


  // Called when the command is initially scheduled. 
  @Override
  public void initialize() {
    start = Instant.now();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
     * Step 1: Set target velocity for flywheel speed + inverted or forward
     * Step 2: Start flywheel
     * note - Step 1 and 2 might need to be swapped
     * Step 3: check if note is there (to see if note has left the storage) if it has go step 4, else continue
     * Step 4: Stop fly wheel
     * 
     * 
     */
    double driveOutput = m_FlyWheelPID.calculate(m_shooterSub.getFlywheelVelocity(), m_targetShooterVelocity); //4200 is a target velocity we don't know what it is
    m_shooterSub.spinFlywheel(driveOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Instant end = Instant.now();
    Duration timeElapsed = Duration.between(start, end);
    return timeElapsed.toSeconds() > 2;
  }
}
