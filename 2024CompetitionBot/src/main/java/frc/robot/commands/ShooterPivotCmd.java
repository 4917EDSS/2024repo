// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.logging.Logger;
import org.ejml.equation.IntegerSequence.Range;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSub;

public class ShooterPivotCmd extends Command {
  private static Logger m_logger = Logger.getLogger(ShooterPivotCmd.class.getName());


  // PID Controllers

  private final PIDController m_pivotForwardPid = new PIDController(1.0, 0, 0); // TODO: Tune the Driving PID

  private final ShooterSub m_ShooterSub;
  private final double m_targetPivotPosition;
  private final boolean m_forward;

  /** Creates a new PivotCmd. */
  public ShooterPivotCmd(ShooterSub shooterSub, double targetPivotPosition, boolean forward) {

    m_forward = forward;
    m_targetPivotPosition = targetPivotPosition;
    // Use addRequirements() here to declare subsystem dependencies.
    m_ShooterSub = shooterSub;
    addRequirements(shooterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ShooterSub.resetPivot();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
     * Step 1 power on the motor
     * Step 2 set position number
     * step 3 set direction
     * step 4 check
     * step 5 watch it move
     */


    int direction = (m_forward == true) ? 1 : -1; //if moving forward keep going forward, else multiply direction to -1
    final double driveOutput = m_pivotForwardPid.calculate(m_ShooterSub.getPivotVelocity(), 0.10 * direction); //10 is a target velocity we don't know what it is
    m_ShooterSub.movePivot(driveOutput);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double tolerance = 5;

    return Math.abs(m_targetPivotPosition - m_ShooterSub.getPivotPosition()) < tolerance;
  }
}
