// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.DrivetrainSub;

public class DriveToRelativePositionCmd extends Command {
  /** Creates a new DriveToPositionCmd. */

  private final DrivetrainSub m_drivetrainSub;
  private Translation2d m_position;
  private boolean atSetpoint = false;

  public DriveToRelativePositionCmd(DrivetrainSub drivetrainSub, Translation2d position) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrainSub);
    m_drivetrainSub = drivetrainSub;
    m_position = position;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_position = new Translation2d(m_position.getX() + m_drivetrainSub.getPos().getX(),
        m_position.getY() + m_drivetrainSub.getPos().getY()); // Make position relative to robot position
    // Resetting odometry is not a good idea m_drivetrainSub.resetOdometry();
    m_drivetrainSub.translateOdometry(m_position); // Set tranlation position
    new PrintCommand("Starting tralsnation");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    atSetpoint = m_drivetrainSub.updateOdometryTransform();
    new PrintCommand("MOVING tralsnation");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSub.drive(0.0, 0.0, 0.0, 0.02);
    new PrintCommand("Finished AutoPos");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
