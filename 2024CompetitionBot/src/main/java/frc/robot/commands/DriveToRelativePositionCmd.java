// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.logging.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.DrivetrainSub;

public class DriveToRelativePositionCmd extends Command {
  private static Logger m_logger = Logger.getLogger(DriveToRelativePositionCmd.class.getName());
  /** Creates a new DriveToPositionCmd. */

  private final DrivetrainSub m_drivetrainSub;
  private Pose2d m_relativePosition;
  private boolean atSetpoint = false;

  public DriveToRelativePositionCmd(DrivetrainSub drivetrainSub, Pose2d position) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrainSub);
    m_drivetrainSub = drivetrainSub;
    m_relativePosition = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d m_position = new Pose2d(m_relativePosition.getX() + m_drivetrainSub.getPos().getX(),
        m_relativePosition.getY() + m_drivetrainSub.getPos().getY(),
        m_relativePosition.getRotation().plus(m_drivetrainSub.getRotation())); // Make position relative to robot position
    // Resetting odometry is not a good idea m_drivetrainSub.resetOdometry();
    m_drivetrainSub.translateOdometry(m_position); // Set tranlation position
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
    return atSetpoint;
  }
}
