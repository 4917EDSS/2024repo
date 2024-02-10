// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.DrivetrainSub.Path;

public class DrivePathCmd extends Command {
  /** Creates a new DrivePathCmd. */
  private final DrivetrainSub m_drivetrainSub;
  private Path daPath;
  private boolean pathStatus = false;

  public DrivePathCmd(DrivetrainSub drivetrainSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrainSub);
    m_drivetrainSub = drivetrainSub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    daPath = m_drivetrainSub.generateTestPath();
    m_drivetrainSub.startPath(daPath);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pathStatus = m_drivetrainSub.runPath(daPath);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSub.drive(0.0, 0.0, 0.0, 0.02);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathStatus;
  }
}
