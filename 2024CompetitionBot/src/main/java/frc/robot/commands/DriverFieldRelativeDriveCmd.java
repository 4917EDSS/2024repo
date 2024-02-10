// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.DrivetrainSub;

public class DriverFieldRelativeDriveCmd extends Command {
  private final DrivetrainSub m_drivetrainSub;
  private final CommandPS4Controller m_driverController;
  private double m_targetHeading;

  /** Creates a new DriverFeildRelativeDriveCmd. */
  public DriverFieldRelativeDriveCmd(DrivetrainSub drivetrainSub, CommandPS4Controller commandPS4Controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrainSub);
    m_drivetrainSub = drivetrainSub;
    m_driverController = commandPS4Controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_targetHeading = m_drivetrainSub.getRotationDegrees();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotationPower = 0;
    if(Math.abs(m_driverController.getRightX()) < 0.07) {
      rotationPower = m_drivetrainSub.getRotationPIDPowerDegrees(m_targetHeading);
    } else {
      rotationPower = -m_driverController.getRightX();
      m_targetHeading = m_drivetrainSub.getRotationDegrees();
    }
    m_drivetrainSub.drive(
        (Math.abs(m_driverController.getLeftX()) < 0.07 ? 0.0
            : m_driverController.getLeftX()),
        (Math.abs(m_driverController.getLeftY()) < 0.07 ? 0.0
            : -m_driverController.getLeftY()),
        rotationPower,
        0.02); // this is the duration fo thh timestep the speeds should be applied to. Should probably be changed
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
