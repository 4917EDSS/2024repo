// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSub;

public class DriveFieldRelativeCmd extends Command {
  private final CommandPS4Controller m_driverController;
  private final DrivetrainSub m_drivetrainSub;

  private final double m_deadband = 0.07;

  private double m_targetHeading;

  public DriveFieldRelativeCmd(CommandPS4Controller commandPS4Controller, DrivetrainSub drivetrainSub) {
    m_drivetrainSub = drivetrainSub;
    m_driverController = commandPS4Controller;

    addRequirements(drivetrainSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_targetHeading = m_drivetrainSub.getYawRotationDegrees();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotationPower = 0;
    boolean driving =
        (Math.abs(m_driverController.getLeftX()) >= m_deadband
            || Math.abs(m_driverController.getLeftY()) >= m_deadband);
    if(Math.abs(m_driverController.getRightX()) < m_deadband) {
      if(driving) {
        rotationPower = m_drivetrainSub.getRotationPIDPowerDegrees(m_targetHeading);
      } else {
        rotationPower = 0.0;
        m_targetHeading = m_drivetrainSub.getYawRotationDegrees();
      }
    } else {
      rotationPower = -m_driverController.getRightX();
      m_targetHeading = m_drivetrainSub.getYawRotationDegrees();
    }
    if(m_driverController.cross().getAsBoolean()) { // Override robot rotation with target heading
      if(DriverStation.getAlliance().get() == Alliance.Blue) { // On blue side
        rotationPower = m_drivetrainSub.getLobRotationPower(58.0);
      } else {
        rotationPower = m_drivetrainSub.getLobRotationPower(122.0);
      }
    } else if(m_driverController.square().getAsBoolean()) {
      rotationPower = m_drivetrainSub.getLobRotationPower(90.0); // Angle facing drivers
    }

    // SmartDashboard.putNumber("rotationPower", rotationPower);

    // Hack to slow down the robot when novice drivers are using it.
    double leftX = m_driverController.getLeftX();
    double leftY = m_driverController.getLeftY();
    if(RobotContainer.m_demoMode) {
      leftX *= 0.3;
      leftY *= 0.3;
      rotationPower *= 0.3;

    }

    m_drivetrainSub.drive(
        (Math.abs(leftX) < m_deadband ? 0.0 : -leftX),
        (Math.abs(leftY) < m_deadband ? 0.0 : leftY),
        rotationPower,
        0.02); // this is the duration fo the timestep the speeds should be applied to. Should probably be changed
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSub.drive(0.0, 0.0, 0.0, 0.02);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
