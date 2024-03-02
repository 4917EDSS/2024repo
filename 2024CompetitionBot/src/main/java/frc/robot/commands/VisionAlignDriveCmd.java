// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.VisionSub;

public class VisionAlignDriveCmd extends Command {
  /** Creates a new VisionAlignDriveCmd. */

  private final VisionSub m_visionSub;
  private final DrivetrainSub m_drivetrainSub;
  private final CommandPS4Controller m_driverController;

  private final PIDController m_lookatPID = new PIDController(0.005, 0.0, 0.0); // For facing apriltag

  public VisionAlignDriveCmd(DrivetrainSub drivetrainSub, VisionSub visionSub, CommandPS4Controller driverController) {

    m_visionSub = visionSub;
    m_drivetrainSub = drivetrainSub;
    m_driverController = driverController;

    addRequirements(visionSub, drivetrainSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Stage 1: Look at apriltag
    double horizontalOffset = m_visionSub.getHorizontalAngle();
    double rotationalPower = MathUtil.clamp(m_lookatPID.calculate(horizontalOffset, 0.0), -0.5, 0.5); // TODO: Can we just remove the second parameter?

    // m_drivetrainSub.drive(Math.abs(m_driverController.getLeftX()) < 0.05 ? 0.0
    //     : -m_driverController.getLeftX(),
    //     (Math.abs(m_driverController.getLeftY()) < 0.05 ? 0.0
    //         : m_driverController.getLeftY()),
    //     rotationalPower, 0.02);

    // Stage 2: Drive relative to apriltag
    Pose3d apriltagInfo = m_visionSub.getTarget3D();

    // Get apriltag angle relative to gyro in radians and constrain it
    // TODO: Make the angle and vector actually point towards the apriltag. It doesn't do that right now
    double fieldApriltagAngle = MathUtil.inputModulus(
        -apriltagInfo.getRotation().getY() + m_drivetrainSub.getRotation().getRadians(), 0.0, 2.0 * Math.PI);

    // Get apriltag angle as a vector. Setting drive x and y power to this would make it drive directly towards it
    double apriltagX = Math.cos(fieldApriltagAngle);
    double apriltagY = Math.sin(fieldApriltagAngle);

    // Print vector
    SmartDashboard.putNumber("ApriltagAX", apriltagX);
    SmartDashboard.putNumber("ApriltagAY", apriltagY);

    // Power towards and away from apriltag
    double forwardPowerX =
        apriltagX * Math.abs(m_driverController.getLeftY()) < 0.05 ? 0.0 : m_driverController.getLeftY();
    double forwardPowerY =
        apriltagY * Math.abs(m_driverController.getLeftY()) < 0.05 ? 0.0 : m_driverController.getLeftY();

    // Power from side to side around apriltag
    double sidePowerX =
        -apriltagY * Math.abs(m_driverController.getLeftX()) < 0.05 ? 0.0 : m_driverController.getLeftX();
    double sidePowerY =
        apriltagX * Math.abs(m_driverController.getLeftX()) < 0.05 ? 0.0 : m_driverController.getLeftX();

    // Vector Math may not be vectoring so make sure the value is clamped
    double xPower = MathUtil.clamp(forwardPowerX + sidePowerX, -1.0, 1.0);
    double yPower = MathUtil.clamp(forwardPowerY + sidePowerY, -1.0, 1.0);

    m_drivetrainSub.drive(xPower, yPower, rotationalPower, 0.02);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSub.drive(0.0, 0.0, 0.0, 0.02);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(!m_visionSub.hasTarget() || m_driverController.square().getAsBoolean() == false)
      return true;
    return false;
  }
}
