// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.VisionSub;

public class VisionAlignDriveCmd extends Command {
  // This command is specifically for automatically lining up infront of an apriltag and moving forwards and backwards from it

  private final CommandPS4Controller m_driverController;
  private final DrivetrainSub m_drivetrainSub;
  private final VisionSub m_visionSub;

  private final PIDController m_lookatPID = new PIDController(0.005, 0.0, 0.0); // For facing apriltag
  private final PIDController m_alignTagPID = new PIDController(0.01, 0, 0); // For aligning infront of apriltag

  private double angleSnapshot = 0.0;
  private boolean hasAngleSnapshot = false;

  public VisionAlignDriveCmd(CommandPS4Controller driverController, DrivetrainSub drivetrainSub, VisionSub visionSub) {
    m_driverController = driverController;
    m_drivetrainSub = drivetrainSub;
    m_visionSub = visionSub;

    addRequirements(drivetrainSub, visionSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_lookatPID.setTolerance(1.0);
    m_alignTagPID.setTolerance(1.0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Stage 1: Look at apriltag
    double horizontalOffset = m_visionSub.getSimpleHorizontalAngle();
    double rotationalPower = MathUtil.clamp(m_lookatPID.calculate(horizontalOffset), -0.5, 0.5);

    // Stage 2: Align with center of april tag

    // Get apriltag angle relative to gyro in radians and constrain it
    double fieldApriltagAngle = m_drivetrainSub.getYawRotation2d().getDegrees() + horizontalOffset;

    // Get apriltag angle as a vector. Setting drive x and y power to this would make it drive directly towards the apriltag
    double apriltagX = Math.cos(Math.toRadians(fieldApriltagAngle));
    double apriltagY = Math.sin(Math.toRadians(fieldApriltagAngle));


    // Power vectors in the direction of the apriltag
    double forwardPowerX =
        apriltagX * (Math.abs(m_driverController.getLeftY()) < 0.05 ? 0.0 : m_driverController.getLeftY());
    double forwardPowerY =
        apriltagY * (Math.abs(m_driverController.getLeftY()) < 0.05 ? 0.0 : m_driverController.getLeftY());


    // Get a snapshot of the apriltag angle so we can just update based on the gyroscope

    // Apriltag snapshot calculations - makes sure it only ever updates the offset if the data is invalidated (apriltag disappears)
    if(!m_visionSub.simpleHasTarget()) { // No targets? No snapshot
      hasAngleSnapshot = false;
    } else if(!hasAngleSnapshot) { // Set snapshot if there isn't currently any valid one (does once)
      angleSnapshot = MathUtil.inputModulus(
          m_drivetrainSub.getYawRotation2d().getDegrees() + m_visionSub.getTargetRotation().getY(), -180.0, 180.0);
      hasAngleSnapshot = true;
    }

    double calculatedSnapshot = angleSnapshot - m_drivetrainSub.getYawRotation2d().getDegrees();
    double powerApriltagAlign =
        MathUtil.clamp(m_alignTagPID.calculate(-calculatedSnapshot), -0.75,
            0.75);
    if(!m_visionSub.simpleHasTarget() || forwardPowerX * forwardPowerY != 0.0) { // Stop aligning if driving  
      powerApriltagAlign = 0.0;
    }

    // Power vectors from side to side around apriltag which will move the robot to align with the front of the tag
    double sidePowerX =
        -apriltagY * powerApriltagAlign;//(Math.abs(m_driverController.getLeftX()) < 0.05 ? 0.0 : m_driverController.getLeftX());
    double sidePowerY =
        apriltagX * powerApriltagAlign;//(Math.abs(m_driverController.getLeftX()) < 0.05 ? 0.0 : m_driverController.getLeftX());

    // Vector Math may not be vectoring so make sure the value is clamped
    double xPower = MathUtil.clamp(forwardPowerX + sidePowerX, -1.0, 1.0);
    double yPower = MathUtil.clamp(forwardPowerY + sidePowerY, -1.0, 1.0);

    //SmartDashboard.putNumber("Apriltag SnapY", angleSnapshot - m_drivetrainSub.getRotation().getDegrees());

    m_drivetrainSub.drive(-xPower, -yPower, rotationalPower, 0.02);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSub.drive(0.0, 0.0, 0.0, 0.02);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(/* !m_visionSub.simpleHasTarget() || */ m_driverController.square().getAsBoolean() == false) { // Command stops when button is released
      return true;
    }
    return false;
  }
}
