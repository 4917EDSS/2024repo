// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.VisionSub;

public class NoteVisionAlignCmd extends Command {
  /** Creates a new NoteVisionAlignCmd. */
  private final VisionSub m_visionSub;
  private final DrivetrainSub m_drivetrainSub;
  private final CommandPS4Controller m_driverController;

  private double noteAngle = 0.0;

  private final double drivePower = 0.7;

  private PIDController m_turnPID = new PIDController(0.006, 0.0, 0.0);

  public NoteVisionAlignCmd(VisionSub visionSub, DrivetrainSub drivetrainSub, CommandPS4Controller driverController) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_visionSub = visionSub;
    m_drivetrainSub = drivetrainSub;
    m_driverController = driverController;

    addRequirements(visionSub, drivetrainSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotationPower = 0.0;
    double setRotation = 0.0;

    noteAngle = m_visionSub.getNoteHorizontalAngle();
    if(m_visionSub.hasNoteTarget()) {
      rotationPower = m_turnPID.calculate(noteAngle);

      setRotation = m_drivetrainSub.getYawRotation2d().getDegrees() + noteAngle;

      double xPower = Math.cos(Math.toRadians(setRotation)) * drivePower;
      double yPower = Math.sin(Math.toRadians(setRotation)) * drivePower;

      m_drivetrainSub.drive(xPower, yPower, rotationPower, 0.02);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSub.drive(0.0, 0.0, 0.0, 0.02);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(!m_driverController.square().getAsBoolean() || !m_visionSub.hasNoteTarget())
      return true;
    return false;
  }
}
