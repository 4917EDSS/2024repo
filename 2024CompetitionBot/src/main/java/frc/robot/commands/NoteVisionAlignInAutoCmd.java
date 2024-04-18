// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.time.Duration;
import java.time.Instant;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.VisionSub;

public class NoteVisionAlignInAutoCmd extends Command {
  /** Creates a new NoteVisionAlignCmd. */
  private final VisionSub m_visionSub;
  private final DrivetrainSub m_drivetrainSub;


  private double noteAngle = 0.0;

  private final double drivePower = 0.5;

  private PIDController m_turnPID = new PIDController(0.01, 0.0, 0.0);

  Instant start;
  //boolean hasSeenNote;
  //boolean goingForSecondNote;

  public NoteVisionAlignInAutoCmd(VisionSub visionSub, DrivetrainSub drivetrainSub) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_visionSub = visionSub;
    m_drivetrainSub = drivetrainSub;

    addRequirements(visionSub, drivetrainSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    start = Instant.now();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotationPower = 0.0;
    double setRotation = 0.0;

    noteAngle = m_visionSub.getNoteHorizontalAngle();
    if(m_visionSub.hasNoteTarget()) {
      // hasSeenNote = true;
      rotationPower = m_turnPID.calculate(noteAngle);

      setRotation = m_drivetrainSub.getYawRotation2d().getDegrees() + noteAngle;

      double slowDownToTurn = (100 - Math.abs(noteAngle)) / 100;
      double xPower = Math.cos(Math.toRadians(setRotation)) * drivePower * slowDownToTurn;
      double yPower = Math.sin(Math.toRadians(setRotation)) * drivePower * slowDownToTurn;

      m_drivetrainSub.drive(xPower, yPower, rotationPower, 0.02);

      start = Instant.now();
    }
    // } else if(goingForSecondNote){
    //   m_drivetrainSub.drive(0, 0, 0.2, 0.02);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSub.drive(0.0, 0.0, 0.0, 0.02);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Duration.between(start, Instant.now()).toMillis() > 150) {
      // if(!hasSeenNote){
      //  goingForSecondNote = true;
      // }
      return true;
    }
    return false;
  }
}
