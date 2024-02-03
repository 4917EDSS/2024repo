// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivetrainSub;

public class VelocityCharacterization extends Command {
  /** Creates a new VelocityCharacterization. */
  private final DrivetrainSub m_drivetrainSub;
  double velocity;
  int index = 0;
  double buffer[] = new double[50];
  double averageVelocity;
  double maxVelocity = 0;
  int p = 0;

  public VelocityCharacterization(DrivetrainSub drivetrainSub) {
    addRequirements(drivetrainSub);
    m_drivetrainSub = drivetrainSub;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //for(int j = 0; j < 100; j++) {
    //System.out.println("Looped");
    //new WaitCommand(1);
    //m_drivetrainSub.drive(0.0, 0.5, 0.0, 0.02);
    //}
    //new WaitCommand(1.0);
    //System.out.println("Finished waiting");
    //m_drivetrainSub.setMotorPower(1);
    //System.out.println("It worked");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    velocity = m_drivetrainSub.getVelocity();
    SmartDashboard.putNumber("Velocity", velocity);
    if(velocity > maxVelocity) {
      maxVelocity = velocity;
    }
    m_drivetrainSub.drive(0.0, 0.5, 0.0, 0.02);
    p++;
    SmartDashboard.putNumber("Drive Enabled", p);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSub.setMotorPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    buffer[index] = velocity;
    if(index < 50) {
      index += 1;
    } else {
      index = 0;
    }
    for(int i = 0; i < 50; i++) {
      averageVelocity += buffer[i];
    }
    if(Math.abs(averageVelocity - velocity) < 1) {
      SmartDashboard.putNumber("Max Velocity", maxVelocity);
      return true;
    } else {
      return false;
    }
  }
}
