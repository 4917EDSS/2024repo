// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
// import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.DrivetrainSub;

public class RotationalVelocityCharacterization extends Command {
  /** Creates a new VelocityCharacterization. */
  private final DrivetrainSub m_drivetrainSub;
  double velocity;
  int index = 0;
  double buffer[] = new double[50];
  double averageVelocity;
  double maxVelocity = 0;
  int p = 0;
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  public RotationalVelocityCharacterization(DrivetrainSub drivetrainSub) {
    addRequirements(drivetrainSub);
    m_drivetrainSub = drivetrainSub;

  }

  @Override
  public void initialize() {
    //SmartDashboard.putNumberArray("Buffer", buffer);
    SmartDashboard.putNumber("Rotational Velocity", velocity);
    SmartDashboard.putNumber("Average Rotational Velocity", averageVelocity);
    for(int i = 0; i < 25; i++) {
      m_drivetrainSub.drive(0.0, 0.0, 1.0, 0.02);
    }
    new WaitCommand(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    velocity = m_gyro.getRate();
    SmartDashboard.putNumber("Rotational Velocity", velocity);
    SmartDashboard.putNumber("Average Rotational Velocity", averageVelocity);
    if(velocity > maxVelocity) {
      maxVelocity = velocity;
    }

    m_drivetrainSub.drive(0.0, 0.0, 1.0, 0.02);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Reached End condition");
    m_drivetrainSub.drive(0.0, 0.0, 0.0, 0.02);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    buffer[index] = velocity;
    if(index < 49) {
      index += 1;
    } else {
      index = 0;
    }
    for(int i = 0; i < 50; i++) {
      averageVelocity += buffer[i];
    }
    if(Math.abs(averageVelocity - velocity) < 1) {
      SmartDashboard.putNumber("Max Rotational Velocity", maxVelocity);
      return true;
    } else {
      averageVelocity = 0;
      return false;
    }
  }
}
