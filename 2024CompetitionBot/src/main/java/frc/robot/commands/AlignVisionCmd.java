// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.FlywheelSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.VisionSub;

public class AlignVisionCmd extends Command {
  /** Creates a new AlignVisionCmd. */

  private final VisionSub m_visionSub;
  private final DrivetrainSub m_drivetrainSub;
  private final CommandPS4Controller m_driverController;
  private final CommandPS4Controller m_operatorController;
  private final ShooterSub m_shooterSub;
  private final FlywheelSub m_flywheelSub;

  private static final double kA = -0.86166;
  private static final double kB = 51.7571;

  private final PIDController m_lookatPID = new PIDController(0.005, 0.0, 0.0); // For facing apriltag

  public AlignVisionCmd(DrivetrainSub drivetrainSub, VisionSub visionSub, ShooterSub shooterSub,
      FlywheelSub flywheelSub,
      CommandPS4Controller driverController, CommandPS4Controller operatorController) {
    m_visionSub = visionSub;
    m_drivetrainSub = drivetrainSub;
    m_shooterSub = shooterSub;
    m_flywheelSub = flywheelSub;

    m_driverController = driverController;
    m_operatorController = operatorController;

    addRequirements(visionSub, drivetrainSub, shooterSub, flywheelSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_lookatPID.setTolerance(1.0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Stage 1: Look at apriltag
    double horizontalOffset = m_visionSub.getSimpleHorizontalAngle();
    double verticalAngle = m_visionSub.getSimpleVerticalAngle();
    double rotationalPower = MathUtil.clamp(m_lookatPID.calculate(horizontalOffset), -0.5, 0.5); // TODO: Can we just remove the second parameter?
    double xPower = Math.abs(m_driverController.getLeftX()) < 0.05 ? 0.0 : m_driverController.getLeftX();
    double yPower = Math.abs(m_driverController.getLeftY()) < 0.05 ? 0.0 : m_driverController.getLeftY();

    double pivotAngle = kA * verticalAngle + kB; // Simple linear conversion from apriltag angle to shooter angle

    if(m_visionSub.simpleHasTarget()) {
      m_shooterSub.runPivotControl(pivotAngle);
      m_flywheelSub.enableFlywheel();

      if(m_operatorController.R2().getAsBoolean()) {
        m_shooterSub.spinBothFeeders(1.0, 1.0);
      } else {
        double feederPower =
            Math.abs(m_operatorController.getRightY()) < 0.05 ? 0.0 : -m_operatorController.getRightY();
        m_shooterSub.spinBothFeeders(feederPower, 0.5 * feederPower);
      }
    } else {
      rotationalPower = Math.abs(m_driverController.getRightX()) < 0.05 ? 0.0 : m_driverController.getRightX();
      m_flywheelSub.disableFlywheel();
    }


    m_drivetrainSub.drive(-xPower, yPower, rotationalPower, 0.02);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSub.drive(0.0, 0.0, 0.0, 0.02);
    m_flywheelSub.disableFlywheel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_driverController.square().getAsBoolean() == false) { // Command stops when button is released
      return true;
    }
    return false;
  }
}
