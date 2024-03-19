// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.logging.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.FeederSub;
import frc.robot.subsystems.FlywheelSub;
import frc.robot.subsystems.LedSub;
import frc.robot.subsystems.LedSub.LedColour;
import frc.robot.subsystems.LedSub.LedZones;
import frc.robot.subsystems.PivotSub;
import frc.robot.subsystems.VisionSub;

public class AlignVisionCmd extends Command {
  private static Logger m_logger = Logger.getLogger(AlignVisionCmd.class.getName());
  private static final double kRotationTolerance = 1.0;

  private final VisionSub m_visionSub;
  private final DrivetrainSub m_drivetrainSub;
  private final CommandPS4Controller m_driverController;
  private final CommandPS4Controller m_operatorController;
  private final PivotSub m_pivotSub;
  private final FeederSub m_feederSub;
  private final FlywheelSub m_flywheelSub;
  private final LedSub m_ledSub;

  private final PIDController m_lookatPID = new PIDController(0.007, 0.0, 0.009); // For facing apriltag

  public AlignVisionCmd(CommandPS4Controller driverController, CommandPS4Controller operatorController,
      DrivetrainSub drivetrainSub, FeederSub feederSub, FlywheelSub flywheelSub, LedSub ledSub, PivotSub pivotSub,
      VisionSub visionSub) {
    m_driverController = driverController;
    m_operatorController = operatorController;

    m_drivetrainSub = drivetrainSub;
    m_feederSub = feederSub;
    m_flywheelSub = flywheelSub;
    m_ledSub = ledSub;
    m_pivotSub = pivotSub;
    m_visionSub = visionSub;

    addRequirements(drivetrainSub, feederSub, flywheelSub, pivotSub, visionSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_lookatPID.setTolerance(kRotationTolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Stage 1: Look at apriltag
    double horizontalOffset = m_visionSub.getSimpleHorizontalAngle();
    double verticalAngle = m_visionSub.getSimpleVerticalAngle();
    double rotationalPower = 0.0;
    double xPower = Math.abs(m_driverController.getLeftX()) < 0.07 ? 0.0 : m_driverController.getLeftX();
    double yPower = Math.abs(m_driverController.getLeftY()) < 0.07 ? 0.0 : m_driverController.getLeftY();

    double pivotAngle = m_pivotSub.interpolateShooterAngle(verticalAngle); // Simple linear conversion from apriltag angle to shooter angle
    boolean hasTarget = m_visionSub.simpleHasTarget();


    if(hasTarget) {
      // if(xPower * xPower + yPower * yPower > 0) {
      //   gyroAngleSet = false;
      // }
      // if(!gyroAngleSet) {
      //   gyroAngleOffset = m_drivetrainSub.getYawRotationDegrees() - horizontalOffset;
      //   gyroAngleSet = true;
      // }
      rotationalPower =
          MathUtil.clamp(m_lookatPID.calculate(horizontalOffset, 0.0), -0.5,
              0.5);
      //rotationalPower += kTurnFedPower * Math.signum(rotationalPower);
      m_pivotSub.setTargetAngle(pivotAngle);
      m_flywheelSub.enableFlywheel();

      double feederPower =
          Math.abs(m_operatorController.getRightY()) < 0.05 ? 0.0 : -m_operatorController.getRightY();
      m_feederSub.spinBothFeeders(feederPower, 0.5 * feederPower);
      if(m_lookatPID.atSetpoint()) {
        rotationalPower = 0.0;
      }

    } else {
      rotationalPower = Math.abs(m_driverController.getRightX()) < 0.05 ? 0.0 : -m_driverController.getRightX();
      m_flywheelSub.disableFlywheel();
    }


    m_drivetrainSub.drive(-xPower, yPower, rotationalPower, 0.02);
    if(m_flywheelSub.isAtTargetVelocity() && m_lookatPID.atSetpoint() && m_pivotSub.isAtPivotAngle()) {
      m_ledSub.setZoneColour(LedZones.ALL, LedColour.BLUE);
      m_logger.fine("Shot with target pivot angle: " + pivotAngle);
      m_logger.fine("Actual pivot angle: " + m_pivotSub.getPivotAngle());
      m_logger.fine("Flywheel speed: " + m_flywheelSub.getFlywheelVelocityL());
    } else if(hasTarget) {
      m_ledSub.setZoneColour(LedZones.ALL, LedColour.YELLOW);
    } else {
      m_ledSub.setZoneColour(LedZones.ALL, LedColour.RED);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSub.drive(0.0, 0.0, 0.0, 0.02);
    if(interrupted) {
      m_flywheelSub.disableFlywheel();
    }
    m_ledSub.setZoneColour(LedZones.ALL, LedColour.ORANGE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_flywheelSub.isAtTargetVelocity() && m_lookatPID.atSetpoint() && m_pivotSub.isAtPivotAngle()) {
      return true;
    }
    return false;
  }


}
