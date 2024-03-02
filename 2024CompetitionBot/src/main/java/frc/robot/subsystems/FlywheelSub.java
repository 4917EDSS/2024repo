// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.logging.Logger;

public class FlywheelSub extends SubsystemBase {
  /** Creates a new FlywheelSub. */

  private final CANSparkMax m_flywheelL =
      new CANSparkMax(Constants.CanIds.kFlywheelL, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_flywheelR =
      new CANSparkMax(Constants.CanIds.kFlywheelR, CANSparkLowLevel.MotorType.kBrushless);
  // PID Controllers
  private final PIDController m_FlyWheelPID = new PIDController(1.0, 0, 0); // TODO: Tune the Driving PID

  private final SimpleMotorFeedforward m_FlyWheelFeedforward =
      new SimpleMotorFeedforward(Constants.Flywheel.ks, Constants.Flywheel.kv);

  private boolean m_isFlywheelEnabled = false;

  private static Logger m_logger = Logger.getLogger(FlywheelSub.class.getName());

  private final ShuffleboardTab m_shuffleboardTab = Shuffleboard.getTab("Flywheel");
  private final GenericEntry m_shooterFlywheelVelocityL, m_shooterFlywheelVelocityR, m_shooterflywheelPowerL,
      m_shooterflywheelPowerR;

  private double m_setFeedForwardPoint = 0.0;

  public FlywheelSub() {
    m_shooterFlywheelVelocityL = m_shuffleboardTab.add("Left Shooter Flywheel Velocity", 0).getEntry();
    m_shooterFlywheelVelocityR = m_shuffleboardTab.add("Right Shooter Flywheel Velocity", 0).getEntry();
    m_shooterflywheelPowerL = m_shuffleboardTab.add("Left ShooterFlywheel power", 0).getEntry();
    m_shooterflywheelPowerR = m_shuffleboardTab.add("Right ShooterFlywheel power", 0).getEntry();
    init();
  }


  public void init() {
    m_logger.info("Initializing FlywheelSub");
    m_flywheelR.setInverted(false);
    m_flywheelL.setInverted(true);
    m_flywheelR.setIdleMode(IdleMode.kCoast);
    m_flywheelL.setIdleMode(IdleMode.kCoast);
    m_flywheelR.setSmartCurrentLimit(40);
    m_flywheelL.setSmartCurrentLimit(40);
    //_flywheelR.getEncoder().setVelocityConversionFactor(1.0);
    m_flywheelL.getEncoder().setVelocityConversionFactor(1.0);
    //m_flywheelR.follow(m_flywheelL);
    m_flywheelR.set(0);
    m_flywheelL.set(0);


    disableFlywheel();
  }

  @Override
  public void periodic() {
    updateShuffleBoard();
    //left is the master right is the follower
    // This method will be called once per scheduler run

    // Flywheel needs to spin at full power prior to m_shooterSub.spinBothFeeders being executed. 
    // only using left Flywheel velocity. Todo add a right velocity
    if(m_isFlywheelEnabled) {
      double driveOutput =
          m_FlyWheelPID.calculate(getFlywheelVelocityL(),
              Constants.Flywheel.kFlywheelShootVelocity); //4200 is a target velocity we don't know what it is 
      m_setFeedForwardPoint = m_FlyWheelFeedforward.calculate(driveOutput / 2, 0.1);
      //m_flywheelR.set(setPoint + driveOutput);      // m_flywheelL.set(m_setFeedForwardPoint);
      // m_flywheelR.set(m_setFeedForwardPoint);

      // double driveOutput =
      //     m_FlyWheelPID.calculate(getFlywheelVelocityL() - m_setFeedForwardPoint,
      //         Constants.Flywheel.kFlywheelShootVelocity); //4200 is a target velocity we don't know what it is 

      // //m_flywheelR.set(setPoint + driveOutput);
      // m_flywheelL.set(m_setFeedForwardPoint);
      // m_flywheelR.set(m_setFeedForwardPoint);
      m_flywheelL.set(m_setFeedForwardPoint);
      m_flywheelR.set(m_setFeedForwardPoint);
      //m_flywheelL.set(setPoint + driveOutput);
    } else {
      m_flywheelR.set(0);
      m_flywheelL.set(0);
    }
  }

  public void enableFlywheel() {
    m_isFlywheelEnabled = true;
  }

  public void disableFlywheel() {
    m_isFlywheelEnabled = false;
  }


  public boolean isAtTargetVelocity() {

    if(Math.abs(
        getFlywheelVelocityL() - Constants.Flywheel.kFlywheelShootVelocity) < Constants.Flywheel.kFlywheelTolerance) {
      return true;
    }
    return false;
  }


  public double getFlywheelVelocityR() {
    return m_flywheelR.getEncoder().getVelocity();
  }

  public double getFlywheelVelocityL() {
    return m_flywheelL.getEncoder().getVelocity();
  }

  private void updateShuffleBoard() {
    m_shooterFlywheelVelocityL.setDouble(getFlywheelVelocityL());
    m_shooterFlywheelVelocityR.setDouble(getFlywheelVelocityR());
    m_shooterflywheelPowerL.setDouble(m_flywheelL.get());
    m_shooterflywheelPowerR.setDouble(m_flywheelR.get());
  }
}

