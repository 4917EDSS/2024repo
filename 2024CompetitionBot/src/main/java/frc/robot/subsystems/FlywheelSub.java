// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.logging.Logger;
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
import frc.robot.RobotContainer;

public class FlywheelSub extends SubsystemBase {
  private static Logger m_logger = Logger.getLogger(FlywheelSub.class.getName());

  private ArrayList<String> loggingStrings = new ArrayList<String>();

  private final CANSparkMax m_flywheelL =
      new CANSparkMax(Constants.CanIds.kFlywheelL, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_flywheelR =
      new CANSparkMax(Constants.CanIds.kFlywheelR, CANSparkLowLevel.MotorType.kBrushless);

  //private final PIDController m_flyWheelPID = new PIDController(0, 0, 0); // We've found that we don't need this
  private final SimpleMotorFeedforward m_flyWheelFeedforward =
      new SimpleMotorFeedforward(Constants.Flywheel.ks, Constants.Flywheel.kv);

  private boolean m_isFlywheelEnabled = false;
  private double m_flywheelShootVelocity = Constants.Flywheel.kFlywheelShootVelocity;
  //private double m_currentVoltage = 0.0;  // For Feedforward testing

  private final ShuffleboardTab m_shuffleboardTab = Shuffleboard.getTab("Flywheel");
  private final GenericEntry m_shooterFlywheelVelocityL, m_shooterFlywheelVelocityR, m_shooterflywheelPowerL,
      m_shooterflywheelPowerR;

  // Only needed if we decide we need a PID controller
  private final PIDController m_flyWheelPID = new PIDController(0.012, 0.0, 0.0);
  double FlywheelShootVelocity;

  public FlywheelSub() {
    m_shooterFlywheelVelocityL = m_shuffleboardTab.add("Left Velocity", 0).getEntry();
    m_shooterFlywheelVelocityR = m_shuffleboardTab.add("Right Velocity", 0).getEntry();
    m_shooterflywheelPowerL = m_shuffleboardTab.add("Left Power", 0).getEntry();
    m_shooterflywheelPowerR = m_shuffleboardTab.add("Right Power", 0).getEntry();

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

    m_flywheelL.getEncoder().setPositionConversionFactor(Constants.Flywheel.kPositionConversionFactor);
    m_flywheelR.getEncoder().setPositionConversionFactor(Constants.Flywheel.kPositionConversionFactor);
    m_flywheelL.getEncoder().setVelocityConversionFactor(Constants.Flywheel.kVelocityConversionFactor);
    m_flywheelR.getEncoder().setVelocityConversionFactor(Constants.Flywheel.kVelocityConversionFactor);

    m_flywheelR.set(0);
    m_flywheelL.set(0);

    disableFlywheel();
  }

  @Override
  public void periodic() {
    updateShuffleBoard();

    // Flywheel needs to spin at set velocity prior to m_pivotSub.spinBothFeeders being executed. 
    if(m_isFlywheelEnabled) {
      double feedForwardVoltage = m_flyWheelFeedforward.calculate(m_flywheelShootVelocity, 0.0);
      // So far, we don't need the PID control.  Feedforward is doing well on its own
      double pidVoltage = m_flyWheelPID.calculate(getFlywheelVelocityL(), m_flywheelShootVelocity);

      setFlywheelVoltage(feedForwardVoltage + pidVoltage);
    } else {
      setFlywheelVoltage(0.0);
    }
  }

  public void addLog(String data) {
    loggingStrings.add(data);
  }

  public void printLogs() {
    for(int logNum = 0; logNum < loggingStrings.size(); logNum++) {
      m_logger.fine("[" + logNum + "] " + loggingStrings.get(logNum));
    }
  }

  public void brakeFlywheels() {
    m_flywheelR.setIdleMode(IdleMode.kBrake);
    m_flywheelL.setIdleMode(IdleMode.kBrake);
  }

  public void unbrakeFlywheels() {
    m_flywheelR.setIdleMode(IdleMode.kCoast);
    m_flywheelL.setIdleMode(IdleMode.kCoast);
  }

  public void setFlywheelVoltage(double voltage) {
    m_flywheelL.setVoltage(voltage);
    m_flywheelR.setVoltage(voltage);

    // m_currentVoltage = voltage; // For Feedforward testing
  }

  public double getFlywheelVelocityR() {
    return m_flywheelR.getEncoder().getVelocity();
  }

  public double getFlywheelVelocityL() {
    return m_flywheelL.getEncoder().getVelocity();
  }

  public void enableFlywheel(double flywheelVelocity) {
    m_isFlywheelEnabled = true;

    // Slow down the shot if in demo mode (will break vision shots)
    if(RobotContainer.m_demoMode) {
      m_flywheelShootVelocity = flywheelVelocity * 0.75;
    } else {
      m_flywheelShootVelocity = flywheelVelocity;
    }
    //System.out.println("En Flywheel");
  }

  public void disableFlywheel() {
    m_isFlywheelEnabled = false;
    //System.out.println("Dis Flywheel");
  }

  public boolean isAtTargetVelocity() {

    if(Math.abs(
        getFlywheelVelocityL() - Constants.Flywheel.kFlywheelShootVelocity) < Constants.Flywheel.kFlywheelTolerance) {
      return true;
    }
    return false;
  }

  public boolean isAtLeastRelativeVelocity(double velocity) {

    if(getFlywheelVelocityL() > velocity - Constants.Flywheel.kFlywheelTolerance) {
      return true;
    }
    return false;
  }

  // For Feedforward testing
  // public void incrementVoltage() {
  //   setFlywheelVoltage(m_currentVoltage + 0.125);
  // }

  private void updateShuffleBoard() {
    m_shooterFlywheelVelocityL.setDouble(getFlywheelVelocityL());
    if(!RobotContainer.disableShuffleboardPrint) {
      m_shooterFlywheelVelocityR.setDouble(getFlywheelVelocityR());
      m_shooterflywheelPowerL.setDouble(m_flywheelL.get());
      m_shooterflywheelPowerR.setDouble(m_flywheelR.get());
    }

    // Uncomment if need to add PID controller
    //m_flyWheelPID.setP(SmartDashboard.getNumber("Flywheel P", m_flywheelP));
    // SmartDashboard.putNumber("FW L Pos", m_flywheelL.getEncoder().getPosition());
    // SmartDashboard.putNumber("FW L Voltage", m_currentVoltage);
  }
}

