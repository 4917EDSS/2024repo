// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class PowerSub extends SubsystemBase {

  PowerDistribution powerDistributionModule = new PowerDistribution(1, ModuleType.kRev);

  private final ShuffleboardTab m_shuffleboardTab = Shuffleboard.getTab("Power");
  private final GenericEntry m_sbVoltage, m_sbTemperature, m_sbTotalCurrent, m_sbTotalPower, m_sbTotalEnergy,
      m_sbDriveMotorFrontRight, m_sbDriveMotorFrontLeft, m_sbDriveMotorBackRight, m_sbDriveMotorBackLeft,
      m_sbSteeringMotorFR, m_sbSteeringMotorFL, m_sbSteeringMotorBR, m_sbSteeringMotorBL,
      m_sbClimbMotorL, m_sbClimbMotorR, m_sbLowerFeeder, m_sbUpperFeeder, m_sbPivot, m_sbFlywheelL, m_sbFlywheelR,
      m_sbIntakeRollers,
      m_sbLimelightUp, m_sbLimelightDown, m_sbCancoders, m_sbVRM, m_sbRoboRio, m_sbArduino, m_sbRadio;

  /** Creates a new PowerSub. */
  public PowerSub() {
    // Get the voltage going into the PDP, in Volts.
    // The PDP returns the voltage in increments of 0.05 Volts.
    m_sbVoltage = m_shuffleboardTab.add("Voltage", 0).withPosition(6, 0).getEntry();
    // Retrieves the temperature of the PDP, in degrees Celsius.
    m_sbTemperature = m_shuffleboardTab.add("Temperature", 0).withPosition(6, 1).getEntry();
    // Get the total current of all channels.
    m_sbTotalCurrent = m_shuffleboardTab.add("Current", 0).withPosition(6, 2).getEntry();
    // Get the total power of all channels.
    // Power is the bus voltage multiplied by the current with the units Watts.
    m_sbTotalPower = m_shuffleboardTab.add("Power", 0).withPosition(6, 3).getEntry();
    // Get the total energy of all channels.
    // Energy is the power summed over time with units Joules.
    m_sbTotalEnergy = m_shuffleboardTab.add("Energy", 0).withPosition(6, 4).getEntry();

    //BREAKERS START HERE
    // Get the current in Amperes.
    // The PDP returns the current in increments of 0.125A.
    // At low currents the current readings tend to be less accurate.
    m_sbDriveMotorFrontRight =
        m_shuffleboardTab.add("DFR" + Constants.Breakers.kDriveMotorFrontRight, 0).withPosition(0, 0)
            .getEntry();
    m_sbDriveMotorFrontLeft =
        m_shuffleboardTab.add("DFL" + Constants.Breakers.kDriveMotorFrontLeft, 0).withPosition(0, 1).getEntry();
    m_sbDriveMotorBackRight =
        m_shuffleboardTab.add("DBR" + Constants.Breakers.kDriveMotorBackRight, 0).withPosition(0, 2).getEntry();
    m_sbDriveMotorBackLeft =
        m_shuffleboardTab.add("DBL" + Constants.Breakers.kDriveMotorBackLeft, 0).withPosition(0, 3).getEntry();

    m_sbSteeringMotorFR =
        m_shuffleboardTab.add("SFR" + Constants.Breakers.kSteeringMotorFR, 0).withPosition(1, 0).getEntry();
    m_sbSteeringMotorFL =
        m_shuffleboardTab.add("SFL" + Constants.Breakers.kSteeringMotorFL, 0).withPosition(1, 1).getEntry();
    m_sbSteeringMotorBR =
        m_shuffleboardTab.add("SBR" + Constants.Breakers.kSteeringMotorBR, 0).withPosition(1, 2).getEntry();
    m_sbSteeringMotorBL =
        m_shuffleboardTab.add("SBL" + Constants.Breakers.kSteeringMotorBL, 0).withPosition(1, 3).getEntry();

    m_sbClimbMotorL =
        m_shuffleboardTab.add("CML" + Constants.Breakers.kClimbMotorL, 0).withPosition(2, 0).getEntry();
    m_sbClimbMotorR =
        m_shuffleboardTab.add("CMR" + Constants.Breakers.kClimbMotorR, 0).withPosition(2, 1).getEntry();

    m_sbLowerFeeder =
        m_shuffleboardTab.add("LF" + Constants.Breakers.kLowerFeeder, 0).withPosition(2, 2).getEntry();
    m_sbUpperFeeder =
        m_shuffleboardTab.add("LF" + Constants.Breakers.kUpperFeeder, 0).withPosition(2, 3).getEntry();

    m_sbPivot =
        m_shuffleboardTab.add("Pivot" + Constants.Breakers.kPivot, 0).withPosition(3, 2).getEntry();
    m_sbFlywheelL =
        m_shuffleboardTab.add("FlywheelL" + Constants.Breakers.kFlywheelL, 0).withPosition(3, 0).getEntry();
    m_sbFlywheelR =
        m_shuffleboardTab.add("FlywheelR" + Constants.Breakers.kFlywheelR, 0).withPosition(3, 1).getEntry();
    m_sbIntakeRollers =
        m_shuffleboardTab.add("Intake Rollers" + Constants.Breakers.kIntakeRollers, 0).withPosition(3, 3)
            .getEntry();

    m_sbLimelightUp =
        m_shuffleboardTab.add("Lime Up" + Constants.Breakers.kLimelightUp, 0).withPosition(4, 0).getEntry();
    m_sbLimelightDown =
        m_shuffleboardTab.add("Lime Down" + Constants.Breakers.kLimelightDown, 0).withPosition(4, 1).getEntry();
    m_sbCancoders =
        m_shuffleboardTab.add("Cancoders" + Constants.Breakers.kCancoders, 0).withPosition(4, 2).getEntry();
    m_sbVRM =
        m_shuffleboardTab.add("VRM" + Constants.Breakers.kVRM, 0).withPosition(4, 3).getEntry();
    m_sbRoboRio =
        m_shuffleboardTab.add("RoboRio" + Constants.Breakers.kRoboRio, 0).withPosition(5, 0).getEntry();
    m_sbArduino =
        m_shuffleboardTab.add("Arduino" + Constants.Breakers.kArduino, 0).withPosition(5, 1).getEntry();
    m_sbRadio =
        m_shuffleboardTab.add("Radio" + Constants.Breakers.kRadio, 0).withPosition(5, 2).getEntry();
  }

  public void init() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(!RobotContainer.disableShuffleboardPrint) {
      updateShuffleBoard();
    }
  }


  private void updateShuffleBoard() {

    m_sbVoltage.setDouble(powerDistributionModule.getVoltage());
    m_sbTemperature.setDouble(powerDistributionModule.getTemperature());
    m_sbTotalCurrent.setDouble(powerDistributionModule.getTotalCurrent());
    m_sbTotalPower.setDouble(powerDistributionModule.getTotalPower());
    m_sbTotalEnergy.setDouble(powerDistributionModule.getTotalEnergy());
    m_sbDriveMotorFrontRight
        .setDouble(powerDistributionModule.getCurrent(Constants.Breakers.kDriveMotorFrontRight));
    m_sbDriveMotorFrontLeft.setDouble(powerDistributionModule.getCurrent(Constants.Breakers.kDriveMotorFrontLeft));
    m_sbDriveMotorBackRight.setDouble(powerDistributionModule.getCurrent(Constants.Breakers.kDriveMotorBackRight));
    m_sbDriveMotorBackLeft.setDouble(powerDistributionModule.getCurrent(Constants.Breakers.kDriveMotorBackLeft));
    m_sbSteeringMotorFR.setDouble(powerDistributionModule.getCurrent(Constants.Breakers.kSteeringMotorFR));
    m_sbSteeringMotorFL.setDouble(powerDistributionModule.getCurrent(Constants.Breakers.kSteeringMotorFL));
    m_sbSteeringMotorBR.setDouble(powerDistributionModule.getCurrent(Constants.Breakers.kSteeringMotorBR));
    m_sbSteeringMotorBL.setDouble(powerDistributionModule.getCurrent(Constants.Breakers.kSteeringMotorBL));
    m_sbClimbMotorL.setDouble(powerDistributionModule.getCurrent(Constants.Breakers.kClimbMotorL));
    m_sbClimbMotorR.setDouble(powerDistributionModule.getCurrent(Constants.Breakers.kClimbMotorR));
    m_sbLowerFeeder.setDouble(powerDistributionModule.getCurrent(Constants.Breakers.kLowerFeeder));
    m_sbUpperFeeder.setDouble(powerDistributionModule.getCurrent(Constants.Breakers.kUpperFeeder));

    m_sbPivot.setDouble(powerDistributionModule.getCurrent(Constants.Breakers.kPivot));
    m_sbFlywheelL.setDouble(powerDistributionModule.getCurrent(Constants.Breakers.kFlywheelL));
    m_sbFlywheelR.setDouble(powerDistributionModule.getCurrent(Constants.Breakers.kFlywheelR));
    m_sbIntakeRollers.setDouble(powerDistributionModule.getCurrent(Constants.Breakers.kIntakeRollers));
    m_sbLimelightUp.setDouble(powerDistributionModule.getCurrent(Constants.Breakers.kLimelightUp));
    m_sbLimelightDown.setDouble(powerDistributionModule.getCurrent(Constants.Breakers.kLimelightDown));
    m_sbCancoders.setDouble(powerDistributionModule.getCurrent(Constants.Breakers.kCancoders));
    m_sbVRM.setDouble(powerDistributionModule.getCurrent(Constants.Breakers.kVRM));
    m_sbRoboRio.setDouble(powerDistributionModule.getCurrent(Constants.Breakers.kRoboRio));
    m_sbArduino.setDouble(powerDistributionModule.getCurrent(Constants.Breakers.kArduino));
    m_sbRadio.setDouble(powerDistributionModule.getCurrent(Constants.Breakers.kRadio));


  }


}
