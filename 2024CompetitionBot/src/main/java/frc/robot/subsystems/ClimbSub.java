// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ClimbSub extends SubsystemBase {
  public static SerialPort.Port kOnboard;

  private final ClimbSub m_climbSub = new ClimbSub();
  private final DrivetrainSub m_drivetrainSub = new DrivetrainSub();
  private final static CANSparkMax m_climbMotorLeft =
      new CANSparkMax(Constants.CanIds.kClimbMotorL, CANSparkLowLevel.MotorType.kBrushless);
  private final static CANSparkMax m_climbMotorRight =
      new CANSparkMax(Constants.CanIds.kClimbMotorR, CANSparkLowLevel.MotorType.kBrushless);
  private final SparkAbsoluteEncoder m_pivotEncoder = m_climbMotorRight.getAbsoluteEncoder(Type.kDutyCycle);
  // private final byte[] bufferByte = new byte[] {0x00};
  private static int loopNumber = '0';
  private static int dataSetLangth = '0';
  private static int loopThroughBufferByte = '0';
  private static int arrayNumberWanted = '1';


  //creating an instances of RS_232 port
  private final SerialPort m_SerialPort =
      new SerialPort(Constants.ClimbConstants.kBaudRate, kOnboard);


  /** Creates a new ClimbSub. */
  public ClimbSub() {
    m_climbMotorLeft.setInverted(true);
    m_climbMotorRight.setInverted(false);
    m_climbMotorLeft.setIdleMode(IdleMode.kBrake);
    m_climbMotorRight.setIdleMode(IdleMode.kBrake);
    m_climbMotorLeft.getEncoder().setPositionConversionFactor(Constants.ClimbConstants.kTickCofficient);
    m_climbMotorRight.getEncoder().setPositionConversionFactor(Constants.ClimbConstants.kTickCofficient);
    setClimbPowerLeft(0.0);
    setClimbPowerRight(0.0);
    SmartDashboard.putData("ClimbReset", new InstantCommand(() -> resetEncoders()));

  }

  @Override
  public void periodic() {
    updateSmartDashboard();
  }

  private void updateSmartDashboard() {
    SmartDashboard.putNumber("Climb Left Power", m_climbMotorLeft.get());
    SmartDashboard.putNumber("Climb Right Power", m_climbMotorRight.get());
    SmartDashboard.putNumber("Climb Left Height", getLeftHeight());
    SmartDashboard.putNumber("Climb Right Height", getRightHeight());
    SmartDashboard.putNumber("Pivot Velocity", getPivotVelocity());
    SmartDashboard.putNumber("Pivot Position", getPivotPosition());
  }

  public void setClimbPowerLeft(double leftPower) {
    m_climbMotorLeft.set(leftPower);

  }

  public void setClimbPowerRight(double rightPower) {
    m_climbMotorRight.set(rightPower);
  }

  public double getLeftHeight() {
    return m_climbMotorLeft.getEncoder().getPosition();
  }

  public double getRightHeight() {
    return m_climbMotorRight.getEncoder().getPosition();
  }

  public double getLeftVelocity() {
    return m_climbMotorLeft.getEncoder().getVelocity();
  }

  public double getRightVelocity() {
    return m_climbMotorRight.getEncoder().getVelocity();
  }

  public void resetEncoders() {
    System.out.println("Reseting Encoders");
    m_climbMotorLeft.getEncoder().setPosition(0.0);
    m_climbMotorRight.getEncoder().setPosition(0.0);
  }

  public double getPivotVelocity() {
    return m_pivotEncoder.getVelocity();
  }

  public double getPivotPosition() {
    return m_pivotEncoder.getPosition();
  }

  public boolean isLeftAtLimit() {
    return m_climbMotorLeft.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed();
  }

  public boolean isRightAtLimit() {
    return m_climbMotorRight.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed();
  }

  public void init() {
    resetEncoders();
  }

  public void RS232Listen() {
    //byte[] m_buffer = m_SerialPort.read(10);
    m_SerialPort.setReadBufferSize(Constants.ClimbConstants.kBufferSize);
    m_SerialPort.setTimeout(Constants.ClimbConstants.kTimeOutLangth);
    //getBytesReceived

    byte byteArray[];
    byteArray = new byte[Constants.ClimbConstants.kBufferSize];

    byte bufferByte[];
    bufferByte = new byte[Constants.ClimbConstants.kBufferSize];

    bufferByte = m_SerialPort.read(Constants.ClimbConstants.kReadByteLength);
    while(loopThroughBufferByte <= Constants.ClimbConstants.kBufferSize) {
      if(bufferByte[loopThroughBufferByte] == 10100101) { //0xA5
        dataSetLangth = bufferByte[loopThroughBufferByte + 1];
      }
      while(loopNumber <= dataSetLangth) {
        byteArray[loopNumber] = bufferByte[loopThroughBufferByte + 1 + arrayNumberWanted];
        arrayNumberWanted++;
        loopNumber++;
      }
      loopThroughBufferByte++;
    }
    System.out.println(byteArray);
  }
}
