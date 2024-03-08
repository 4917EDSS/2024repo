// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Parity;
import edu.wpi.first.wpilibj.SerialPort.StopBits;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArduinoSub extends SubsystemBase {

  private final ShuffleboardTab m_shuffleboardTab = Shuffleboard.getTab("Arduino");
  private final GenericEntry m_sensorFlywheelNear, m_sensorFlywheelMid, m_sensorFlywheelFar, m_sensorCentreFlywheelSide,
      m_sensorCentreIntakeSide, m_sensorIntakeFar,
      m_sensorIntakeMid, m_sensorIntakeNear;

  /** Creates a new ArduinoSub. */
  private static byte[] m_LEDUpdateMessage = new byte[75];
  private static byte[] m_getMessage = {(byte) Constants.Arduino.kMessageHeader, (byte) 0x01, (byte) 0xA6}; // header, command 1 to get sensor info, checksum
  private static boolean m_LEDHasChanged = true;

  private static int loopNumber = 0;
  private static int dataSetLength = 0;

  private static int arrayNumberWanted = 1;
  private static int byteArrayCount = 0;
  private static int checkSum = 0;
  private static int checkSumWatchDog = 0;

  private static int m_intakeSensors[] = new int[8];

  // Creating an instances of RS_232 port to communicate with Arduino (sensors)
  private final SerialPort m_SerialPort =
      new SerialPort(Constants.Arduino.kBaudRate, SerialPort.Port.kMXP, 8, Parity.kNone, StopBits.kOne);

  public ArduinoSub() {
    m_sensorFlywheelNear = m_shuffleboardTab.add("S FW Near", 0).getEntry();
    m_sensorFlywheelMid = m_shuffleboardTab.add("S FW Mid", 0).getEntry();
    m_sensorFlywheelFar = m_shuffleboardTab.add("S FW Far", 0).getEntry();
    m_sensorCentreFlywheelSide = m_shuffleboardTab.add("S Centre FW", 0).getEntry();
    m_sensorCentreIntakeSide = m_shuffleboardTab.add("S Centre Intake", 0).getEntry();
    m_sensorIntakeFar = m_shuffleboardTab.add("S Intk Far", 0).getEntry();
    m_sensorIntakeMid = m_shuffleboardTab.add("Se Intk Mid", 0).getEntry();
    m_sensorIntakeNear = m_shuffleboardTab.add("S Intk Near", 0).getEntry();

    init();
  }

  public void init() {
    // TODO:  Power cycle the arduino?

    m_SerialPort.setReadBufferSize(m_LEDUpdateMessage.length);
    m_LEDUpdateMessage[0] = Constants.Arduino.kMessageHeader;
    m_LEDUpdateMessage[0] = (byte) 0x02; //command byte for updating LEDs
    for(int i = 0; i < 2; i++) {
      updateLED(i, 0, 255, 0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // RS232Listen();
    writeToSerial();
    updateShuffleBoard();
  }

  private void updateShuffleBoard() {
    m_sensorFlywheelNear.setDouble(m_intakeSensors[0]);
    m_sensorFlywheelMid.setDouble(m_intakeSensors[1]);
    m_sensorFlywheelFar.setDouble(m_intakeSensors[2]);
    m_sensorCentreFlywheelSide.setDouble(m_intakeSensors[3]);
    m_sensorCentreIntakeSide.setDouble(m_intakeSensors[4]);
    m_sensorIntakeFar.setDouble(m_intakeSensors[5]);
    m_sensorIntakeMid.setDouble(m_intakeSensors[6]);
    m_sensorIntakeNear.setDouble(m_intakeSensors[7]);
  }

  private void writeToSerial() {
    if(!m_LEDHasChanged) {
      m_SerialPort.write(m_getMessage, m_getMessage.length);
    } else {
      m_LEDUpdateMessage[8] = 0;
      for(int i = 0; i < 8; i++) {
        m_LEDUpdateMessage[8] += m_LEDUpdateMessage[i];
      }
      m_SerialPort.write(m_LEDUpdateMessage, m_LEDUpdateMessage.length);
      m_LEDHasChanged = false;
    }
    m_SerialPort.flush();

  }

  public void updateLED(int LEDIndex, int r, int g, int b) {
    m_LEDUpdateMessage[LEDIndex * 3 + 2] = (byte) r;
    m_LEDUpdateMessage[LEDIndex * 3 + 3] = (byte) g;
    m_LEDUpdateMessage[LEDIndex * 3 + 4] = (byte) b;
    m_LEDHasChanged = true;
  }

  public boolean isSensorTripped(int sensorIndex) {
    if(m_intakeSensors[sensorIndex] < 0) { //need to measure distance to note to replace 0
      return true;
    }
    return false;
  }

  public void RS232Listen() {
    //byte[] m_buffer = m_SerialPort.read(10);
    m_SerialPort.setReadBufferSize(Constants.Arduino.kBufferSize);
    m_SerialPort.setTimeout(Constants.Arduino.kTimeOutLength);

    byte receiveBuffer[] = new byte[Constants.Arduino.kBufferSize];
    byte byteArray[] = new byte[Constants.Arduino.kSensorDataLength];

    int bytesInBuffer = m_SerialPort.getBytesReceived();
    if(bytesInBuffer < Constants.Arduino.kReadMessageLength) {
      //wait for complete message
      return;
    }

    receiveBuffer = m_SerialPort.read(bytesInBuffer);

    int bufferIndex = 0;
    byte checksum = 0;
    byte version = 0;
    while(bufferIndex < receiveBuffer.length) {
      if((receiveBuffer[bufferIndex++] & 0xFF) != Constants.Arduino.kMessageHeader) {
        // Haven't found the header yet
        continue;
      }

      if(bytesInBuffer - bufferIndex < Constants.Arduino.kReadMessageLength) {
        break;
      }

      for(int i = bufferIndex; i < bufferIndex + Constants.Arduino.kReadMessageLength - 1; i++) {
        checksum += receiveBuffer[i];
      }
      if(checksum != receiveBuffer[bufferIndex + Constants.Arduino.kReadMessageLength]) {
        continue;
      }

      // We should check that this is a known version but we'll implement that only if we need to
      version = receiveBuffer[bufferIndex++];
    }
  }
}
