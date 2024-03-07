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
  private static byte[] m_LEDBuffer = new byte[8];
  private static boolean m_LEDBufferCheckSumCalculated = false;

  private static int loopNumber = 0;
  private static int dataSetLength = 0;
  private static int loopThroughBufferByte = 0;
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

    m_SerialPort.setReadBufferSize(m_LEDBuffer.length);
    m_LEDBuffer[0] = (byte) 0xA5;
    for(int i = 0; i < 2; i++) {
      updateLED(i, 0, 255, 0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // RS232Listen();
    // writeToSerial();
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
    if(!m_LEDBufferCheckSumCalculated) {
      m_LEDBuffer[8] = 0;
      for(int i = 0; i < 8; i++) {
        m_LEDBuffer[8] += m_LEDBuffer[i];
      }
      m_LEDBufferCheckSumCalculated = true;
    }
    m_SerialPort.write(m_LEDBuffer, m_LEDBuffer.length);
    m_SerialPort.flush();

  }

  public void updateLED(int LEDIndex, int r, int g, int b) {
    m_LEDBuffer[LEDIndex * 3 + 1] = (byte) r;
    m_LEDBuffer[LEDIndex * 3 + 2] = (byte) g;
    m_LEDBuffer[LEDIndex * 3 + 3] = (byte) b;
    m_LEDBufferCheckSumCalculated = false;
  }

  public boolean isSensorTripped(int sensorIndex) {
    if(m_intakeSensors[sensorIndex] < 0) { //need to measure distance to note to replace 0
      return true;
    }
    return false;
  }

  public int[] RS232Listen() {
    //byte[] m_buffer = m_SerialPort.read(10);
    m_SerialPort.setReadBufferSize(Constants.Arduino.kBufferSize);
    m_SerialPort.setTimeout(Constants.Arduino.kTimeOutLength);
    // m_SerialPort.setFlowControl(SerialPort.FlowControl.kXonXoff);
    //getBytesReceived

    byte byteArray[] = new byte[Constants.Arduino.kByteArrayLength];

    byte bufferByte[] = new byte[Constants.Arduino.kBufferSize];

    checkSumWatchDog = 0;

    do {
      Arrays.fill(byteArray, (byte) 0);
      Arrays.fill(bufferByte, (byte) 0);
      if(checkSumWatchDog > 5) {
        System.out.println("========checkSum corrupt 5 times=========");
        // reset the arduino
        break;
      }
      checkSumWatchDog++;
      m_SerialPort.reset();

      if(m_SerialPort.getBytesReceived() != 0) { //shouldn't read if there is no new data, this method doesn't work
        bufferByte = m_SerialPort.read(Constants.Arduino.kReadByteLength);
      }
      bufferByte = m_SerialPort.read(Constants.Arduino.kReadByteLength);
      // System.out.println("===============================");
      // System.out.println(bufferByte.length);

      byteArrayCount = 0;
      loopThroughBufferByte = 0;

      while(loopThroughBufferByte < bufferByte.length) {
        if((bufferByte[loopThroughBufferByte] & 0xFF) == 0xA5) { //finds 0xA5, the start of the data sent

          dataSetLength = bufferByte[loopThroughBufferByte + 1];
          if(dataSetLength > Constants.Arduino.kByteArrayLength) {
            break;
          }

          loopNumber = 0;
          arrayNumberWanted = 1;
          while(loopNumber < dataSetLength) {
            byteArray[byteArrayCount] = bufferByte[loopThroughBufferByte + 1 + arrayNumberWanted];
            byteArrayCount++;
            arrayNumberWanted++;
            loopNumber++;
          }
          break;
        }
        loopThroughBufferByte++;
      }
      for(int i = 0; i < Constants.Arduino.kByteArrayLength - 1; i++) {
        checkSum += byteArray[i];
      }
    } while(checkSum != byteArray[4]);
    checkSum = 0;

    for(int i = 0; i < m_intakeSensors.length; i++) {
      m_intakeSensors[i] = (((byteArray[i + 1] & 0xFF) << 8) | (byteArray[i] & 0xFF));
    }

    // StringBuilder sb = new StringBuilder(byteArray.length * 2);
    // for(byte b : byteArray) {
    //   sb.append(String.format("%02x", b));
    // }
    // System.out.println("===========================================================");
    // System.out.println(m_intakeSensors[0]);
    // System.out.println(sb);
    // System.out.println(byteArray);
    // System.out.println(bufferByte);
    // System.out.println("===========================================================");

    return m_intakeSensors;
  }
}
