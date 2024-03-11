// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.time.Duration;
import java.time.Instant;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Parity;
import edu.wpi.first.wpilibj.SerialPort.StopBits;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ArduinoSub extends SubsystemBase {

  private final ShuffleboardTab m_shuffleboardTab = Shuffleboard.getTab("Arduino");
  private final GenericEntry m_sensorFlywheelNear, m_sensorFlywheelMid, m_sensorFlywheelFar, m_sensorCentreFlywheelSide,
      m_sensorCentreIntakeSide, m_sensorIntakeFar,
      m_sensorIntakeMid, m_sensorIntakeNear;

  /** Creates a new ArduinoSub. */
  private static byte[] m_LEDUpdateMessage = new byte[75];
  private static byte[] m_getMessage = {(byte) Constants.Arduino.kMessageHeader, (byte) 0x01, (byte) 0xA6}; // header, command 1 to get sensor info, checksum
  private static boolean m_LEDHasChanged = true;
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

    //Set top and bottom led sections to green
    m_SerialPort.setReadBufferSize(m_LEDUpdateMessage.length);
    m_LEDUpdateMessage[0] = Constants.Arduino.kMessageHeader;
    m_LEDUpdateMessage[1] = (byte) 0x02; //command byte for updating LEDs

    // Set top and bottom LEDs
    updateLED(0, 0, 255, 0);
    updateLED(1, 0, 255, 0);

    m_SerialPort.setReadBufferSize(Constants.Arduino.kBufferSize); // John cthis should only be called once during initialization
    m_SerialPort.setTimeout(Constants.Arduino.kTimeOutLength); // John c: consider setting this to zero to only read the available bytes

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    RS232Listen();
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
    // Get intake sensor data
    if(!m_LEDHasChanged) {
      m_SerialPort.write(m_getMessage, m_getMessage.length);
    }
    // Update LED panel colors when the update request is true
    else {
      m_LEDUpdateMessage[8] = 0;
      for(int i = 0; i < 8; i++) {
        m_LEDUpdateMessage[8] += m_LEDUpdateMessage[i];
      }
      m_SerialPort.write(m_LEDUpdateMessage, 9);
      m_LEDHasChanged = false;
    }
    //m_SerialPort.flush();

  }

  public void updateLED(int LEDIndex, int r, int g, int b) {
    m_LEDUpdateMessage[LEDIndex * 3 + 2] = (byte) r;
    m_LEDUpdateMessage[LEDIndex * 3 + 3] = (byte) g;
    m_LEDUpdateMessage[LEDIndex * 3 + 4] = (byte) b;
    m_LEDHasChanged = false;
  }

  public boolean isSensorTripped(int sensorIndex) {
    if(m_intakeSensors[sensorIndex] < 900) {
      return true;
    }
    return false;
  }

  public void RS232Listen() {

    byte receiveBuffer[] = new byte[Constants.Arduino.kBufferSize];

    int bytesInBuffer = m_SerialPort.getBytesReceived();
    SmartDashboard.putBoolean("Got Bytes", false);
    if(bytesInBuffer < Constants.Arduino.kReadMessageLength) {
      //wait for complete message
      return;
    }
    SmartDashboard.putBoolean("Got Bytes", true);
    receiveBuffer = m_SerialPort.read(bytesInBuffer);

    int bufferIndex = 0;
    byte checksum = 0;
    byte version = 0;

    // Checksum check
    SmartDashboard.putBoolean("Valid Arduino Data", false);
    SmartDashboard.putBoolean("Not enough bytes", false);
    SmartDashboard.putBoolean("Found header", false);
    while(bufferIndex < receiveBuffer.length) {

      // Loop until header found
      if((receiveBuffer[bufferIndex++]) != Constants.Arduino.kMessageHeader) {
        // Haven't found the header yet
        continue;
      }
      // Start at the header
      SmartDashboard.putBoolean("Found header", true);
      // Don't go over max amount of bytes recieved
      if((bytesInBuffer - (bufferIndex - 1)) < Constants.Arduino.kReadMessageLength) {
        SmartDashboard.putBoolean("Not enough bytes", true);
        break;
      }

      // Get checksum of data after header
      for(int i = bufferIndex - 1; i < (bufferIndex - 1) + Constants.Arduino.kReadMessageLength - 1; i++) {
        checksum += receiveBuffer[i];
      }
      // Is the checksum wrong?
      if(checksum != receiveBuffer[(bufferIndex - 1) + Constants.Arduino.kReadMessageLength - 1]) {
        // Keep going in case that it was just a random 0xA5 (misinput)
        continue;
      }
      // Checksum was valid 
      // John C - increment the buffer index to point to the version
      version = receiveBuffer[bufferIndex];
      if(version != 0x03) {
        // We should check that this is a known version but we'll implement that only if we need to
        // For now, assume we're getting the right message
      }
      bufferIndex++;
      // Parse sensor data
      // John C - bufferIndex should be pointing at the data
      SmartDashboard.putBoolean("Valid Arduino Data", true);
      for(int s = 0; s < 8; s++) {
        // John C : need to convert signed byte to unsigned values before doing the math!!!

        int sensorValLow = (int) receiveBuffer[bufferIndex + s * 2];
        int sensorValHigh = (int) receiveBuffer[bufferIndex + s * 2 + 1];
        // Force value to be under 1023
        m_intakeSensors[s] = (sensorValLow | (sensorValHigh << 8)) & 0x3ff;
      }
      // Done
    }
  }

}
