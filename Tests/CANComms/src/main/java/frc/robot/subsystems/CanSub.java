// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.IntBuffer;

import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class CanSub extends SubsystemBase {
  byte m_data_buffer[];
  DigitalOutput m_do0;
  DigitalOutput m_do1;
  DigitalOutput m_do2;
  int m_ARBID;

  /** Creates a new CanSub. */
  public CanSub(int CustomSensorID) {
    m_data_buffer = new byte[8];
    m_do0 = new DigitalOutput(0);
    m_do1 = new DigitalOutput(1);
    m_do2 = new DigitalOutput(2);
    m_ARBID = createCANId(0x123, CustomSensorID, 8, 10);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Read sensor(s)
    UpdateCustomSensor();
  }


  public byte getDataBufferByte(int byteIndex) {
    return m_data_buffer[byteIndex];
  }

  // A helper function to assemble a full arbitration ID.
  public int createCANId(int apiId, int deviceId, int manufacturer, int deviceType) {
    return ((int) (deviceType) & 0x1F) << 24 | ((int) (manufacturer) & 0xFF) << 16 | (apiId & 0x3FF) << 6 | (deviceId & 0x3F);
  }

  public void UpdateCustomSensor() {
    ByteBuffer targetedMessageID = ByteBuffer.allocateDirect(4);//Must be direct
    targetedMessageID.order(ByteOrder.LITTLE_ENDIAN); //Set order of bytes
    targetedMessageID.asIntBuffer().put(0, m_ARBID); //Put the arbID into the buffer
    ByteBuffer timeStamp = ByteBuffer.allocateDirect(4); //Allocate memory for time stamp

    byte[] output_data = new byte[8];
    
    try {
      // Read data from the CAN bus.  If this fails, fall back to the error handler.

      // FRCNetCommCANSessionMuxReceiveMessage
      // IntBuffer messageID - The combination of device type, manf, API ID, and device number. This 
      //                       will be replaced with the actual ID received (if the mask didn't require
      //                       an exact match).
      // int messageIDMask - Search for messages matching the messageID. A mask of 0xFFFFFFFF will only
      //                     match exact messages. A mask of 0xFFFFFFFC0 will match everything except
      //                     device ID allowing one class to process all sensors.
      // ByteBuffer timeStamp - The timestamp of when the message was received. Unsure if this is the
      //                        OS clock or a hardware clock linked to the CAN Bus processing.
      
      System.arraycopy(
          CANJNI.FRCNetCommCANSessionMuxReceiveMessage(targetedMessageID.asIntBuffer(), 0xFFFFFFC0, timeStamp), 0,
          m_data_buffer, 0, m_data_buffer.length);

      m_do0.set(!m_do0.get());

      if((m_data_buffer != null) && (timeStamp != null)) {
        if(m_data_buffer.length >= 3) {
          m_do1.set(m_data_buffer[1] > 0);
          m_do2.set(m_data_buffer[2] > 0);

          // setting bits 0 and 1 of byte 0 of the output data
          output_data[0] = (byte) ((m_data_buffer[1] > 0) ? 1 : 0);
          output_data[0] = (byte) (output_data[0] | (((m_data_buffer[2] > 0) ? 1 : 0) << 1));
          
        }

        //Send a message back to the same device
        // Note that NO REPEAT means it will only send the output once. 
        CANJNI.FRCNetCommCANSessionMuxSendMessage(targetedMessageID.getInt(), output_data, CANJNI.CAN_SEND_PERIOD_NO_REPEAT); 
      

      }

    } catch (edu.wpi.first.hal.can.CANMessageNotFoundException e) {
      return;
      //No CAN message, not a bad thing due to periodicity of messages
    } catch (Exception e) {
      //Other exception, print it out to make sure user sees it
      System.out.println(e.toString());
    }
    return;
  }

}
