// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CanSub extends SubsystemBase {
  byte m_data_buffer[];


  /** Creates a new CanSub. */
  public CanSub() 
  {
    m_data_buffer = new byte[8];


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  int createCANId(short apiId, byte deviceId, byte manufacturer, byte deviceType)
  {
    return ((int)(deviceType) & 0x1F) << 24 | ((int)(manufacturer) & 0xFF) << 16 | (apiId & 0x3FF) << 6 | (deviceId & 0x3F);
  }

  void UpdateCustomSensor(int sensorID)
	{
		ByteBuffer targetedMessageID = ByteBuffer.allocateDirect(4);//Must be direct
		targetedMessageID.order(ByteOrder.LITTLE_ENDIAN); //Set order of bytes
		targetedMessageID.asIntBuffer().put(0, 0x1E040000); //Put the arbID into the buffer
		ByteBuffer timeStamp = ByteBuffer.allocateDirect(4); //Allocate memory for time stamp

		try
		{
			//Return call is data, selection is assigned
			m_data_buffer = CANJNI.FRCNetCommCANSessionMuxReceiveMessage(targetedMessageID.asIntBuffer(), 0xFFFFFFFF, timeStamp);
			//Send back the acknowledgement of selection
			//CANJNI.FRCNetCommCANSessionMuxSendMessage(0x1E040001, currentS, 100); 
			
		}
		catch(edu.wpi.first.hal.can.CANMessageNotFoundException e)
		{ 
			return;
			//No CAN message, not a bad thing due to periodicity of messages
		}
		catch(Exception e)
		{
			//Other exception, print it out to make sure user sees it
			System.out.println(e.toString());
		}
		return;
	}

}
