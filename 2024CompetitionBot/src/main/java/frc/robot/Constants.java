// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public final static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public final static class DriveConstants {
    public static final double kAbsoluteEncoderOffsetFL = -0.358;
    public static final double kAbsoluteEncoderOffsetFR = -0.323;
    public static final double kAbsoluteEncoderOffsetBL = 0.345;
    public static final double kAbsoluteEncoderOffsetBR = -0.435;
  }

  public final static class CanIds {
    public final static int kDriveMotorFL = 1; // NEOs 
    public final static int kDriveMotorFR = 3;
    public final static int kDriveMotorBL = 5;
    public final static int kDriveMotorBR = 7;

    public final static int kSteeringMotorFL = 2; // Falcons 
    public final static int kSteeringMotorFR = 4;
    public final static int kSteeringMotorBL = 6;
    public final static int kSteeringMotorBR = 8;

    public final static int kEncoderFL = 17;
    public final static int kEncoderFR = 13;
    public final static int kEncoderBL = 15;
    public final static int kEncoderBR = 11;
    
    public static final int kIntakeRollers = 25;
    public static final int kGroundIntakeUpDownPivot = 26;
    public final static int kFlywheelMotor = 20;
  }

  public final static class PwmIds {
    public final static int kLedStripPwmPort = 0;
  }
  public final static class VisionConstants {
    public static final double kApriltagOffset = 0.0825; // Apriltag height + bot height (Will need to be changed in the future)
    public static final double kApriltagHeights[] =
        {1.22, 1.22, 1.32, 1.32, 1.22, 1.22, 1.32, 1.32, 1.22, 1.22, 1.24, 1.24, 1.24, 1.24, 1.24, 1.24};
  }

}
