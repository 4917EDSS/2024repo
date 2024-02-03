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

  public final static class ShooterPivotPositionConstants {
    public static final double kSpeakerPosition = 1;
    public static final double kTrapPosition = 2;
    public static final double kAmpPosition = 3;
  }

  public final static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public final static class DriveConstants {
    public static final double kAbsoluteEncoderOffsetFL = -2.292;//-2.224; //-0.358 + 1.892;
    public static final double kAbsoluteEncoderOffsetFR = 0.586;//0.716; //-0.323 + 2.004;
    public static final double kAbsoluteEncoderOffsetBL = 0.337;//1.003; //0.345 - 0.835;
    public static final double kAbsoluteEncoderOffsetBR = 2.787;//3.031; //-0.435 - 3.574;
  }

  public final static class CanIds {
    // Neos for propulsion 
    public final static int kDriveMotorFL = 1;
    public final static int kDriveMotorFR = 3;
    public final static int kDriveMotorBL = 5;
    public final static int kDriveMotorBR = 7;
    // Falcons for turning
    public final static int kSteeringMotorFL = 2;
    public final static int kSteeringMotorFR = 4;
    public final static int kSteeringMotorBL = 6;
    public final static int kSteeringMotorBR = 8;
    // Swerve turning encoders
    public final static int kEncoderFL = 17;
    public final static int kEncoderFR = 13;
    public final static int kEncoderBL = 15;
    public final static int kEncoderBR = 11;

    public final static int kIntakeRollers = 18;
    public final static int kFlywheelL = 19;
    public final static int kFlywheelR = 20;
    public final static int kPivot = 21;
    public final static int kUpperFeeder = 22;
    public final static int kLowerFeeder = 23;
    //public final static int kTransfer = 28;
    public final static int kClimbMotorL = 24;
    public final static int kClimbMotorR = 25;

  }

  public final static class DioIds {
    public final static int kShooterNoteLimit = 0;
    public final static int kIntakeLimitPort = 1;
  }

  public final static class PwmIds {
    public final static int kLedStripPwmPort = 0;
  }

  public final static class VisionConstants {
    public static final double kApriltagOffset = 0.0825; // Apriltag height + bot height (Will need to be changed in the future)
    public static final double kApriltagHeights[] =
        {1.22, 1.22, 1.32, 1.32, 1.22, 1.22, 1.32, 1.32, 1.22, 1.22, 1.24, 1.24, 1.24, 1.24, 1.24, 1.24};

  }
  public static final class ClimbConstants {
    //TODO Change all the hights
    //all meshermintes need to be fixed and are in m
    public static final double kShortHookRaised = 0.40;
    public static final double kHookLowered = 0.01;
    public static final double kTallHookRaised = 0.20;
    public static final double kHookScoring = 0.25;
    /** Convert ticks to meters (Ticks over 80cm) */
    public static final double kTickCofficient = 0.8 / 769.637939453125;
    public static final int kBaudRate = 115200;
    public static final int kBufferSize = 20;
    public static final int kTimeOutLangth = 120;
    public static final int kReadByteLength = 1;
  }
  // public static final class AnalogInIds {
  //   public static final int kFrontDistanceSenor = 0;
  // }
  public static final class AprilTagIds {
    public static final class Blue {
      public static final int kSourceRight = 1;
      public static final int kSourceLeft = 2;
      public static final int kAmp = 6;
      public static final int kCenterStage = 14;
      public static final int kStageLeft = 15;
      public static final int kStageRight = 16;
      public static final int kSpeakerCenter = 7;
      public static final int kSpeakerLeft = 8;
    }
    public static final class Red {
      public static final int kSpeakerRight = 3;
      public static final int kSpeakerCenter = 4;
      public static final int kAmp = 5;
      public static final int kSourceRight = 9;
      public static final int kSourceLeft = 10;
      public static final int kStageLeft = 11;
      public static final int kStageRight = 12;
      public static final int kCenterStage = 13;
    }
  }
}
