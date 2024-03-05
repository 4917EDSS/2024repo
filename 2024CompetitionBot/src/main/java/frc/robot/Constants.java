// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.logging.Level;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;


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
  // Max log level to print (SEVERE, WARNING, INFO, CONFIG, FINE, FINER, or FINEST)
  // e.g. Level.WARNING will only print WARNING and SEVERE log messages
  public static final Level kLogLevel = Level.FINE;


  // Hardware constants
  public final static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
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
    public final static int kClimbMotorL = 24;
    public final static int kClimbMotorR = 25;
  }

  public final static class DioIds {
    public final static int kShooterNoteLimit = 0;
    public final static int kIntakeLimitPort = 1;
    public final static int kHackIntakeLimitSwitch = 9;
  }

  public final static class PwmIds {
    public final static int kLedStripPwmPort = 0;
  }

  public final static class Arduino {
    public static final int kBaudRate = 38400;
    public static final int kBufferSize = 15; //(2 * bitDataLength) - 1
    public static final int kTimeOutLength = 30;
    public static final int kReadByteLength = 15;
    public static final int kByteArrayLength = 5;
  }

  // public static final class AnalogInIds {
  //   public static final int kFrontDistanceSenor = 0;
  // }


  // Values that are specific to a particular physical robot
  public static final class RobotSpecific {
    public static final String PrototypeSerialNumber = "03264244";//"03178417";
    public static final String PracticeSerialNumber = "03147322";
    public static final String CompetitionSerialNumber = "03264244";//"03178417";
    public static final String serialNumber = System.getenv("serialnum");

    public static final class Prototype {
      public static final double kAbsoluteEncoderOffsetFL = 1.881;
      public static final double kAbsoluteEncoderOffsetFR = -2.628;
      public static final double kAbsoluteEncoderOffsetBL = 0.836;
      public static final double kAbsoluteEncoderOffsetBR = -1.715;
    }

    public static final class Practice {
      public static final double kAbsoluteEncoderOffsetFL = 2.359;
      public static final double kAbsoluteEncoderOffsetFR = 1.472;
      public static final double kAbsoluteEncoderOffsetBL = 1.056;
      public static final double kAbsoluteEncoderOffsetBR = -1.072;
    }

    public static final class Competition {
      public static final double kAbsoluteEncoderOffsetFL = 1.881;
      public static final double kAbsoluteEncoderOffsetFR = -2.628;
      public static final double kAbsoluteEncoderOffsetBL = 0.836;
      public static final double kAbsoluteEncoderOffsetBR = -1.715;
    }

    public static final class Unknown {
      public static final double kAbsoluteEncoderOffsetFL = 0.0;
      public static final double kAbsoluteEncoderOffsetFR = 0.0;
      public static final double kAbsoluteEncoderOffsetBL = 0.0;
      public static final double kAbsoluteEncoderOffsetBR = 0.0;
    }
  }


  // Subsystem Constants
  public final static class Vision {
    public static final double kApriltagOffset = 0.0825; // Apriltag height + bot height (Will need to be changed in the future)
    public static final double kApriltagHeights[] =
        {1.22, 1.22, 1.32, 1.32, 1.22, 1.22, 1.32, 1.32, 1.22, 1.22, 1.24, 1.24, 1.24, 1.24, 1.24, 1.24};

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

  public final static class Drivetrain {
    // Measured precicely on Feb 10 kAbsoluteEncoderOffsetFL
    public static final String serialNumber = System.getenv("serialnum");
    // Translation PID, Rotation PID, Max module speed (m/s), Robot radius, default path config
    // TODO: These PIDs should be tuned
    public static final HolonomicPathFollowerConfig kPathingConfig =
        new HolonomicPathFollowerConfig(new PIDConstants(0.5, 0.0, 0.0), new PIDConstants(0.2, 0.0, 0.0), 4.2,
            0.45, new ReplanningConfig());
  }

  public static final class Climb {
    //TODO Change all the heights
    // Heights in meters
    public static final double kHeightShortHookRaised = 0.40;
    public static final double kHeightHookLowered = 0.01;
    public static final double kHeightTallHookRaised = 0.20;
    public static final double kHeightHookScoring = 0.25;
    public static final double kHeightGrabChain = 0.38; // For trap shot when needing to drive back with chain
    public static final double kGoToTrapShot = 0.00; //needs to be found for trapshot

    public static final double kPower = 1.0;

    /** Convert ticks to meters (Ticks over 80cm) */
    public static final double kTickCofficient = 0.4 / 361.945; //0.8 / 769.637939453125

    // Parameters to keep the climb horizontal (using gyro's 'roll' angle)
    public static final double kHeightTolerence = 0.005; // In meters
    public static final double kRollZero = -4.3; // In degrees
    public static final double kRollTolerence = 10; // In degrees
    public static final double kMinRollAngle = kRollZero - kRollTolerence;
    public static final double kMaxRollAngle = kRollZero + kRollTolerence;
  }

  public static final class Shooter {
    public static final int kNumNoteSensors = 4;
    public static final int kNoteSensorAtRoller = 0;
    public static final int kNoteSensorNearRoller = 1;
    public static final int kNoteSensorNearFlywheel = 2;
    public static final int kNoteSensorAtFlywheel = 3;

    public static final double kPivotAngleConversion = 54.5 / 32.856; // Degrees / ticks measured
    public static final double kPivotAngleTolerance = 1.0;

    public static final double kAngleFloorIntake = 0.0;
    public static final double kAngleSourceIntake = 135.0;
    public static final double kAngleSubwooferSpeaker = 24.0;
    public static final double kAngleAmp = 190.0;
    public static final double kAngleTrap = 270.0;
    public static final double kAngleAutoLine = 45.0;
    public static final double kAnglePodium = 58.276;
    public static final double kAngleWingLine = 72.522;
    public static final double kAngleTrapShot = 180.0;//needs to be found for trap shot


    public static final double kNoteLowerIntakePower = 1.0;
    public static final double kNoteUpperIntakePower = kNoteLowerIntakePower * 0.9;

    public static final double kFlyWheelSpeed = 0.5;
    public static final double kNoteLowerAmpShotPower = -1.0;
    public static final double kNoteUpperAmpShotPower = -1.0;
  }

  public static final class Intake {
    public static final double kNoteIntakePower = 1.0;
    public static final double kNoteExpelPower = -1.0;
  }

  public static final class Breakers {
    public static final int kDriveMotorFrontRight = 0;
    public static final int kDriveMotorFrontLeft = 1;
    public static final int kDriveMotorBackRight = 2;
    public static final int kDriveMotorBackLeft = 3;
    public static final int kSteeringMotorFR = 4;
    public static final int kSteeringMotorFL = 5;
    public static final int kSteeringMotorBR = 6;
    public static final int kSteeringMotorBL = 7;


    public static final int kClimbMotorL = 8;
    public static final int kClimbMotorR = 10;

    public static final int kLowerFeeder = 9; //
    public static final int kUpperFeeder = 12; //
    public static final int kPivot = 17; //

    public static final int kFlywheelL = 11;
    public static final int kFlywheelR = 13;


    public static final int kIntakeRollers = 14;

  }

  public static final class Flywheel {
    public static final double kFlywheelShootVelocity = 60;
    public static final double ks = 0.25;
    public static final double kv = 0.125;
    public static final double kFlywheelTolerance = 0.1;
    public static final double kEncoderConversionFactor = 10.0 / 16.5;
    public static final double kVelocityConversionFactor = 1.0 / 60.0;
  }
}
