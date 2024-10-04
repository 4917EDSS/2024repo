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
    public final static int kUpperFeeder = 23;
    public final static int kLowerFeeder = 22;
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
    public static final double kTimeOutLength = 0.0;
    public static final int kReadMessageLength = 19;
    public static final int kBufferSize = kReadMessageLength * 2;
    public static final int kSensorDataLength = 16;
    public static final byte kMessageHeader = (byte) 0xA5;
  }

  // public static final class AnalogInIds {
  //   public static final int kFrontDistanceSenor = 0;
  // }

  public static final class Breakers {
    // commented out values are for practice bot
    public static final int kDriveMotorFrontRight = 4; //6
    public static final int kDriveMotorFrontLeft = 11; //19
    public static final int kDriveMotorBackRight = 6; //4
    public static final int kDriveMotorBackLeft = 1; //0
    public static final int kSteeringMotorFR = 5; //5
    public static final int kSteeringMotorFL = 14; //10
    public static final int kSteeringMotorBR = 7; //3
    public static final int kSteeringMotorBL = 15; //18

    public static final int kClimbMotorL = 3; //1
    public static final int kClimbMotorR = 0; //2

    public static final int kLowerFeeder = 12; //12
    public static final int kUpperFeeder = 9; //11
    public static final int kPivot = 13; //17

    public static final int kFlywheelL = 2; //8
    public static final int kFlywheelR = 8; //7

    public static final int kIntakeRollers = 10; //9

    public static final int kLimelightUp = 19; // Not on practice bot
    public static final int kLimelightDown = 18; // Not on practice bot
    public static final int kCancoders = 20; // Not on practice bot //they are on the vrm
    public static final int kVRM = 20; //20
    public static final int kRoboRio = 21; //21
    public static final int kArduino = 23; //23 
    public static final int kRadio = 22; //22


  }


  // Values that are specific to a particular physical robot
  public static final class RobotSpecific {
    public static final String PracticeSerialNumber = "03147322";
    public static final String CompetitionSerialNumber = "03264244";
    public static final String serialNumber = System.getenv("serialnum");

    public static final class Practice {
      public static final double kAbsoluteEncoderOffsetFL = 2.359;
      public static final double kAbsoluteEncoderOffsetFR = 1.472;
      public static final double kAbsoluteEncoderOffsetBL = 1.056;
      public static final double kAbsoluteEncoderOffsetBR = -1.072;
      public static final double kGearRatio = 6.12;
      public static final boolean kInvertLowerFeeder = false;
    }

    public static final class Competition {
      public static final double kAbsoluteEncoderOffsetFL = 1.831;
      public static final double kAbsoluteEncoderOffsetFR = -2.371;
      public static final double kAbsoluteEncoderOffsetBL = 0.984;
      public static final double kAbsoluteEncoderOffsetBR = -0.489;
      public static final boolean kInvertLowerFeeder = true;
      public static final double kGearRatio = 6.05; // THIS IS WRONG, USE WITH CAUTION
    }

    public static final class Unknown {
      public static final double kAbsoluteEncoderOffsetFL = 0.0;
      public static final double kAbsoluteEncoderOffsetFR = 0.0;
      public static final double kAbsoluteEncoderOffsetBL = 0.0;
      public static final double kAbsoluteEncoderOffsetBR = 0.0;
      public static final boolean kInvertLowerFeeder = true;
      public static final double kGearRatio = 0.0;
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

  public static final class ModuleConstants {

    // Maxes
    public static final double kMaxModuleAngularSpeed = 8.0 * Math.PI; // In Radians Per Second
    public static final double kMaxModuleAngularAcceleration = 100.0 * Math.PI; // In Radians Per Second Squared

    // Conversion factors
    public static final double kWheelBaseDiameter = 0.1016; // Meters
  }

  public final static class Drivetrain {
    public static final double kMaxChassisSpeed = 4.2;
    // Measured precicely on Feb 10 kAbsoluteEncoderOffsetFL
    public static final String serialNumber = System.getenv("serialnum");
    // Translation PID, Rotation PID, Max module speed (m/s), Robot radius, default path config
    // TODO ONCMP
    // 4.2 here is the "max speed". We should use the exact same number as in DrivetrainSub, we should probably 
    // use the actual same constant. Then, we should update the global constraints in PathPlanner, and
    // make the max speed the same as this #. ALSO, we should tune the PathPlanner acceleration, to be higher, basically
    // as high as we can track with little error.
    public static final HolonomicPathFollowerConfig kPathingConfig =
        new HolonomicPathFollowerConfig(new PIDConstants(7, 0.0, 0.0), new PIDConstants(3.5, 0.0, 0.0),
            kMaxChassisSpeed, //7 for translation P, 6 for rotation p, 0.25 for rotation D, 0.5 for translation D
            0.54, new ReplanningConfig());
    public static final double kGyroPhysicalOffsetAngle = -90; // Gyro's 0 is pointing at this angle (e.g. -90 is pointing right)
  }

  public static final class Climb {
    // Heights in meters
    public static final double kHeightHookLowered = 0.00; // Retracted hooks height
    public static final double kHeightTallHookRaised = 0.20; // Minimum hook height, climb in centre
    public static final double kHeightShortHookRaised = 0.66; // Maximum hook height, short trap hooks or tall hooks with a side climb
    public static final double kHeightGrabChain = 0.38; // For trap shot when needing to drive back with chain
    public static final double kHeightTrap = 0.00; // For trap shot when expelling note

    public static final double kPower = 1.0;

    /** Convert ticks to meters (Ticks over 80cm) */
    public static final double kPositionConversionFactorL =
        (Constants.Drivetrain.serialNumber.equals(Constants.RobotSpecific.CompetitionSerialNumber)) ? 0.40 / 271.9804 // for competition robot
            : 0.4 / 361.945; // for any other ones;
    public static final double kPositionConversionFactorR =
        (Constants.Drivetrain.serialNumber.equals(Constants.RobotSpecific.CompetitionSerialNumber)) ? 0.40 / 262.7435 // for competition robot
            : 0.4 / 361.945; // for any other ones;

    // Parameters to keep the climb horizontal (using gyro's 'roll' angle)
    public static final double kHeightTolerence = 0.01; // In meters
    public static final double kResetHeightTolerence = 0.005; // In meters
    public static final double kRollZero = 0.0; // In degrees
    public static final double kRollTolerence = 10; // In degrees
    public static final double kMinRollAngle = kRollZero - kRollTolerence;
    public static final double kMaxRollAngle = kRollZero + kRollTolerence;
  }

  public static final class Shooter {
    public static final int kNumNoteSensors = 8;
    public static final int kNoteSensorFwNear = 0;
    public static final int kNoteSensorFwMid = 1;
    public static final int kNoteSensorFwFar = 2;
    public static final int kNoteSensorCentreFW = 3;
    public static final int kNoteSensorCentreIntake = 4;
    public static final int kNoteSensorIntakeFar = 5;
    public static final int kNoteSensorIntakeMid = 6;
    public static final int kNoteSensorIntakeNear = 7;

    public static final double ks = 0;//0.037;
    public static final double kg = 0.018;

    // Angles are in degrees
    public static final double kPivotAngleConversion = 360; // Radians to Degrees
    public static final double kPivotAngleTolerance = 0.75;
    public static final double kPivotRolloverAngle = 243.4;

    public static final double kAngleFloorIntake = 0.0;
    public static final double kAngleSourceIntake = 217.0;
    public static final double kAngleSubwooferSpeaker = 37.95;
    public static final double kAngleAmp = 215.0;
    public static final double kAnglePreAmp = 200.0;
    public static final double kAngleTrap = 270.0;
    public static final double kAngleAutoLine = 57.0;
    public static final double kAngleAutoLineAmp = 33.0;
    public static final double kAnglePassing = 52.0;
    public static final double kAngleWingLine = 72.522;
    public static final double kAngleTrapShot = 180.0;//needs to be found for trap shot
    public static final double kAngleZero = 0.0;
    public static final double kHighPickUp = 212.0;

    public static final double kNoteUpperSurfaceSpeedDifferential = 0.9;

    public static final double kNoteLowerIntakePower = -1.0;
    public static final double kNoteUpperIntakePower = kNoteLowerIntakePower * kNoteUpperSurfaceSpeedDifferential;

    public static final double kNoteLowerExpelPower = 0.3;
    public static final double kNoteUpperExpelPower = kNoteLowerExpelPower * kNoteUpperSurfaceSpeedDifferential;

    public static final double kLowerExpelPower = 0.5;
    public static final double kUpperExpelPower = 0.5;

    public static final double kLowerExpelPowerAmpShot = 0.4; //bigger
    public static final double kUpperExpelPowerAmpShot = 0.4;

    public static final double kNoteLowerAmpShotPower = -1.0;
    public static final double kNoteUpperAmpShotPower = -1.0;

    //limits speed for a soft stop
    public static final double kArmPivotSlowSpeed = 0.2;
    public static final double kArmPivotSlowSpeedPrep = 0.3;
    public static final double kArmPivotSlowSpeedPrepBefore = 0.6;
    public static final double kImpossibleZone = 300;

    //for joystick movment
    public static final double kMovingCoifficant = 7 / 0.2;
  }

  public static final class Intake {
    public static final double kNoteIntakePower = 1.0;
    public static final double kNoteExpelPower = -1.0;
  }

  public static final class Flywheel {
    public static final double kFlywheelShootVelocity = 70.0; // 76 is MAX velocity in RPS. Set a little below for when battery weakens
    public static final double kFlywheelShootAmpVelocity = 75.0;
    public static final double kFlywheelLobVelocity = 42.0;
    public static final double ks = 0.5;
    public static final double kv = 0.131; // ~0.95 per 0.125 V.  Was 0.128 with old gearing
    //public static final double kStaticThresholdVelocity = 1.6; // RPS when we apply ks volts
    public static final double kFlywheelTolerance = 1;
    public static final double kPositionConversionFactor = 10.0 / 9.524; // Was 10.0 / 16.5 with previous gearing
    public static final double kVelocityConversionFactor = 1.0 / 60.0; // RPM to RPS
  }
}
