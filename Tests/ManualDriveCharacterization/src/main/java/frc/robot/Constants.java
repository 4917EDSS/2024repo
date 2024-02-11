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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class CanIds {
    // Neos for propulsion 
    public static final int kDriveMotorFL = 1;
    public static final int kDriveMotorFR = 3;
    public static final int kDriveMotorBL = 5;
    public static final int kDriveMotorBR = 7;
    // Falcons for turning
    public static final int kSteeringMotorFL = 2;
    public static final int kSteeringMotorFR = 4;
    public static final int kSteeringMotorBL = 6;
    public static final int kSteeringMotorBR = 8;
    // Swerve turning encoders
    public static final int kEncoderFL = 17;
    public static final int kEncoderFR = 13;
    public static final int kEncoderBL = 15;
    public static final int kEncoderBR = 11;

    public static final int kIntakeRollers = 18;
    public static final int kFlywheelL = 19;
    public static final int kFlywheelR = 20;
    public static final int kPivot = 21;
    public static final int kUpperFeeder = 22;
    public static final int kLowerFeeder = 23;
    public static final int kClimbMotorL = 24;
    public static final int kClimbMotorR = 25;
  }

  public static final class Drivetrain {
    public static final int driveCurrentLimit = 80;

    // In theory:
    // TicksPerMotorRotation? * L3GearRation / 4"WheelCircumference
    // 2048 * 6.12 / (4in * 0.0254m/in * PI) = 39267.94 ticks per m
    // Actual: 55.548 for 3m
    public static final double kDriveDistanceFactor = 3.0 / 55.548;
    public static final double kDriveVelocityFactor = kDriveDistanceFactor / 60.0; // RPM to m/s

    public static final double kSteeringKP = 0.01;
    public static final double kSteeringKI = 0.0;
    public static final double kSteeringKD = 0.0;
    public static final double kSteeringKFF = 0.0;

    // Measured precicely on Feb 11 - but the modules drift (something loose)
    // Used aluminum square tube against the non-cog side of the wheels
    public static final double kAbsoluteEncoderOffsetFL = -2.028;
    public static final double kAbsoluteEncoderOffsetFR = -2.634;
    public static final double kAbsoluteEncoderOffsetBL = -0.463;
    public static final double kAbsoluteEncoderOffsetBR = -0.851;
  }
}
