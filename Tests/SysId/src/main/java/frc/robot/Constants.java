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

  public final static class Flywheel {
    public final static double kEncoderConversionFactor = 1.0;
  }
}
