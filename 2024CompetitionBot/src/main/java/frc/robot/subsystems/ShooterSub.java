// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.LedSub.LedColour;
import frc.robot.subsystems.LedSub.LedZones;


public class ShooterSub extends SubsystemBase {

  private static Logger m_logger = Logger.getLogger(ShooterSub.class.getName());


  /** Creates a new Shooter. */
  // private final CANSparkMax m_flywheel =
  //new CANSparkMax(Constants.CanIds.kFlywheelL, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_upperFeeder =
      new CANSparkMax(Constants.CanIds.kUpperFeeder, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_lowerFeeder =
      new CANSparkMax(Constants.CanIds.kLowerFeeder, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_pivot =
      new CANSparkMax(Constants.CanIds.kPivot, CANSparkLowLevel.MotorType.kBrushless);
  private final DigitalInput m_hackLimitSwitch = new DigitalInput(Constants.DioIds.kHackIntakeLimitSwitch);

  private final ShuffleboardTab m_shuffleboardTab = Shuffleboard.getTab("Shooter");
  private final GenericEntry m_shooterPivotPosition, m_shooterPivotVelocity, m_shooterPivotPower,
      m_shooterNoteInPosition;

  private final LedSub m_ledSub;

  private boolean[] m_noteSwitches = new boolean[Constants.Shooter.kNumNoteSensors];

  private PIDController m_pivotPID = new PIDController(0.05, 0.0, 0.001);
  private ArmFeedforward m_pivotFeedforward = new ArmFeedforward(0.053, 0.02, 0); // Tuned by finding the max power it ever needs to move (horizontal) and splitting it between static and gravity gain


  public ShooterSub(LedSub ledSub) {

    m_pivotPID.setTolerance(Constants.Shooter.kPivotAngleTolerance);

    // When true, positive power will turn motor backwards, negitive forwards.
    m_upperFeeder.setInverted(false);
    m_lowerFeeder.setInverted(true);
    m_pivot.setInverted(true);

    //m_flywheel.setIdleMode(IdleMode.kCoast);
    m_upperFeeder.setIdleMode(IdleMode.kBrake);
    m_lowerFeeder.setIdleMode(IdleMode.kBrake);
    m_pivot.setIdleMode(IdleMode.kBrake);

    m_upperFeeder.setSmartCurrentLimit(40);
    m_lowerFeeder.setSmartCurrentLimit(40);
    m_pivot.setSmartCurrentLimit(40);

    m_pivot.getEncoder().setVelocityConversionFactor(1.0);
    m_pivot.getEncoder().setPositionConversionFactor(Constants.Shooter.kPivotAngleConversion);

    m_ledSub = ledSub;

    m_shooterPivotPosition = m_shuffleboardTab.add("Shooter Pivot Position", 0).getEntry();
    m_shooterPivotVelocity = m_shuffleboardTab.add("Shooter Pivot velocity", 0).getEntry();
    m_shooterPivotPower = m_shuffleboardTab.add("Shooter Pivot Power", 0).getEntry();
    m_shooterNoteInPosition = m_shuffleboardTab.add("Shooter Note In Position", 0).getEntry();

    init();
  }

  public void init() {
    m_logger.info("Initializing ShooterSub");
    resetPivot();
    spinBothFeeders(0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateShuffleBoard();
    // TODO remove this hack when we have proper sensors
    m_noteSwitches[Constants.Shooter.kNoteSensorAtFlywheel] = !m_hackLimitSwitch.get(); // kSensorAtFlyWheel being used for temperary limit switch
    m_noteSwitches[Constants.Shooter.kNoteSensorNearFlywheel] = m_noteSwitches[Constants.Shooter.kNoteSensorAtFlywheel]; // kNoteSensorNearFlywheel being used for temperary limit switch
    m_noteSwitches[Constants.Shooter.kNoteSensorAtRoller] = m_noteSwitches[Constants.Shooter.kNoteSensorAtFlywheel]; // kNoteSensorNearFlywheel being used for temperary limit switch


    // TODO:  This might be easier to do inside the commands that pivot the shooter
    if(getPivotAngle() == Constants.Shooter.kAngleAmp) {
      m_ledSub.setZoneColour(LedZones.DIAG_SHOOTER_POSITION, LedColour.PURPLE);
    }

    if(getPivotAngle() == Constants.Shooter.kAngleSubwooferSpeaker) {
      m_ledSub.setZoneColour(LedZones.DIAG_SHOOTER_POSITION, LedColour.WHITE);
    }

    // If note wasn't in last time and note is in now
    // Flash green
    // Set orange

    // Index out of bounds in LedSub
    // if(!m_noteWasIn && isNoteAtPosition(Constants.Shooter.kNoteSensorAtFlywheel)) {
    //   m_ledSub.Flash(LedColour.GREEN);
    //   m_ledSub.setZoneColour(LedZones.GAME_PIECE, LedColour.ORANGE);
    //   m_noteWasIn = true;
    // } else if(m_noteWasIn && !isNoteAtPosition(Constants.Shooter.kNoteSensorAtFlywheel)) {
    //   // If note was in and it's no longer in
    //   // Set green
    //   m_ledSub.setZoneColour(LedZones.GAME_PIECE, LedColour.GREEN);
    //   m_noteWasIn = false;
    // }
  }

  private void updateShuffleBoard() {
    //m_shooterFlywheelVelocity.setDouble(getFlywheelVelocity());
    SmartDashboard.putBoolean("Forward Limit", isPivotAtForwardLimit());
    SmartDashboard.putBoolean("Back Limit", isPivotAtReverseLimit());
    m_shooterPivotPosition.setDouble(getPivotAngle());
    m_shooterPivotVelocity.setDouble(getPivotVelocity());
    // m_shooterflywheelPower.setDouble(m_flywheel.get());
    m_shooterPivotPower.setDouble(m_pivot.get());
    m_shooterNoteInPosition.setBoolean(isNoteAtPosition(Constants.Shooter.kNoteSensorAtFlywheel));
  }

  // public void spinFlywheel(double power) {
  // m_flywheel.set(power);
  // }

  public void spinUpperFeeder(double power) {
    m_upperFeeder.set(power);
  }

  public void spinLowerFeeder(double power) {
    m_lowerFeeder.set(power);
  }

  public void spinBothFeeders(double lowerPower, double upperPower) {
    spinLowerFeeder(lowerPower);
    spinUpperFeeder(upperPower);
  }

  public void movePivot(double power) {
    m_pivot.set(power);
  }

  public double getPivotPower() {
    return m_pivot.get();
  }

  public void runPivotControl(double targetAngle) { // Returns true when at position
    //double fixedAngle = MathUtil.clamp(angle, 0.0, 275.0); // Make sure it isn't trying to go to an illegal value

    double pidPower = m_pivotPID.calculate(getPivotAngle(), targetAngle);
    double fedPower = m_pivotFeedforward.calculate(Math.toRadians(getPivotAngle() - 90.0), pidPower); // Feed forward expects 0 degrees as horizontal

    double pivotPower = pidPower + fedPower;
    // TODO: Run pivot motor based on power
    movePivot(pivotPower);
  }

  public boolean isAtPivotAngle() {
    return m_pivotPID.atSetpoint();
  }

  public void resetPivot() {
    m_pivot.getEncoder().setPosition(0);
  }

  public double getPivotAngle() {
    return m_pivot.getEncoder().getPosition();
  }

  public double getPivotVelocity() {
    return m_pivot.getEncoder().getVelocity();
  }

  public boolean isPivotAtReverseLimit() {
    return m_pivot.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed();
  }

  public boolean isPivotAtForwardLimit() {
    return m_pivot.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed();
  }

  public boolean isNoteAtPosition(int noteSensorId) {
    return m_noteSwitches[noteSensorId];
  }
}
