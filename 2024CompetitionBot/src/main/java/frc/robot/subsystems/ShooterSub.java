// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


// TODO PE# - These are the steps to selectively include the changes made on Wednesday (when we didn't know how the
// encoder worked)
public class ShooterSub extends SubsystemBase {
  private static Logger m_logger = Logger.getLogger(ShooterSub.class.getName());

  private final CANSparkMax m_upperFeeder =
      new CANSparkMax(Constants.CanIds.kUpperFeeder, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_lowerFeeder =
      new CANSparkMax(Constants.CanIds.kLowerFeeder, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_pivot =
      new CANSparkMax(Constants.CanIds.kPivot, CANSparkLowLevel.MotorType.kBrushless);
  private final SparkLimitSwitch m_reverseLimit = m_pivot.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
  private final SparkLimitSwitch m_forwardLimit = m_pivot.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

  // TODO PE1 - Create new absolute encoder:  private final SparkAbsoluteEncoder m_pivotAbsoluteEncoder = m_pivot.getAbsoluteEncoder(Type.kDutyCycle);
  private final SparkAbsoluteEncoder m_pivotAbsoluteEncoder = m_pivot.getAbsoluteEncoder(Type.kDutyCycle);
  private final DigitalInput m_hackLimitSwitch = new DigitalInput(Constants.DioIds.kHackIntakeLimitSwitch); // TODO: Remove when Arduino board works

  private final PIDController m_pivotPID = new PIDController(0.05, 0.0, 0.001);
  private final ArmFeedforward m_pivotFeedforward = new ArmFeedforward(0.053, 0.02, 0); // Tuned by finding the max power it ever needs to move (horizontal) and splitting it between static and gravity gain

  private final ShuffleboardTab m_shuffleboardTab = Shuffleboard.getTab("Shooter");
  private final GenericEntry m_shooterPivotPosition, m_shooterPivotVelocity, m_shooterPivotPower,
      m_shooterNoteInPosition;

  private boolean[] m_noteSwitches = new boolean[Constants.Shooter.kNumNoteSensors]; // TODO: Remove when Arduino board works

  private double m_lastPivotAngle = 0.0;
  private double m_currentRolloverAngleOffset = 0.0;


  public ShooterSub() {
    m_shooterPivotPosition = m_shuffleboardTab.add("Pivot Pos", 0).getEntry();
    m_shooterPivotVelocity = m_shuffleboardTab.add("Pivot Vel", 0).getEntry();
    m_shooterPivotPower = m_shuffleboardTab.add("Pivot Power", 0).getEntry();
    m_shooterNoteInPosition = m_shuffleboardTab.add("Note In", 0).getEntry();

    init();
  }

  public void init() {
    m_logger.info("Initializing ShooterSub");

    m_upperFeeder.setInverted(false);
    if(Constants.Drivetrain.serialNumber.equals(Constants.RobotSpecific.PracticeSerialNumber)) {
      m_lowerFeeder.setInverted(Constants.RobotSpecific.Practice.kInvertLowerFeeder);
    } else if(Constants.Drivetrain.serialNumber.equals(Constants.RobotSpecific.CompetitionSerialNumber)) {
      m_lowerFeeder.setInverted(Constants.RobotSpecific.Competition.kInvertLowerFeeder);
    } else {
      m_lowerFeeder.setInverted(Constants.RobotSpecific.Unknown.kInvertLowerFeeder);
    }

    m_pivot.setInverted(true);

    m_upperFeeder.setIdleMode(IdleMode.kBrake);
    m_lowerFeeder.setIdleMode(IdleMode.kBrake);
    m_pivot.setIdleMode(IdleMode.kBrake);

    m_upperFeeder.setSmartCurrentLimit(40);
    m_lowerFeeder.setSmartCurrentLimit(40);
    m_pivot.setSmartCurrentLimit(40);

    m_pivotAbsoluteEncoder.setPositionConversionFactor(Constants.Shooter.kPivotAngleConversion); // TODO PE2 - Replace m_pivot.getEncoder(). with m_pivotAbsoluteEncoder.
    m_pivotAbsoluteEncoder.setVelocityConversionFactor(1.0); // TODO PE3 - Replace m_pivot.getEncoder(). with m_pivotAbsoluteEncoder.

    m_pivotPID.setTolerance(Constants.Shooter.kPivotAngleTolerance);

    //resetPivot(); // TODO PE4 - Remove this line.  We'll reset the encoder only when we hit the lower limit switch (and if necessary)
    spinBothFeeders(0, 0);
  }

  boolean m_forwardDirection = false;
  boolean m_backwardDirection = false;

  @Override
  public void periodic() {
    // Reset the pivot encoder if needed - we're at the reverse limit (angle 0) and the encoder
    // is off by more than 1 degree
    // TODO PE8 
    // - Enable the following code after the encoder is set to read in degrees
    // - Test to make sure it doesn't break anything (moving pivot up and down)
    if(isPivotAtReverseLimit()) { //&& (Math.abs(getPivotAngle()) > 1.0)) {

      resetPivot();
    }

    // If the angle since last time has changed by more than 10 degrees, it's because the encoder rolled over
    // TODO PE9
    // - Measure the angle at which the encoder rolls over back to 0 and update kPivotRolloverAngle to match
    // - Keep track of rollovers of the absolute encoder by adding code like this
    double currentAngle = getPivotAngle();
    //System.out.println("curentangle " + currentAngle + " last pivot angle " + m_lastPivotAngle + " roolover offset "
    //   + m_currentRolloverAngleOffset);

    if(currentAngle != m_lastPivotAngle) {

      System.out.println(" ++++ " +
          (Math.abs(currentAngle - Constants.Shooter.kPivotRolloverAngle) < 10) + " | " +
          m_forwardDirection + " | " +
          m_backwardDirection + " | " +
          (m_currentRolloverAngleOffset > Constants.Shooter.kPivotRolloverAngle)


      );

      if((currentAngle - m_lastPivotAngle) > 10) {
        m_forwardDirection = true;
        m_backwardDirection = false;
      } else if((m_lastPivotAngle - currentAngle) > 10) {

        m_forwardDirection = false;
        m_backwardDirection = true;
      }


      if((Math.abs(currentAngle - Constants.Shooter.kPivotRolloverAngle) < 10)
          && (m_currentRolloverAngleOffset < Constants.Shooter.kPivotRolloverAngle)
          && (m_forwardDirection == true)) {
        // if(m_lastPivotAngle < currentAngle) {
        //}
        // Rolled over from larger angle to 0, add offset
        m_currentRolloverAngleOffset += Constants.Shooter.kPivotRolloverAngle;
        System.out.println(
            ">>>>>> curentangle " + currentAngle + " last pivot angle " + m_lastPivotAngle + " roolover offset "
                + m_currentRolloverAngleOffset);


        // } else if((Math.abs(currentAngle - Constants.Shooter.kPivotRolloverAngle) < 10)
        //     && (m_angleOffsetCalculated == true)
        //     && (m_currentRolloverAngleOffset < Constants.Shooter.kPivotRolloverAngle)) {

      } else if((Math.abs(currentAngle - Constants.Shooter.kPivotRolloverAngle) < 10)
          && (m_backwardDirection == true)) {


        System.out
            .println("<<<<< curentangle " + currentAngle + " last pivot angle " + m_lastPivotAngle + " roolover offset "
                + m_currentRolloverAngleOffset);
        //   // Rolled over from smaller to larger angle, remove offset
        m_currentRolloverAngleOffset -= Constants.Shooter.kPivotRolloverAngle;

      }

    }

    //   //update only if the angle is different
    m_lastPivotAngle = currentAngle;


    // This method will be called once per scheduler run
    updateShuffleBoard();

    // TODO remove this hack when we have proper sensors
    m_noteSwitches[Constants.Shooter.kNoteSensorAtFlywheel] = !m_hackLimitSwitch.get(); // kSensorAtFlyWheel being used for temperary limit switch
    m_noteSwitches[Constants.Shooter.kNoteSensorNearFlywheel] = m_noteSwitches[Constants.Shooter.kNoteSensorAtFlywheel]; // kNoteSensorNearFlywheel being used for temperary limit switch
    m_noteSwitches[Constants.Shooter.kNoteSensorAtRoller] = m_noteSwitches[Constants.Shooter.kNoteSensorAtFlywheel]; // kNoteSensorNearFlywheel being used for temperary limit switch

  }

  private void updateShuffleBoard() {
    m_shooterPivotPosition.setDouble(getPivotAngle());
    m_shooterPivotVelocity.setDouble(getPivotVelocity());
    m_shooterPivotPower.setDouble(m_pivot.get());
    m_shooterNoteInPosition.setBoolean(isNoteAtPosition(Constants.Shooter.kNoteSensorAtFlywheel));

    // We want this easily accessible to the drivers so put on SmartDashboard tab
    SmartDashboard.putBoolean("Pivot Fwd Limit", isPivotAtForwardLimit());
    SmartDashboard.putBoolean("Pivot Bck Limit", isPivotAtReverseLimit());
    SmartDashboard.putNumber("last pivot angle", m_lastPivotAngle);
  }

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

  public void resetPivot() {
    m_logger.warning("Zeroing pivot encoder");

    // TODO PE5 - Replace the code below with:  
    //m_pivotAbsoluteEncoder.setZeroOffset(getPivotAngle() + m_pivotAbsoluteEncoder.getZeroOffset());
    m_pivotAbsoluteEncoder.setZeroOffset(getPivotAngle() + m_pivotAbsoluteEncoder.getZeroOffset());
    m_lastPivotAngle = getPivotAngle();
  }

  public double getPivotAngle() {
    // TODO PE6
    // - Replace m_pivot.getEncoder(). with m_pivotAbsoluteEncoder.
    // - In Constants.java, set kPivotAngleConversion to 1.0 so we can figure out the conversion factor for this new encoder

    // TODO PE10
    // - Add + m_currentRolloverAngleOffset after getPosition() to account for absolute encoder rollovers
    return m_pivotAbsoluteEncoder.getPosition() + m_currentRolloverAngleOffset;
  }

  public double getPivotVelocity() {
    // TODO PE7 
    // - Replace m_pivot.getEncoder(). with m_pivotAbsoluteEncoder. in the code below
    // - Move the shooter to the 0 position.
    // - Connect using the Rev Hardware software and zero the encoder (save the zero to flash)
    // - Deploy this code
    // - Move the pivot to around 180 degrees
    // - Update kPivotAngleConversion using the degrees you measured divided by the 'Pivot Pos' value in Shuffleboard > Shooter
    // - Deploy the updated code 
    // - Move the pivot and check that 'Pivot Pos' now shows degrees (if not, try inverting the math for kPivotAngleConversion)
    // - Go to TODO PE8
    return m_pivotAbsoluteEncoder.getVelocity();
  }

  public boolean isPivotAtReverseLimit() {
    return m_reverseLimit.isPressed();
  }

  public boolean isPivotAtForwardLimit() {
    return m_forwardLimit.isPressed();
  }

  public boolean isAtPivotAngle() {
    return m_pivotPID.atSetpoint();
  }

  public void runPivotControl(double targetAngle) { // Returns true when at position
    //double fixedAngle = MathUtil.clamp(angle, 0.0, 275.0); // Make sure it isn't trying to go to an illegal value

    double pidPower = m_pivotPID.calculate(getPivotAngle(), targetAngle);
    double fedPower = m_pivotFeedforward.calculate(Math.toRadians(getPivotAngle() - 90.0), pidPower); // Feed forward expects 0 degrees as horizontal

    double pivotPower = pidPower + fedPower;
    // TODO: Run pivot motor based on power
    movePivot(pivotPower);
  }

  // TODO:  Remove when Arduino board is working
  public boolean isNoteAtPosition(int noteSensorId) {
    return m_noteSwitches[noteSensorId];
  }
}
