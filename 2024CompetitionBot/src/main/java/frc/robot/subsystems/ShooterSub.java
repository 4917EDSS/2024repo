// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ShooterSub extends SubsystemBase {
  private static Logger m_logger = Logger.getLogger(ShooterSub.class.getName());


  private final CANSparkMax m_pivot =
      new CANSparkMax(Constants.CanIds.kPivot, CANSparkLowLevel.MotorType.kBrushless);
  private final SparkLimitSwitch m_reverseLimit = m_pivot.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
  private final SparkLimitSwitch m_forwardLimit = m_pivot.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

  private final SparkAbsoluteEncoder m_pivotAbsoluteEncoder = m_pivot.getAbsoluteEncoder(Type.kDutyCycle);

  private final PIDController m_pivotPID = new PIDController(0.035, 0.0, 0.0);
  private double m_targetAngle;
  private boolean m_areWeTryingToHold = false;
  private final ArmFeedforward m_pivotFeedforward = new ArmFeedforward(Constants.Shooter.ks, Constants.Shooter.kg, 0); // Tuned by finding the max power it ever needs to move (horizontal) and splitting it between static and gravity gain

  private final ShuffleboardTab m_shuffleboardTab = Shuffleboard.getTab("Shooter");
  private final GenericEntry m_shooterPivotPosition, m_shooterPivotVelocity, m_shooterPivotPower;


  public ShooterSub() {
    m_shooterPivotPosition = m_shuffleboardTab.add("Pivot Pos", 0).getEntry();
    m_shooterPivotVelocity = m_shuffleboardTab.add("Pivot Vel", 0).getEntry();
    m_shooterPivotPower = m_shuffleboardTab.add("Pivot Power", 0).getEntry();

    init();
  }

  public void init() {
    m_logger.info("Initializing ShooterSub");

    m_pivot.setInverted(true);

    m_pivot.setIdleMode(IdleMode.kBrake);

    m_pivot.setSmartCurrentLimit(40);

    m_pivotAbsoluteEncoder.setPositionConversionFactor(Constants.Shooter.kPivotAngleConversion);
    m_pivotAbsoluteEncoder.setVelocityConversionFactor(1.0);

    m_pivotPID.setTolerance(Constants.Shooter.kPivotAngleTolerance);
  }

  public void setTargetAngle(double position) {
    if(position >= Constants.Shooter.kImpossibleZone) {
      position = 0;
    }
    m_targetAngle = position;
    m_areWeTryingToHold = true;
  }

  public void disableTargetAngle() {
    m_areWeTryingToHold = false;
  }


  @Override
  public void periodic() {
    if(m_areWeTryingToHold) {
      runPivotControl();
    }
    if(isPivotAtReverseLimit() && (Math.abs(getPivotAngle()) > 1.5)) {
      resetPivot();
    }

    // This method will be called once per scheduler run
    updateShuffleBoard();
  }


  private void updateShuffleBoard() {
    m_shooterPivotPosition.setDouble(getPivotAngle());
    m_shooterPivotVelocity.setDouble(getPivotVelocity());
    m_shooterPivotPower.setDouble(m_pivot.get());


    // We want this easily accessible to the drivers so put on SmartDashboard tab
    SmartDashboard.putBoolean("Pivot Fwd Limit", isPivotAtForwardLimit());
    SmartDashboard.putBoolean("Pivot Bck Limit", isPivotAtReverseLimit());

  }

  public void movePivot(double power) {
    if((getPivotAngle() <= 20 || getPivotAngle() >= Constants.Shooter.kImpossibleZone) && power < 0) {
      if(power < -Constants.Shooter.kArmPivotSlowSpeed) {
        power = -Constants.Shooter.kArmPivotSlowSpeed;
      }
    } else if((getPivotAngle() <= 60) && power < 0) {
      if(power < -Constants.Shooter.kArmPivotSlowSpeedPrep) {
        power = -Constants.Shooter.kArmPivotSlowSpeedPrep;
      }
    } else if(getPivotAngle() >= 250 && power > 0) {
      if(power > Constants.Shooter.kArmPivotSlowSpeed) {
        power = Constants.Shooter.kArmPivotSlowSpeed;
      }
    } else if(getPivotAngle() >= 220 && power > 0) {
      if(power > Constants.Shooter.kArmPivotSlowSpeedPrep) {
        power = Constants.Shooter.kArmPivotSlowSpeedPrep;
      }
    }
    m_pivot.set(power);
  }

  public double getPivotPower() {
    return m_pivot.get();
  }

  int resetDelay = 0;


  public void resetPivot() {
    if(resetDelay == 0) {
      // Don't update the zero more than every half second
      resetDelay = 25;
      m_logger.warning(
          "Zeroing pivot encoder. Cur Ticks=" + getPivotAngle() + " CurZero=" + m_pivotAbsoluteEncoder.getZeroOffset()
              + " New=" + (getPivotAngle() + m_pivotAbsoluteEncoder.getZeroOffset()) % 360);
      m_pivotAbsoluteEncoder.setZeroOffset((getPivotAngle() + m_pivotAbsoluteEncoder.getZeroOffset()) % 360);
      m_logger.warning("New zero offset =" + m_pivotAbsoluteEncoder.getZeroOffset());
    }
    resetDelay--;
  }


  public double getPivotAngle() {
    return m_pivotAbsoluteEncoder.getPosition();
  }

  public double getPivotVelocity() {
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

  private void runPivotControl() { // Returns true when at position
    //double fixedAngle = MathUtil.clamp(angle, 0.0, 275.0); // Make sure it isn't trying to go to an illegal value

    double pidPower = m_pivotPID.calculate(getPivotAngle(), m_targetAngle);
    double fedPower = m_pivotFeedforward.calculate(Math.toRadians(getPivotAngle() - 90.0), pidPower); // Feed forward expects 0 degrees as horizontal

    double pivotPower = pidPower + fedPower;
    movePivot(pivotPower);
  }
}
