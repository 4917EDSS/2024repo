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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class PivotSub extends SubsystemBase {
  private static Logger m_logger = Logger.getLogger(PivotSub.class.getName());


  private final CANSparkMax m_pivot =
      new CANSparkMax(Constants.CanIds.kPivot, CANSparkLowLevel.MotorType.kBrushless);
  private final SparkLimitSwitch m_reverseLimit = m_pivot.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
  private final SparkLimitSwitch m_forwardLimit = m_pivot.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

  private final SparkAbsoluteEncoder m_pivotAbsoluteEncoder = m_pivot.getAbsoluteEncoder(Type.kDutyCycle);

  private final PIDController m_pivotPID = new PIDController(0.02, 0.0, 0.0); //P used to be 0.3, changed to improve oscillations. PID not perfect, could still use tuning.
  private double m_targetAngle;
  private boolean m_areWeTryingToHold = false;
  private final ArmFeedforward m_pivotFeedforward = new ArmFeedforward(Constants.Shooter.ks, Constants.Shooter.kg, 0); // Tuned by finding the max power it ever needs to move (horizontal) and splitting it between static and gravity gain

  private final ShuffleboardTab m_shuffleboardTab = Shuffleboard.getTab("Shooter");
  private final GenericEntry m_shooterPivotVelocity, m_shooterPivotPower;

  // -13 degrees to 2 degrees
  //private static final int kMinLimelightAngle = -13;
  //private static final int kMaxLimelightAngle = 2;

  private static final double kA = 1.41;
  private static final double kB = 1.05;
  private static final double kC = 0.06;
  private static final double kD = 0.34;
  private static final double kLimelightAngle = 30.0; // degrees


  private static final double kMinLimelightAngle = -17.0;
  private static final double kMaxLimelightAngle = 15.0;
  // -19 to 13 degrees
  // @formatter:off
  private final double[] limelightAngles = {kMinLimelightAngle, -14.9, -12.9, -11.2, -8.90, -0.40, 5.20, 11.9, 14.0, kMaxLimelightAngle}; //
  private final double[] shooterAngles = {                70.4,  69.8,  67.9,  66.3,  64.0,  57.3, 52.0, 45.7, 42.5, 40.0}; //
  // @formatter:on

  private int m_hitLimitCounter = 0;

  public PivotSub() {
    m_shooterPivotVelocity = m_shuffleboardTab.add("Pivot Vel", 0).getEntry();
    m_shooterPivotPower = m_shuffleboardTab.add("Pivot Power", 0).getEntry();

    init();
  }

  public void init() {
    m_logger.info("Initializing pivotSub");

    disableTargetAngle();

    m_pivot.setInverted(false);

    m_pivot.setIdleMode(IdleMode.kBrake);

    m_pivot.setSmartCurrentLimit(40);

    m_pivotAbsoluteEncoder.setPositionConversionFactor(Constants.Shooter.kPivotAngleConversion);
    m_pivotAbsoluteEncoder.setVelocityConversionFactor(1.0);
    m_pivot.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 20);

    m_pivotPID.setTolerance(Constants.Shooter.kPivotAngleTolerance);
  }

  public void setTargetAngle(double position) {
    if(position >= Constants.Shooter.kImpossibleZone) {
      position = 0;
    }
    m_targetAngle = position;
    m_areWeTryingToHold = true;
    runPivotControl(true);
  }

  public void disableTargetAngle() {
    m_areWeTryingToHold = false;
  }


  @Override
  public void periodic() {
    if(m_areWeTryingToHold) {
      runPivotControl(false);
    }
    if(isPivotAtReverseLimit() && Math.abs(getPivotAngle()) > 0.15) {
      m_hitLimitCounter++;
    } else {
      m_hitLimitCounter = 0;
    }

    if(m_hitLimitCounter >= 5) {
      resetPivot();
      m_hitLimitCounter = 0;
    }

    // This method will be called once per scheduler run
    updateShuffleBoard();
  }

  public double calcShooterAngle(double limelightAngle) {
    double x = kA / ((kB / Math.tan(Math.toRadians(limelightAngle + kLimelightAngle))) + kC - kD);
    double v = Math.toDegrees(Math.atan(x));
    return 90.0 - v;
  }

  public double interpolateShooterAngle(double limelightAngle) {
    double shooterAngle = calcShooterAngle(limelightAngle);
    if(limelightAngle > kMinLimelightAngle && limelightAngle < kMaxLimelightAngle) { // Range of inaccuracy
      int minNum = 0; // lower interpolation value
      int maxNum = 1; // upper interpolation
      // find values limelight angle is between
      for(int i = 0; i < limelightAngles.length - 1; i++) {
        if(limelightAngle < limelightAngles[i + 1]) {
          minNum = i;
          maxNum = i + 1;
          break;
        }
      }

      double interpolatedT = (limelightAngle - limelightAngles[minNum])
          / (limelightAngles[maxNum] - limelightAngles[minNum]);
      shooterAngle = MathUtil.interpolate(shooterAngles[minNum], shooterAngles[maxNum], interpolatedT);
    }
    return shooterAngle;
  }

  private void updateShuffleBoard() {
    m_shooterPivotVelocity.setDouble(getPivotVelocity());
    m_shooterPivotPower.setDouble(m_pivot.get());

    // We want this easily accessible to the drivers so put on SmartDashboard tab
    SmartDashboard.putBoolean("Pivot Fwd Limit", isPivotAtForwardLimit());
    SmartDashboard.putBoolean("Pivot Bck Limit", isPivotAtReverseLimit());
    SmartDashboard.putNumber("Pivot Angle", getPivotAngle());
  }

  public void movePivot(double power) {
    // if((getPivotAngle() <= 20 || getPivotAngle() >= Constants.Shooter.kImpossibleZone) && power < 0) {
    //   if(power < -Constants.Shooter.kArmPivotSlowSpeed) {
    //     power = -Constants.Shooter.kArmPivotSlowSpeed;
    //   }
    // } else if((getPivotAngle() <= 40) && power < 0) {
    //   if(power < -Constants.Shooter.kArmPivotSlowSpeedPrep) {
    //     power = -Constants.Shooter.kArmPivotSlowSpeedPrep;
    //   }
    // } else if((getPivotAngle() <= 60) && power < 0) {
    //   if(power < -Constants.Shooter.kArmPivotSlowSpeedPrepBefore) {
    //     power = -Constants.Shooter.kArmPivotSlowSpeedPrepBefore;
    //   }
    // } else
    if(getPivotAngle() <= 20 && power < 0) {
      double testPower = getPivotAngle() / 20;
      if(testPower < Constants.Shooter.kArmPivotSlowSpeed) {
        testPower = Constants.Shooter.kArmPivotSlowSpeed;
      }
      if(power < -testPower) {
        power = -testPower;
      }
    }
    if(getPivotAngle() >= 250 && power > 0) {
      if(power > Constants.Shooter.kArmPivotSlowSpeed) {
        power = Constants.Shooter.kArmPivotSlowSpeed;
      }
    } else if(getPivotAngle() >= 230 && power > 0) {
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
    m_pivotAbsoluteEncoder.setZeroOffset((getPivotAngle() + m_pivotAbsoluteEncoder.getZeroOffset()) % 360);
    m_logger.fine("Pivot Reset");
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

  public void runPivotControl(boolean justCalculate) {
    double pidPower = m_pivotPID.calculate(getPivotAngle(), m_targetAngle);
    double fedPower = m_pivotFeedforward.calculate(Math.toRadians(getPivotAngle() - 90.0), pidPower); // Feed forward expects 0 degrees as horizontal

    if(!justCalculate) {
      double pivotPower = pidPower + fedPower;
      movePivot(pivotPower);
    }
  }
}
